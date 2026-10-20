"""Entry point: python -m src.main --config config/default.yaml [--headless]

Simulation only. Nothing here connects to or commands a real vehicle.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import time
from pathlib import Path

from src.config import Config, ConfigError, load_config
from src.sim.world import World

SPEEDS = (0.25, 0.5, 1.0, 2.0, 4.0, 8.0)
WINDOW = (1440, 860)
MAP_W = 920
FPS = 60


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Multi-drone decentralized target inspection (simulation)")
    ap.add_argument("--config", default="config/default.yaml", help="scenario YAML file")
    ap.add_argument("--headless", action="store_true", help="run without the dashboard (overrides sim.headless)")
    ap.add_argument("--seed", type=int, help="override sim.seed")
    ap.add_argument("--duration", type=float, help="override sim.duration (s)")
    ap.add_argument("--output", default="output", help="output directory")
    ap.add_argument("--run-id", help="run directory name (default: <scenario>_seed<seed>_<timestamp>)")
    return ap.parse_args(argv)


def apply_overrides(cfg: Config, args: argparse.Namespace) -> Config:
    sim = cfg.sim
    if args.seed is not None:
        sim = dataclasses.replace(sim, seed=args.seed)
    if args.duration is not None:
        if args.duration <= 0:
            raise ConfigError("--duration must be > 0")
        sim = dataclasses.replace(sim, duration=args.duration)
    if args.headless:
        sim = dataclasses.replace(sim, headless=True)
    return dataclasses.replace(cfg, sim=sim)


def write_outputs(world: World, out_root: Path, run_id: str) -> Path:
    out = out_root / run_id
    world.log.write(out / "events.jsonl")
    summary = {
        "scenario": world.cfg.name,
        "seed": world.cfg.sim.seed,
        "sim_time": world.t,
        "ticks": world.tick,
        "events": len(world.log.records),
        "event_log_sha256": world.log.digest(),
        "geofence_violations": len(world.geofence_violations),
        "false_confirmations": world.false_confirmations(),
        "separation_violations": [list(v) for v in world.separation_violations],
        "messages": dict(world.network.stats),
        "messages_by_type": dict(sorted(world.network.sent_by_type.items())),
        "allocations": [{k: v for k, v in r.items() if k != "baseline_bids"} for r in world.baseline_results()],
        "targets": {
            d.id: [{"id": tv.id, "state": tv.state, "confirmation": tv.confirmation, "est_pos": [round(v, 2) for v in tv.est_pos],
                    "observations": tv.n_obs} for tv in d.targets]
            for d in world.snapshot.drones
        },
        "drones": {
            d.id: {"mode": d.mode, "battery_pct": round(d.battery_pct, 4), "pos": [round(d.x, 2), round(d.y, 2)],
                   "alt": round(d.alt, 2), "docked": d.docked}
            for d in world.snapshot.drones
        },
    }  # fmt: skip
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return out


def run_headless(world: World) -> None:
    world.run()


def run_gui(world_factory) -> World:
    import pygame

    from src.viz.panel import Panel
    from src.viz.renderer import MapRenderer, UiState

    if sys.platform == "win32":
        # Without this, Windows display scaling (125 %, 150 %) blows the window up past the screen edges.
        try:
            import ctypes

            ctypes.windll.user32.SetProcessDPIAware()
        except (AttributeError, OSError):
            pass
    pygame.init()
    pygame.display.set_caption("Multi-drone inspection (simulation)")
    # SCALED: the dashboard is drawn at 1440 x 860 and scaled to fit the screen (mouse clicks are mapped back);
    # RESIZABLE: the window can be resized or maximised.
    screen = pygame.display.set_mode(WINDOW, pygame.SCALED | pygame.RESIZABLE)
    fonts = {
        "small": pygame.font.Font(None, 18),
        "normal": pygame.font.Font(None, 22),
        "title": pygame.font.Font(None, 26),
        "mono": pygame.font.SysFont("dejavusansmono,couriernew,monospace", 14),
        "mono_small": pygame.font.SysFont("dejavusansmono,couriernew,monospace", 12),
    }
    world: World = world_factory()
    renderer = MapRenderer(pygame.Rect(0, 0, MAP_W, WINDOW[1]), world.cfg.sim.map_size)
    panel = Panel(pygame.Rect(MAP_W, 0, WINDOW[0] - MAP_W, WINDOW[1]))
    ids = [d.id for d in world.snapshot.drones]
    ui = UiState(selected=ids[0] if ids else None)
    speed_idx = SPEEDS.index(1.0)
    clock = pygame.time.Clock()
    sim_debt = 0.0  # simulated seconds owed to keep up with wall-clock time x speed

    running = True
    while running:
        wall_dt = clock.tick(FPS) / 1000.0
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_SPACE:
                    ui.paused = not ui.paused
                elif ev.key == pygame.K_RIGHT:
                    speed_idx = min(speed_idx + 1, len(SPEEDS) - 1)
                elif ev.key == pygame.K_LEFT:
                    speed_idx = max(speed_idx - 1, 0)
                elif ev.key == pygame.K_r:
                    world = world_factory()
                    renderer.reset()
                    ui.targets_hidden = False
                    sim_debt = 0.0
                elif ev.key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4):
                    i = ev.key - pygame.K_1
                    if i < len(ids):
                        ui.selected = ids[i]
                elif ev.key == pygame.K_f and ui.selected:
                    # Operator input becomes a scenario event; the dashboard never touches the agent.
                    world.scenario.inject(world.t, "fail_drone", {"drone": ui.selected})
                elif ev.key == pygame.K_l:
                    ui.show_legend = not ui.show_legend
                elif ev.key == pygame.K_g:
                    ui.show_ground_truth = not ui.show_ground_truth
                elif ev.key == pygame.K_m:
                    ui.show_messages = not ui.show_messages
                elif ev.key == pygame.K_TAB:
                    ui.feed = "messages" if ui.feed == "events" else "events"
                elif ev.key == pygame.K_t:
                    ui.msg_filter += 1
                elif ev.key == pygame.K_h:
                    # Hide / show every ground-truth target (forces track loss). A scenario event, logged.
                    ui.targets_hidden = not ui.targets_hidden
                    world.scenario.inject(world.t, "hide_target" if ui.targets_hidden else "show_target")
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1 and renderer.contains(ev.pos):
                wx, wy = renderer.to_world(*ev.pos)
                pos = [round(wx, 1), round(wy, 1)]
                if pygame.key.get_mods() & pygame.KMOD_SHIFT:
                    world.scenario.inject(world.t, "external_cue", {"pos": pos})  # coordinates only, never evidence
                else:
                    # A ground-truth target: the drones still have to detect it with their sensors.
                    world.scenario.inject(world.t, "place_target", {"pos": pos})
        ui.speed = SPEEDS[speed_idx]
        if not ui.paused and not world.finished:
            sim_debt += wall_dt * ui.speed
            while sim_debt >= world.dt and not world.finished:
                world.step()
                sim_debt -= world.dt
        screen.fill((0, 0, 0))
        renderer.draw(screen, world.snapshot, ui, fonts)
        panel.draw(screen, world.snapshot, ui, fonts)
        pygame.display.flip()
    pygame.quit()
    return world


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        cfg = apply_overrides(load_config(args.config), args)
        world = World(cfg)  # geometry checks (geofence, homes, patrol routes) run here, before tick 0
    except ConfigError as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2
    for msg in cfg.info:
        print(f"info: {msg}")
    run_id = args.run_id or f"{cfg.name}_seed{cfg.sim.seed}_{time.strftime('%Y%m%d-%H%M%S')}"

    if cfg.sim.headless:
        run_headless(world)
    else:
        world = run_gui(lambda: World(cfg))

    out = write_outputs(world, Path(args.output), run_id)
    print(f"t = {world.t:.1f} s, {len(world.log.records)} events, sha256 {world.log.digest()[:16]}")
    for d in world.snapshot.drones:
        print(f"  {d.id}: {d.mode:<10} battery {d.battery_pct * 100:5.1f}%  pos ({d.x:7.1f}, {d.y:7.1f})  alt {d.alt:5.1f}")
    print(f"outputs written to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
