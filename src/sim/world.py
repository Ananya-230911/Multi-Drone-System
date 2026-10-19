"""Simulation clock and deterministic tick loop (PLAN.md section 4.2).

Tick:
  1. scenario.apply(t)
  2. (network.deliver -- Phase 3)
  3. each drone, in sorted ID order: drone.step(t, dt, sensor_view)
  4. record: drain agent events into the event log, measure true geofence
     violations (evaluation only, never fed back), build the Snapshot

The world owns ground truth and the per-drone sensor models. A drone only
ever receives the `sensor_view` closure for its own sensor, which returns
observations with the ground-truth `is_spurious` flag stripped.
"""

from __future__ import annotations

import zlib

import numpy as np

from src.config import Config, ConfigError
from src.drone.agent import Drone, SharedContext
from src.drone.sensor import Sensor
from src.logging_.event_log import EventLog
from src.safety.geofence import Geofence
from src.safety.route_validator import validate_route
from src.sim.ground_truth import GroundTruth
from src.sim.scenario import Scenario
from src.tracking.observation import AgentObservation, Observation
from src.viz.snapshot import FenceView, GroundTruthView, Snapshot

RECENT_EVENTS = 200  # events kept in the snapshot for the dashboard


def make_rng(seed: int, component: str) -> np.random.Generator:
    """One independent generator per component, derived only from `sim.seed` and the component name."""
    return np.random.default_rng(np.random.SeedSequence([seed, zlib.crc32(component.encode("utf-8"))]))


def validate_geometry(cfg: Config, fence: Geofence) -> None:
    """Section 9.1 startup checks: homes, start positions and patrol routes must be geofence-valid."""
    for d in sorted(cfg.drones, key=lambda d: d.id):
        if fence.violation(*d.home):
            raise ConfigError(f"drone {d.id}: home {list(d.home)} is outside the geofence ({fence.violation(*d.home)})")
        start = (d.pos[0], d.pos[1], d.alt)
        if fence.violation(*start):
            raise ConfigError(f"drone {d.id}: start position {list(start)} is invalid ({fence.violation(*start)})")
        loop = [start] + list(d.patrol) + [d.patrol[0]]
        check = validate_route(loop, fence)
        if not check.ok:
            seg = f"segment {check.index}: {list(loop[check.index])} -> {list(loop[min(check.index + 1, len(loop) - 1)])}"
            raise ConfigError(f"drone {d.id}: patrol route is invalid ({check.reason}, {seg})")


class World:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.dt = cfg.sim.dt
        self.tick = 0
        self.t = 0.0
        self.n_ticks = int(round(cfg.sim.duration / cfg.sim.dt))
        self.fence = Geofence.from_config(cfg)
        validate_geometry(cfg, self.fence)
        self.log = EventLog()
        self.scenario = Scenario(cfg.scenario_events)
        self.ground_truth = GroundTruth(cfg.targets)
        ids = tuple(cfg.drone_ids)
        shared = SharedContext(
            drone_ids=ids,
            fence=self.fence,
            navigation=cfg.navigation,
            confirmation=cfg.confirmation,
            association=cfg.association,
            observation=cfg.observation,
            confidence=cfg.confidence,
            track=cfg.track,
            inspection=cfg.inspection,
        )
        ordered = sorted(cfg.drones, key=lambda d: d.id)
        self.drones: dict[str, Drone] = {d.id: Drone(d, shared) for d in ordered}
        self.sensors: dict[str, Sensor] = {
            d.id: Sensor(d.id, d.params.sensor, cfg.confidence, make_rng(cfg.sim.seed, f"sensor:{d.id}")) for d in ordered
        }
        self.observations: list[Observation] = []  # every observation produced, with ground-truth flags (eval only)
        self.geofence_violations: list[tuple[float, str, str]] = []  # (t, drone, reason), measured from true state
        self.snapshot = self._build_snapshot()

    @property
    def finished(self) -> bool:
        return self.tick >= self.n_ticks

    def step(self) -> Snapshot:
        if self.finished:
            return self.snapshot
        self.tick += 1
        # Integer tick counter avoids accumulating floating-point error in the clock.
        self.t = round(self.tick * self.dt, 9)
        self.scenario.apply(self.t, self.drones, self.ground_truth, self.log)
        for drone_id in sorted(self.drones):
            self.drones[drone_id].step(self.t, self.dt, self._sensor_view(drone_id))
        self._record()
        return self.snapshot

    def run(self) -> Snapshot:
        while not self.finished:
            self.step()
        return self.snapshot

    def _sensor_view(self, drone_id: str):
        sensor = self.sensors[drone_id]

        def view(x: float, y: float, alt: float, now: float) -> list[AgentObservation]:
            obs = sensor.sample(now, self.dt, x, y, alt, self.ground_truth.visible(now))
            self.observations.extend(obs)
            return [o.to_agent() for o in obs]

        return view

    def false_confirmations(self) -> list[tuple[str, str]]:
        """(drone, target_id) of CONFIRMED records built only from spurious observations (evaluation only)."""
        spurious = {o.obs_id for o in self.observations if o.is_spurious}
        out = []
        for d in self.snapshot.drones:
            for tv in d.targets:
                if tv.confirmation == "CONFIRMED" and tv.obs_ids and all(o in spurious for o in tv.obs_ids):
                    out.append((d.id, tv.id))
        return out

    def _record(self) -> None:
        for drone_id in sorted(self.drones):
            agent = self.drones[drone_id]
            self.log.extend(agent.events)
            agent.events.clear()
        self.snapshot = self._build_snapshot()
        for d in self.snapshot.drones:
            if d.mode == "LANDED":
                continue  # an emergency landing is allowed to go below the altitude floor
            reason = self.fence.violation(d.x, d.y, d.alt)
            if reason:
                self.geofence_violations.append((self.t, d.id, reason))

    def _build_snapshot(self) -> Snapshot:
        f = self.fence
        return Snapshot(
            t=self.t,
            tick=self.tick,
            duration=self.cfg.sim.duration,
            finished=self.finished,
            scenario=self.cfg.name,
            seed=self.cfg.sim.seed,
            map_size=self.cfg.sim.map_size,
            boundary=tuple(self.cfg.geofence.boundary),
            fence=FenceView(
                buffered_boundary=tuple(f.boundary.exterior.coords),
                zones=tuple((z.id, tuple(z.raw.exterior.coords), tuple(z.buffered.exterior.coords)) for z in f.zones),
                altitude_limits=(f.min_alt, f.max_alt),
            ),
            drones=tuple(self.drones[i].export_view() for i in sorted(self.drones)),
            ground_truth=tuple(
                GroundTruthView(t.id, t.pos, t.visible(self.t), t.appear_at <= self.t) for t in self.ground_truth.targets
            ),
            recent_events=tuple(self.log.tail(RECENT_EVENTS)),
            geofence_violations=len(self.geofence_violations),
        )
