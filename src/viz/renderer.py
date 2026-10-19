"""Map layer and top bar (PLAN.md section 11.1).

Draws only from a `Snapshot`. Trails are kept here, on the visualizer side,
from successive snapshots; the agents are never touched.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import pygame

from src.viz import legend as L
from src.viz.snapshot import Snapshot

TRAIL_LEN = 300  # points kept per drone
TRAIL_EVERY = 5  # keep one trail point every N ticks
HATCH_STEP = 10  # px between zone hatch lines


@dataclass
class UiState:
    paused: bool = False
    speed: float = 1.0
    selected: str | None = None
    show_legend: bool = True
    show_ground_truth: bool = True
    targets_hidden: bool = False


class MapRenderer:
    def __init__(self, rect: pygame.Rect, map_size: tuple[float, float], top_bar: int = 32):
        self.rect = rect
        self.top_bar = top_bar
        area_w, area_h = rect.width - 20, rect.height - top_bar - 20
        self.scale = min(area_w / map_size[0], area_h / map_size[1])
        self.ox = rect.x + (rect.width - map_size[0] * self.scale) / 2
        self.oy = rect.y + top_bar + (rect.height - top_bar - map_size[1] * self.scale) / 2
        self.map_h = map_size[1]
        self.trails: dict[str, deque[tuple[float, float]]] = {}
        self._last_tick = -1

    def to_screen(self, x: float, y: float) -> tuple[float, float]:
        # World +y is north (up); screen +y is down.
        return self.ox + x * self.scale, self.oy + (self.map_h - y) * self.scale

    def to_world(self, sx: float, sy: float) -> tuple[float, float]:
        return (sx - self.ox) / self.scale, self.map_h - (sy - self.oy) / self.scale

    def contains(self, pos: tuple[int, int]) -> bool:
        return self.rect.collidepoint(pos) and pos[1] > self.rect.y + self.top_bar

    def reset(self) -> None:
        self.trails.clear()
        self._last_tick = -1

    def update_trails(self, snap: Snapshot) -> None:
        if snap.tick == self._last_tick:
            return
        if snap.tick < self._last_tick:
            self.reset()
        self._last_tick = snap.tick
        if snap.tick % TRAIL_EVERY:
            return
        for d in snap.drones:
            self.trails.setdefault(d.id, deque(maxlen=TRAIL_LEN)).append((d.x, d.y))

    def draw(self, surf: pygame.Surface, snap: Snapshot, ui: UiState, fonts: dict[str, pygame.font.Font]) -> None:
        self.update_trails(snap)
        pygame.draw.rect(surf, L.MAP_BG, self.rect)
        self._draw_grid(surf, snap)
        self._draw_fence(surf, snap, fonts)
        self._draw_footprints(surf, snap)
        for d in snap.drones:
            col = L.mode_colour(d.mode)
            dim = tuple(int(c * 0.45) for c in col)
            if len(d.patrol_route) >= 2:
                pygame.draw.lines(surf, dim, True, [self.to_screen(x, y) for x, y, _ in d.patrol_route], 1)
            hx, hy = self.to_screen(d.home[0], d.home[1])
            pygame.draw.rect(surf, L.HOME, (hx - 5, hy - 5, 10, 10), 1)
            for tx, ty in self.trails.get(d.id, ()):
                sx, sy = self.to_screen(tx, ty)
                surf.set_at((int(sx), int(sy)), dim)
            if d.route:
                pts = [self.to_screen(d.x, d.y)] + [self.to_screen(x, y) for x, y, _ in d.route]
                pygame.draw.lines(surf, L.ROUTE, False, pts, 2)
                for p in pts[1:]:
                    pygame.draw.circle(surf, L.ROUTE, (int(p[0]), int(p[1])), 3)
        if ui.show_ground_truth:
            for g in snap.ground_truth:
                if g.appeared:
                    gx, gy = self.to_screen(*g.pos)
                    L.draw_gt_marker(surf, gx, gy, g.visible)
                    txt = fonts["small"].render(g.id + ("" if g.visible else " (hidden)"), True, L.GT if g.visible else L.MUTED)
                    surf.blit(txt, (gx - txt.get_width() - 10, gy + 6))  # left of the marker; drone/target labels go right
        self._draw_targets(surf, snap, fonts)
        for d in snap.drones:
            sx, sy = self.to_screen(d.x, d.y)
            L.draw_drone_icon(surf, sx, sy, d.heading, d.mode, 8)
            if d.id == ui.selected:
                pygame.draw.circle(surf, L.SELECT, (int(sx), int(sy)), 16, 1)
            if d.mode == "SAFETY_HOLD" or d.safety_reason:
                pygame.draw.circle(surf, L.WARN, (int(sx), int(sy)), 20, 2)
                surf.blit(fonts["small"].render(d.safety_reason or "SAFETY", True, L.WARN), (sx + 12, sy + 10))
            label = f"{d.id} {L.mode_label(d.mode)} {d.alt:.0f}m"
            surf.blit(fonts["small"].render(label, True, L.TEXT), (sx + 12, sy - 18))
            if d.battery_pct < d.reserve_pct:
                surf.blit(fonts["small"].render("LOW BATT", True, L.WARN), (sx + 12, sy - 4))
        self._draw_top_bar(surf, snap, ui, fonts)
        if ui.show_legend:
            L.draw_legend(surf, fonts["small"], (self.rect.x + 10, self.rect.y + self.top_bar + 10))

    def _draw_fence(self, surf: pygame.Surface, snap: Snapshot, fonts) -> None:
        if len(snap.boundary) >= 3:
            pygame.draw.polygon(surf, L.BOUNDARY, [self.to_screen(*p) for p in snap.boundary], 2)
        if snap.fence is None:
            return
        L.dashed_polygon(surf, L.MARGIN, [self.to_screen(*p) for p in snap.fence.buffered_boundary[:-1]])
        for zid, raw, buffered in snap.fence.zones:
            pts = [self.to_screen(*p) for p in raw[:-1]]
            self._hatch(surf, pts)
            pygame.draw.polygon(surf, L.ZONE, pts, 2)
            L.dashed_polygon(surf, L.ZONE, [self.to_screen(*p) for p in buffered[:-1]])
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            txt = fonts["normal"].render(zid, True, L.ZONE)
            surf.blit(txt, (cx - txt.get_width() / 2, cy - txt.get_height() / 2))

    def _hatch(self, surf: pygame.Surface, pts: list[tuple[float, float]]) -> None:
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        box = pygame.Rect(min(xs), min(ys), max(xs) - min(xs) + 1, max(ys) - min(ys) + 1)
        layer = pygame.Surface(box.size, pygame.SRCALPHA)
        mask = pygame.Surface(box.size, pygame.SRCALPHA)
        pygame.draw.polygon(mask, (255, 255, 255, 255), [(x - box.x, y - box.y) for x, y in pts])
        for k in range(-box.height, box.width, HATCH_STEP):
            pygame.draw.line(layer, (*L.ZONE, 90), (k, box.height), (k + box.height, 0), 1)
        layer.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
        surf.blit(layer, box.topleft)

    def _draw_footprints(self, surf: pygame.Surface, snap: Snapshot) -> None:
        layer = pygame.Surface(self.rect.size, pygame.SRCALPHA)
        for d in snap.drones:
            if d.r_eff <= 0 or d.mode in ("FAILED", "LANDED") or d.docked:
                continue
            sx, sy = self.to_screen(d.x, d.y)
            r = max(int(d.r_eff * self.scale), 1)
            pygame.draw.circle(layer, (*L.mode_colour(d.mode), 45), (int(sx - self.rect.x), int(sy - self.rect.y)), r)
        surf.blit(layer, self.rect.topleft)

    def _draw_targets(self, surf: pygame.Surface, snap: Snapshot, fonts) -> None:
        """Belief targets from every agent's local view; one marker per target ID (lowest drone ID's view)."""
        seen: dict[str, tuple[str, object]] = {}
        for d in snap.drones:
            for tv in d.targets:
                if tv.id not in seen:
                    seen[tv.id] = (d.id, tv)
        for tid, (holder, tv) in sorted(seen.items()):
            if tv.state == "CANCELLED" and snap.t - tv.created_at > 60:
                continue  # old cancelled records would clutter the map
            x, y = self.to_screen(*tv.est_pos)
            if tv.state not in ("CANCELLED", "COMPLETED"):
                pygame.draw.circle(surf, L.target_colour(tv.state), (int(x), int(y)), int(8 + 14 * tv.last_confidence), 1)
            L.draw_target_marker(surf, x, y, tv.state)
            col = L.target_colour(tv.state)
            surf.blit(fonts["small"].render(f"{tid} {L.target_label(tv.state)}", True, col), (x + 10, y + 6))  # below; drone labels sit above

    def _draw_grid(self, surf: pygame.Surface, snap: Snapshot) -> None:
        step = 250.0
        w, h = snap.map_size
        x = 0.0
        while x <= w:
            pygame.draw.line(surf, L.GRID, self.to_screen(x, 0), self.to_screen(x, h))
            x += step
        y = 0.0
        while y <= h:
            pygame.draw.line(surf, L.GRID, self.to_screen(0, y), self.to_screen(w, y))
            y += step

    def _draw_top_bar(self, surf: pygame.Surface, snap: Snapshot, ui: UiState, fonts) -> None:
        bar = pygame.Rect(self.rect.x, self.rect.y, self.rect.width, self.top_bar)
        pygame.draw.rect(surf, L.BG, bar)
        status = "FINISHED" if snap.finished else ("PAUSED" if ui.paused else "RUNNING")
        text = (
            f"t = {snap.t:7.1f} / {snap.duration:.0f} s   speed {ui.speed:g}x   {status}"
            f"   scenario: {snap.scenario}   seed {snap.seed}   geofence violations: {snap.geofence_violations}"
        )
        col = L.WARN if ui.paused or snap.geofence_violations else L.TEXT
        surf.blit(fonts["normal"].render(text, True, col), (bar.x + 10, bar.y + 8))
