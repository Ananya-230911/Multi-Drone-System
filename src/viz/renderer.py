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


@dataclass
class UiState:
    paused: bool = False
    speed: float = 1.0
    selected: str | None = None
    show_legend: bool = True


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
        if len(snap.boundary) >= 3:
            pygame.draw.polygon(surf, L.BOUNDARY, [self.to_screen(*p) for p in snap.boundary], 2)
        for d in snap.drones:
            col = L.mode_colour(d.mode)
            dim = tuple(int(c * 0.45) for c in col)
            if len(d.patrol_route) >= 2:
                pts = [self.to_screen(x, y) for x, y, _ in d.patrol_route]
                pygame.draw.lines(surf, dim, True, pts, 1)
            hx, hy = self.to_screen(d.home[0], d.home[1])
            pygame.draw.rect(surf, L.HOME, (hx - 5, hy - 5, 10, 10), 1)
            for tx, ty in self.trails.get(d.id, ()):
                sx, sy = self.to_screen(tx, ty)
                surf.set_at((int(sx), int(sy)), dim)
            if d.route and d.mode != "PATROL":
                pts = [self.to_screen(d.x, d.y)] + [self.to_screen(x, y) for x, y, _ in d.route]
                pygame.draw.lines(surf, (230, 230, 120), False, pts, 2)
        for d in snap.drones:
            sx, sy = self.to_screen(d.x, d.y)
            L.draw_drone_icon(surf, sx, sy, d.heading, d.mode, 8)
            if d.id == ui.selected:
                pygame.draw.circle(surf, L.SELECT, (int(sx), int(sy)), 16, 1)
            label = f"{d.id} {L.mode_label(d.mode)} {d.alt:.0f}m"
            surf.blit(fonts["small"].render(label, True, L.TEXT), (sx + 12, sy - 18))
            if d.battery_pct < d.reserve_pct:
                surf.blit(fonts["small"].render("LOW BATT", True, L.WARN), (sx + 12, sy - 4))
        self._draw_top_bar(surf, snap, ui, fonts)
        if ui.show_legend:
            L.draw_legend(surf, fonts["small"], (self.rect.x + 10, self.rect.y + self.top_bar + 10))

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
            f"t = {snap.t:7.1f} / {snap.duration:.0f} s    speed {ui.speed:g}x    {status}"
            f"    scenario: {snap.scenario}    seed {snap.seed}"
        )
        col = L.WARN if ui.paused else L.TEXT
        surf.blit(fonts["normal"].render(text, True, col), (bar.x + 10, bar.y + 8))
