"""Side panel: drone table, recent events, controls (PLAN.md section 11.1).

Phase 1 shows the DRONES section and an event feed. Target, bid and message
sections are added by later phases.
"""

from __future__ import annotations

import pygame

from src.viz import legend as L
from src.viz.renderer import UiState
from src.viz.snapshot import Snapshot

FEED_SKIP = {"WAYPOINT_REACHED"}  # too frequent to be useful in the feed
CONTROLS = [
    "SPACE pause/resume    <- / -> speed",
    "R reset    1-4 select drone    F fail selected",
    "L legend",
]


class Panel:
    def __init__(self, rect: pygame.Rect):
        self.rect = rect

    def draw(self, surf: pygame.Surface, snap: Snapshot, ui: UiState, fonts: dict[str, pygame.font.Font]) -> None:
        pygame.draw.rect(surf, L.PANEL_BG, self.rect)
        x0, y = self.rect.x + 14, self.rect.y + 12
        surf.blit(fonts["title"].render("DRONES", True, L.TEXT), (x0, y))
        y += 30
        header = f"{'ID':<4}{'MODE':<11}{'BATT':>6}{'ALT':>7}{'SPD':>7}"
        surf.blit(fonts["mono"].render(header, True, L.MUTED), (x0, y))
        y += 20
        for d in snap.drones:
            row_rect = pygame.Rect(self.rect.x + 6, y - 2, self.rect.width - 12, 40)
            if d.id == ui.selected:
                pygame.draw.rect(surf, (52, 58, 72), row_rect)
                pygame.draw.rect(surf, L.SELECT, row_rect, 1)
            col = L.mode_colour(d.mode)
            mode_txt = d.mode + (" (home)" if d.docked else "")
            line = f"{d.id:<4}{mode_txt:<11}{d.battery_pct * 100:5.1f}%{d.alt:6.0f}m{d.speed:5.1f}/s"
            surf.blit(fonts["mono"].render(line, True, col), (x0, y))
            self._battery_bar(surf, x0, y + 20, self.rect.width - 40, d.battery_pct, d.reserve_pct)
            y += 44
        y += 8
        surf.blit(fonts["title"].render("EVENTS", True, L.TEXT), (x0, y))
        y += 28
        feed = [e for e in snap.recent_events if e["type"] not in FEED_SKIP][-16:]
        for e in reversed(feed):
            surf.blit(fonts["small"].render(_describe(e), True, _event_colour(e)), (x0, y))
            y += 17
        y = self.rect.bottom - 20 * len(CONTROLS) - 10
        for line in CONTROLS:
            surf.blit(fonts["small"].render(line, True, L.MUTED), (x0, y))
            y += 20

    @staticmethod
    def _battery_bar(surf, x: int, y: int, w: int, pct: float, reserve: float) -> None:
        pygame.draw.rect(surf, L.BAR_BG, (x, y, w, 8))
        col = L.WARN if pct < reserve else (90, 200, 120)
        pygame.draw.rect(surf, col, (x, y, int(w * max(0.0, min(pct, 1.0))), 8))
        rx = x + int(w * reserve)
        pygame.draw.line(surf, L.TEXT, (rx, y - 2), (rx, y + 10), 1)  # reserve marker


def _describe(e: dict) -> str:
    d = e.get("data", {})
    t = f"{e['t']:7.1f}"
    if e["type"] == "DRONE_MODE":
        return f"{t} {e['actor']} {d.get('from')} -> {d.get('to')} ({d.get('reason')})"
    if e["type"] == "SCENARIO_EVENT":
        who = f" {d['drone']}" if "drone" in d else ""
        ok = "" if d.get("applied") else " [ignored]"
        return f"{t} scenario {d.get('type')}{who}{ok}"
    if e["type"] == "SAFETY_RETURN":
        return f"{t} {e['actor']} SAFETY_RETURN {d.get('reason')} (E_home {d.get('e_home_wh', 0):.1f} Wh)"
    if e["type"] == "HOME_REACHED":
        nxt = "resume patrol" if d.get("resume_patrol") else "parked"
        return f"{t} {e['actor']} home, batt {d.get('battery_pct', 0) * 100:.1f}% -> {nxt}"
    return f"{t} {e['actor']} {e['type']}"


def _event_colour(e: dict) -> tuple[int, int, int]:
    if e["type"].startswith("SAFETY") or e["type"] == "DRONE_FAILED":
        return L.WARN
    if e["type"] == "DRONE_MODE":
        return L.mode_colour(e["data"].get("to", ""))
    return L.TEXT
