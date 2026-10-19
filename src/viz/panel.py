"""Side panel: drone table, target records, recent events, controls (PLAN.md section 11.1).

Bid, transition-history and message sections are added by later phases.
"""

from __future__ import annotations

import pygame

from src.viz import legend as L
from src.viz.renderer import UiState
from src.viz.snapshot import Snapshot

FEED_SKIP = {"WAYPOINT_REACHED", "OBSERVATION", "TRACK_FRESHNESS", "ROUTE_PLANNED"}  # too frequent for the feed
MAX_TARGET_ROWS = 7
CONTROLS = [
    "SPACE pause   <- / -> speed   R reset   1-4 select   F fail",
    "click: place ground-truth target   Shift+click: cue (Phase 3)",
    "H hide/show targets   G ground truth   L legend",
]


class Panel:
    def __init__(self, rect: pygame.Rect):
        self.rect = rect

    def draw(self, surf: pygame.Surface, snap: Snapshot, ui: UiState, fonts: dict[str, pygame.font.Font]) -> None:
        pygame.draw.rect(surf, L.PANEL_BG, self.rect)
        x0, y = self.rect.x + 14, self.rect.y + 10
        surf.blit(fonts["title"].render("DRONES", True, L.TEXT), (x0, y))
        y += 26
        header = f"{'ID':<4}{'MODE':<13}{'BATT':>6}{'ALT':>6}{'SPD':>8}"
        surf.blit(fonts["mono"].render(header, True, L.MUTED), (x0, y))
        y += 18
        for d in snap.drones:
            row_rect = pygame.Rect(self.rect.x + 6, y - 2, self.rect.width - 12, 36)
            if d.id == ui.selected:
                pygame.draw.rect(surf, (52, 58, 72), row_rect)
                pygame.draw.rect(surf, L.SELECT, row_rect, 1)
            col = L.mode_colour(d.mode)
            mode_txt = "PARKED HOME" if d.docked else d.mode
            line = f"{d.id:<4}{mode_txt:<13}{d.battery_pct * 100:5.1f}%{d.alt:5.0f}m{d.speed:5.1f}m/s"
            surf.blit(fonts["mono"].render(line, True, col), (x0, y))
            self._battery_bar(surf, x0, y + 19, self.rect.width - 40, d.battery_pct, d.reserve_pct)
            y += 38
        y += 4
        y = self._draw_targets(surf, snap, fonts, x0, y)
        y += 6
        surf.blit(fonts["title"].render("EVENTS", True, L.TEXT), (x0, y))
        y += 24
        bottom = self.rect.bottom - 20 * len(CONTROLS) - 14
        feed = [e for e in snap.recent_events if e["type"] not in FEED_SKIP]
        for e in reversed(feed):
            if y > bottom - 16:
                break
            surf.blit(fonts["small"].render(_describe(e), True, _event_colour(e)), (x0, y))
            y += 16
        y = bottom + 6
        for line in CONTROLS:
            surf.blit(fonts["small"].render(line, True, L.MUTED), (x0, y))
            y += 20

    def _draw_targets(self, surf, snap: Snapshot, fonts, x0: int, y: int) -> int:
        surf.blit(fonts["title"].render("TARGETS (each drone's own view)", True, L.TEXT), (x0, y))
        y += 24
        rows = []
        for d in snap.drones:
            for tv in d.targets:
                rows.append((tv.state == "CANCELLED", -tv.created_at, d.id, tv))
        if not rows:
            surf.blit(fonts["small"].render("no target records yet", True, L.MUTED), (x0, y))
            return y + 18
        rows.sort(key=lambda r: (r[0], r[1]))
        for _, _, holder, tv in rows[:MAX_TARGET_ROWS]:
            age = snap.t - tv.created_at
            line = (f"{holder}: {tv.id:<10} {tv.state:<9} conf {tv.last_confidence:4.2f} {tv.category:<6}"
                    f" {tv.freshness:<5} obs {tv.n_obs:<3} {age:4.0f}s")  # fmt: skip
            surf.blit(fonts["mono_small"].render(line, True, L.target_colour(tv.state)), (x0, y))
            y += 17
        if len(rows) > MAX_TARGET_ROWS:
            surf.blit(fonts["small"].render(f"... {len(rows) - MAX_TARGET_ROWS} more", True, L.MUTED), (x0, y))
            y += 17
        return y

    @staticmethod
    def _battery_bar(surf, x: int, y: int, w: int, pct: float, reserve: float) -> None:
        pygame.draw.rect(surf, L.BAR_BG, (x, y, w, 7))
        col = L.WARN if pct < reserve else (90, 200, 120)
        pygame.draw.rect(surf, col, (x, y, int(w * max(0.0, min(pct, 1.0))), 7))
        rx = x + int(w * reserve)
        pygame.draw.line(surf, L.TEXT, (rx, y - 2), (rx, y + 9), 1)  # reserve marker


def _describe(e: dict) -> str:
    d = e.get("data", {})
    t = f"{e['t']:7.1f}"
    tid = e.get("target_id", "")
    if e["type"] == "DRONE_MODE":
        return f"{t} {e['actor']} {d.get('from')} -> {d.get('to')} ({d.get('reason')})"
    if e["type"] == "SCENARIO_EVENT":
        who = f" {d['drone']}" if "drone" in d else (f" {d['target']}" if "target" in d else "")
        ok = "" if d.get("applied") else " [ignored]"
        return f"{t} scenario {d.get('type')}{who}{ok}"
    if e["type"] == "SAFETY_RETURN":
        e_home = d.get("e_home_wh")
        return f"{t} {e['actor']} SAFETY_RETURN {d.get('reason')} (E_home {e_home:.1f} Wh)" if e_home is not None else f"{t} {e['actor']} SAFETY_RETURN"
    if e["type"] in ("SAFETY_HOLD", "SAFETY_REPLAN", "SAFETY_LAND"):
        return f"{t} {e['actor']} {e['type']} {d.get('reason', d.get('result', ''))}"
    if e["type"] == "HOME_REACHED":
        nxt = "resume patrol" if d.get("resume_patrol") else "parked"
        return f"{t} {e['actor']} home, batt {d.get('battery_pct', 0) * 100:.1f}% -> {nxt}"
    if e["type"] == "TARGET_TRANSITION":
        return f"{t} {e['actor']} {tid} {d.get('from')} -> {d.get('to')} (row {d.get('row')}, {d.get('reason')})"
    if e["type"] == "CONFIRMATION_RESULT":
        return f"{t} {e['actor']} {tid} {d.get('result')} ({d.get('observations_in_window')} obs, mean {d.get('mean_confidence', 0):.2f})"
    if e["type"] == "TARGET_MERGED":
        return f"{t} {e['actor']} merged {d.get('alias')} into {d.get('survivor')}"
    if e["type"] == "ROUTE_REJECTED":
        return f"{t} {e['actor']} ROUTE_REJECTED {d.get('kind')} {d.get('reason')}"
    if e["type"] == "OBSERVATION_REJECTED":
        return f"{t} {e['actor']} observation rejected: {d.get('reason')}"
    return f"{t} {e['actor']} {e['type']} {tid}"


def _event_colour(e: dict) -> tuple[int, int, int]:
    if e["type"].startswith("SAFETY") or e["type"] in ("DRONE_FAILED", "ROUTE_REJECTED"):
        return L.WARN
    if e["type"] == "DRONE_MODE":
        return L.mode_colour(e["data"].get("to", ""))
    if e["type"] == "TARGET_TRANSITION":
        return L.target_colour(e["data"].get("to", ""))
    return L.TEXT
