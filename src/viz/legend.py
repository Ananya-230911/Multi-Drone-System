"""Visual encoding shared by the map, panel and legend (PLAN.md section 11.3).

Every drone mode has its own colour AND its own short label, so modes stay
readable without relying on colour alone.
"""

from __future__ import annotations

import pygame

# mode -> (colour, label shown next to the drone)
MODE_STYLE: dict[str, tuple[tuple[int, int, int], str]] = {
    "PATROL": ((70, 160, 255), "PAT"),
    "CONFIRM_HOLD": ((255, 200, 60), "CHK"),
    "ENROUTE": ((60, 220, 120), "ENR"),
    "INSPECTING": ((0, 200, 200), "INS"),
    "REACQUIRING": ((255, 140, 0), "ACQ"),
    "RETURNING": ((190, 120, 255), "RTN"),
    "SAFETY_HOLD": ((255, 80, 80), "HLD"),
    "LANDED": ((150, 150, 150), "LND"),
    "FAILED": ((110, 110, 110), "FAIL"),
}
PHASE1_MODES = ("PATROL", "RETURNING", "FAILED")

BG = (18, 20, 26)
MAP_BG = (26, 30, 38)
PANEL_BG = (32, 36, 46)
GRID = (38, 43, 54)
TEXT = (225, 228, 235)
MUTED = (140, 146, 160)
BOUNDARY = (200, 205, 215)
HOME = (240, 240, 240)
SELECT = (255, 255, 255)
WARN = (255, 90, 90)
BAR_BG = (55, 60, 72)


def mode_colour(mode: str) -> tuple[int, int, int]:
    return MODE_STYLE.get(mode, ((255, 255, 255), "?"))[0]


def mode_label(mode: str) -> str:
    return MODE_STYLE.get(mode, ((255, 255, 255), mode))[1]


def draw_drone_icon(surf: pygame.Surface, x: float, y: float, heading: float, mode: str, size: float) -> None:
    """Triangle pointing along `heading` (screen coordinates); FAILED drones are drawn as a cross."""
    import math

    col = mode_colour(mode)
    if mode == "FAILED":
        s = size * 0.8
        pygame.draw.line(surf, col, (x - s, y - s), (x + s, y + s), 3)
        pygame.draw.line(surf, col, (x - s, y + s), (x + s, y - s), 3)
        return
    pts = []
    for ang, r in ((0.0, size * 1.3), (2.5, size), (-2.5, size)):
        a = heading + ang
        pts.append((x + r * math.cos(a), y - r * math.sin(a)))
    pygame.draw.polygon(surf, col, pts)
    pygame.draw.polygon(surf, (0, 0, 0), pts, 1)
    if mode == "RETURNING":
        pygame.draw.circle(surf, col, (int(x), int(y)), int(size * 1.8), 1)


def draw_legend(surf: pygame.Surface, font: pygame.font.Font, topleft: tuple[int, int]) -> None:
    rows: list[tuple[str, str]] = [(m, f"{mode_label(m):<4} {m}") for m in PHASE1_MODES]
    extra = ["patrol route (thin)", "active route (bright)", "trail", "home (square)", "operating boundary"]
    w, h = 250, 24 + 20 * (len(rows) + len(extra))
    x0, y0 = topleft
    box = pygame.Surface((w, h), pygame.SRCALPHA)
    box.fill((10, 12, 16, 215))
    surf.blit(box, (x0, y0))
    surf.blit(font.render("LEGEND (L)", True, TEXT), (x0 + 8, y0 + 4))
    y = y0 + 24
    for mode, text in rows:
        draw_drone_icon(surf, x0 + 18, y + 8, 0.0, mode, 6)
        surf.blit(font.render(text, True, TEXT), (x0 + 36, y))
        y += 20
    # route / trail / home / boundary samples
    pygame.draw.line(surf, (70, 90, 120), (x0 + 8, y + 8), (x0 + 28, y + 8), 1)
    surf.blit(font.render(extra[0], True, TEXT), (x0 + 36, y))
    y += 20
    pygame.draw.line(surf, (230, 230, 120), (x0 + 8, y + 8), (x0 + 28, y + 8), 2)
    surf.blit(font.render(extra[1], True, TEXT), (x0 + 36, y))
    y += 20
    for i in range(4):
        pygame.draw.circle(surf, MUTED, (x0 + 10 + i * 6, y + 8), 1)
    surf.blit(font.render(extra[2], True, TEXT), (x0 + 36, y))
    y += 20
    pygame.draw.rect(surf, HOME, (x0 + 13, y + 3, 10, 10), 1)
    surf.blit(font.render(extra[3], True, TEXT), (x0 + 36, y))
    y += 20
    pygame.draw.line(surf, BOUNDARY, (x0 + 8, y + 8), (x0 + 28, y + 8), 2)
    surf.blit(font.render(extra[4], True, TEXT), (x0 + 36, y))
