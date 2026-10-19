"""Visual encoding shared by the map, panel and legend (PLAN.md section 11.3).

Every drone mode and every target state has its own colour AND its own shape
or short label, so states stay readable without relying on colour alone.
"""

from __future__ import annotations

import math

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
PHASE_MODES = ("PATROL", "CONFIRM_HOLD", "RETURNING", "SAFETY_HOLD", "LANDED", "FAILED")

# target state -> (colour, marker, label)
TARGET_STYLE: dict[str, tuple[tuple[int, int, int], str, str]] = {
    "DETECTED": ((255, 215, 0), "diamond_open", "DET"),
    "CONFIRMED": ((255, 140, 40), "diamond", "CNF"),
    "ALLOCATED": ((60, 220, 120), "square_open", "ALC"),
    "ENROUTE": ((60, 220, 120), "square", "ENR"),
    "TRACKING": ((0, 220, 220), "circle", "TRK"),
    "TRACKING_LOST": ((255, 90, 90), "circle_open", "LOST"),
    "REACQUIRING": ((255, 140, 0), "circle_open", "ACQ"),
    "LOGGED": ((120, 255, 120), "star", "LOG"),
    "COMPLETED": ((120, 255, 120), "star", "DONE"),
    "CANCELLED": ((120, 120, 120), "cross", "CXL"),
}
PHASE_TARGET_STATES = ("DETECTED", "CONFIRMED", "ALLOCATED", "CANCELLED")

# message type -> colour of the flash line / message log entry
MSG_COLOURS: dict[str, tuple[int, int, int]] = {
    "HEARTBEAT": (90, 100, 120),
    "TARGET_DETECTED": (255, 215, 0),
    "OBSERVATION": (200, 180, 90),
    "TARGET_STATE": (255, 140, 40),
    "MERGE": (230, 130, 230),
    "BID": (90, 200, 255),
    "CLAIM": (120, 255, 120),
    "ACK": (120, 200, 120),
    "RELEASE": (255, 90, 90),
    "TARGET_LOGGED": (120, 255, 120),
}

BG = (18, 20, 26)
MAP_BG = (26, 30, 38)
PANEL_BG = (32, 36, 46)
GRID = (38, 43, 54)
TEXT = (225, 228, 235)
MUTED = (140, 146, 160)
BOUNDARY = (200, 205, 215)
MARGIN = (120, 126, 140)
ZONE = (200, 70, 70)
HOME = (240, 240, 240)
SELECT = (255, 255, 255)
WARN = (255, 90, 90)
GT = (150, 255, 150)
BAR_BG = (55, 60, 72)
ROUTE = (230, 230, 120)


def mode_colour(mode: str) -> tuple[int, int, int]:
    return MODE_STYLE.get(mode, ((255, 255, 255), "?"))[0]


def mode_label(mode: str) -> str:
    return MODE_STYLE.get(mode, ((255, 255, 255), mode))[1]


def target_colour(state: str) -> tuple[int, int, int]:
    return TARGET_STYLE.get(state, ((255, 255, 255), "circle", "?"))[0]


def target_label(state: str) -> str:
    return TARGET_STYLE.get(state, ((255, 255, 255), "circle", state))[2]


def dashed_line(surf, col, a, b, width: int = 1, dash: float = 6.0) -> None:
    (x1, y1), (x2, y2) = a, b
    length = math.hypot(x2 - x1, y2 - y1)
    if length < 1:
        return
    n = int(length // dash)
    for i in range(0, n, 2):
        s, e = i / n, min((i + 1) / n, 1.0)
        pygame.draw.line(surf, col, (x1 + (x2 - x1) * s, y1 + (y2 - y1) * s), (x1 + (x2 - x1) * e, y1 + (y2 - y1) * e), width)


def dashed_polygon(surf, col, pts, width: int = 1) -> None:
    for a, b in zip(pts, pts[1:] + pts[:1]):
        dashed_line(surf, col, a, b, width)


def draw_drone_icon(surf: pygame.Surface, x: float, y: float, heading: float, mode: str, size: float) -> None:
    """Triangle pointing along `heading`; FAILED is a cross, LANDED a filled square."""
    col = mode_colour(mode)
    if mode == "FAILED":
        s = size * 0.8
        pygame.draw.line(surf, col, (x - s, y - s), (x + s, y + s), 3)
        pygame.draw.line(surf, col, (x - s, y + s), (x + s, y - s), 3)
        return
    if mode == "LANDED":
        pygame.draw.rect(surf, col, (x - size * 0.7, y - size * 0.7, size * 1.4, size * 1.4))
        return
    pts = []
    for ang, r in ((0.0, size * 1.3), (2.5, size), (-2.5, size)):
        a = heading + ang
        pts.append((x + r * math.cos(a), y - r * math.sin(a)))
    pygame.draw.polygon(surf, col, pts)
    pygame.draw.polygon(surf, (0, 0, 0), pts, 1)
    if mode == "RETURNING":
        pygame.draw.circle(surf, col, (int(x), int(y)), int(size * 1.8), 1)
    elif mode == "CONFIRM_HOLD":
        dashed_polygon(surf, col, [(x - size * 1.8, y - size * 1.8), (x + size * 1.8, y - size * 1.8),
                                   (x + size * 1.8, y + size * 1.8), (x - size * 1.8, y + size * 1.8)])  # fmt: skip


def draw_target_marker(surf: pygame.Surface, x: float, y: float, state: str, size: float = 7) -> None:
    col, marker, _ = TARGET_STYLE.get(state, ((255, 255, 255), "circle", "?"))
    diamond = [(x, y - size), (x + size, y), (x, y + size), (x - size, y)]
    square = pygame.Rect(x - size * 0.8, y - size * 0.8, size * 1.6, size * 1.6)
    if marker == "diamond_open":
        pygame.draw.polygon(surf, col, diamond, 2)
    elif marker == "diamond":
        pygame.draw.polygon(surf, col, diamond)
    elif marker == "square_open":
        pygame.draw.rect(surf, col, square, 2)
    elif marker == "square":
        pygame.draw.rect(surf, col, square)
    elif marker == "circle":
        pygame.draw.circle(surf, col, (int(x), int(y)), int(size * 0.8))
    elif marker == "circle_open":
        pygame.draw.circle(surf, col, (int(x), int(y)), int(size * 0.8), 2)
    elif marker == "star":
        pts = [(x + (size if i % 2 == 0 else size * 0.45) * math.cos(math.pi / 2 + i * math.pi / 5),
                y - (size if i % 2 == 0 else size * 0.45) * math.sin(math.pi / 2 + i * math.pi / 5)) for i in range(10)]  # fmt: skip
        pygame.draw.polygon(surf, col, pts)
    else:  # cross
        s = size * 0.6
        pygame.draw.line(surf, col, (x - s, y - s), (x + s, y + s), 2)
        pygame.draw.line(surf, col, (x - s, y + s), (x + s, y - s), 2)


def draw_gt_marker(surf: pygame.Surface, x: float, y: float, visible: bool) -> None:
    col = GT if visible else MUTED
    pygame.draw.line(surf, col, (x - 9, y), (x + 9, y), 1)
    pygame.draw.line(surf, col, (x, y - 9), (x, y + 9), 1)


def draw_legend(surf: pygame.Surface, font: pygame.font.Font, topleft: tuple[int, int]) -> None:
    rows_mode = list(PHASE_MODES)
    rows_target = list(PHASE_TARGET_STATES)
    extras = ["sensor footprint r_eff", "restricted zone (+margin dashed)", "boundary (+margin dashed)",
              "ground truth (G)", "planned route", "home", "safety warning", "comm range (selected)",
              "message flash (M): BID / CLAIM / other"]  # fmt: skip
    w = 270
    h = 26 + 18 * (len(rows_mode) + len(rows_target) + len(extras)) + 12
    x0, y0 = topleft
    box = pygame.Surface((w, h), pygame.SRCALPHA)
    box.fill((10, 12, 16, 220))
    surf.blit(box, (x0, y0))
    surf.blit(font.render("LEGEND (L)", True, TEXT), (x0 + 8, y0 + 4))
    y = y0 + 24
    for m in rows_mode:
        draw_drone_icon(surf, x0 + 18, y + 8, 0.0, m, 5)
        surf.blit(font.render(f"{mode_label(m):<4} drone {m}", True, TEXT), (x0 + 40, y))
        y += 18
    y += 6
    for st in rows_target:
        draw_target_marker(surf, x0 + 18, y + 8, st, 6)
        surf.blit(font.render(f"{target_label(st):<4} target {st}", True, TEXT), (x0 + 40, y))
        y += 18
    y += 6
    sample = pygame.Surface((22, 14), pygame.SRCALPHA)
    pygame.draw.circle(sample, (70, 160, 255, 60), (11, 7), 7)
    surf.blit(sample, (x0 + 7, y + 1))
    surf.blit(font.render(extras[0], True, TEXT), (x0 + 40, y))
    y += 18
    pygame.draw.rect(surf, ZONE, (x0 + 10, y + 3, 16, 10), 1)
    surf.blit(font.render(extras[1], True, TEXT), (x0 + 40, y))
    y += 18
    pygame.draw.line(surf, BOUNDARY, (x0 + 8, y + 8), (x0 + 28, y + 8), 2)
    surf.blit(font.render(extras[2], True, TEXT), (x0 + 40, y))
    y += 18
    draw_gt_marker(surf, x0 + 18, y + 8, True)
    surf.blit(font.render(extras[3], True, TEXT), (x0 + 40, y))
    y += 18
    pygame.draw.line(surf, ROUTE, (x0 + 8, y + 8), (x0 + 28, y + 8), 2)
    surf.blit(font.render(extras[4], True, TEXT), (x0 + 40, y))
    y += 18
    pygame.draw.rect(surf, HOME, (x0 + 13, y + 3, 10, 10), 1)
    surf.blit(font.render(extras[5], True, TEXT), (x0 + 40, y))
    y += 18
    pygame.draw.circle(surf, WARN, (x0 + 18, y + 8), 7, 2)
    surf.blit(font.render(extras[6], True, TEXT), (x0 + 40, y))
    y += 18
    dashed_line(surf, MUTED, (x0 + 6, y + 8), (x0 + 30, y + 8), 1, 4)
    surf.blit(font.render(extras[7], True, TEXT), (x0 + 40, y))
    y += 18
    for i, m in enumerate(("BID", "CLAIM", "HEARTBEAT")):
        pygame.draw.line(surf, MSG_COLOURS[m], (x0 + 6 + i * 9, y + 13), (x0 + 12 + i * 9, y + 3), 2)
    surf.blit(font.render(extras[8], True, TEXT), (x0 + 40, y))
