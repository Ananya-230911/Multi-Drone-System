"""Route validation (PLAN.md section 9.2).

A route is a list of 3D waypoints (x, y, alt). It is valid only if every
segment stays strictly inside the buffered boundary, does not touch or cross
any buffered restricted zone, and has both ends inside the altitude limits.
Altitude is linear between waypoints, so valid ends keep the whole segment
within limits. The only exception is a purely vertical segment above a home
position (takeoff / landing), which may go below `min_alt`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.config import Point3
from src.safety.geofence import Geofence

Waypoint = Point3
_EPS = 1e-6


@dataclass(frozen=True)
class ValidationResult:
    ok: bool
    index: int | None = None  # first failing segment (or point) index
    reason: str | None = None

    @staticmethod
    def passed() -> "ValidationResult":
        return ValidationResult(True)


def validate_point(p: Waypoint, fence: Geofence) -> str | None:
    return fence.violation(p[0], p[1], p[2])


def validate_segment(a: Waypoint, b: Waypoint, fence: Geofence) -> ValidationResult:
    reason = fence.horizontal_segment_violation((a[0], a[1]), (b[0], b[1]))
    if reason:
        return ValidationResult(False, 0, reason)
    vertical = math.hypot(b[0] - a[0], b[1] - a[1]) <= _EPS
    for x, y, z in (a, b):
        if z > fence.max_alt:
            return ValidationResult(False, 0, "ALT_ABOVE_MAX")
        if z < fence.min_alt and not (vertical and fence.at_home(x, y)):
            return ValidationResult(False, 0, "ALT_BELOW_MIN")
    return ValidationResult.passed()


def validate_route(route: list[Waypoint], fence: Geofence) -> ValidationResult:
    if not route:
        return ValidationResult(False, 0, "GOAL_INVALID")
    if validate_point(route[0], fence):
        return ValidationResult(False, 0, "START_INVALID")
    if validate_point(route[-1], fence):
        return ValidationResult(False, len(route) - 1, "GOAL_INVALID")
    for i in range(len(route) - 1):
        r = validate_segment(route[i], route[i + 1], fence)
        if not r.ok:
            return ValidationResult(False, i, r.reason)
    return ValidationResult.passed()


def route_length(route: list[Waypoint]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(route, route[1:]))
