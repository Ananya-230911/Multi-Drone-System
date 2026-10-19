"""Waypoint follower (PLAN.md section 6.5).

Kinematic point mass: horizontal speed <= max_speed and vertical speed <=
climb_rate, both axes moving at the same time. Within one tick, time left over
after reaching a waypoint carries on toward the next one, so corners do not
slow the drone down.

The navigator only follows waypoints it is given. It never invents a path
toward a goal; route validation against the geofence is added in Phase 2.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.config import Point3

_EPS = 1e-9


@dataclass
class StepResult:
    x: float
    y: float
    alt: float
    horizontal: float = 0.0  # meters flown horizontally this tick
    climb: float = 0.0  # meters climbed this tick
    descent: float = 0.0  # meters descended this tick
    idle_time: float = 0.0  # seconds with no waypoint to fly to
    reached: list[int] = field(default_factory=list)  # waypoint indices reached this tick
    finished: bool = False  # True once a non-looping route is complete


class Navigator:
    def __init__(self, waypoints: list[Point3], loop: bool = False, start_index: int = 0):
        if not waypoints:
            raise ValueError("a route needs at least one waypoint")
        if not 0 <= start_index < len(waypoints):
            raise ValueError("start_index out of range")
        self.waypoints = list(waypoints)
        self.loop = loop
        self.index = start_index
        self.finished = False

    @property
    def target(self) -> Point3 | None:
        return None if self.finished else self.waypoints[self.index]

    def remaining(self) -> list[Point3]:
        """Waypoints still to be flown, in order (one lap only when looping)."""
        if self.finished:
            return []
        rest = self.waypoints[self.index :]
        if self.loop:
            rest += self.waypoints[: self.index]
        return rest

    def step(self, x: float, y: float, alt: float, dt: float, max_speed: float, climb_rate: float) -> StepResult:
        res = StepResult(x, y, alt)
        remaining = dt
        stalls = 0  # consecutive zero-length waypoints; stops a loop of identical points
        stalled = False
        while remaining > _EPS and not self.finished:
            tx, ty, tz = self.waypoints[self.index]
            dx, dy, dz = tx - res.x, ty - res.y, tz - res.alt
            d_h = math.hypot(dx, dy)
            t_need = max(d_h / max_speed, abs(dz) / climb_rate)
            if t_need <= remaining + _EPS:
                # Reach this waypoint inside the tick.
                self._accumulate(res, d_h, dz)
                res.x, res.y, res.alt = tx, ty, tz
                remaining -= t_need
                res.reached.append(self.index)
                stalls = stalls + 1 if t_need <= _EPS else 0
                self._advance()
                if stalls >= len(self.waypoints):
                    stalled = True  # every waypoint is where we already are: hover
                    break
            else:
                h = min(d_h, max_speed * remaining)
                v = min(abs(dz), climb_rate * remaining)
                if d_h > _EPS:
                    res.x += dx / d_h * h
                    res.y += dy / d_h * h
                res.alt += math.copysign(v, dz)
                self._accumulate(res, h, math.copysign(v, dz))
                remaining = 0.0
        if self.finished or stalled:
            res.idle_time = max(remaining, 0.0)
        res.finished = self.finished
        return res

    def _advance(self) -> None:
        if self.index + 1 < len(self.waypoints):
            self.index += 1
        elif self.loop:
            self.index = 0
        else:
            self.finished = True

    @staticmethod
    def _accumulate(res: StepResult, horizontal: float, dz: float) -> None:
        res.horizontal += horizontal
        if dz > 0:
            res.climb += dz
        else:
            res.descent += -dz
