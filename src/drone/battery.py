"""Energy model (PLAN.md sections 6.3 and 6.4).

Pure functions plus a small `Battery` holder. Energy estimates are always
computed over a list of route waypoints, never a straight line to the goal.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from src.config import DroneParams, Point3


@dataclass(frozen=True)
class EnergyModel:
    k_h: float  # Wh per meter of level flight
    k_c: float  # Wh per meter climbed
    k_d: float  # Wh per meter descended
    hover_power: float  # Wh per second while holding position in the air

    @classmethod
    def from_params(cls, p: DroneParams) -> "EnergyModel":
        return cls(k_h=p.k_h, k_c=p.k_c, k_d=p.k_d, hover_power=p.hover_power)

    def move_energy(self, horizontal: float, climb: float, descent: float) -> float:
        """Energy for a displacement. `climb` and `descent` are non-negative meters."""
        if horizontal < 0 or climb < 0 or descent < 0:
            raise ValueError("distances must be non-negative")
        return self.k_h * horizontal + self.k_c * climb + self.k_d * descent

    def hover_energy(self, seconds: float) -> float:
        if seconds < 0:
            raise ValueError("seconds must be non-negative")
        return self.hover_power * seconds


@dataclass(frozen=True)
class RouteCost:
    horizontal: float
    climb: float
    descent: float
    energy_wh: float


def route_cost(start: Point3, waypoints: Iterable[Point3], model: EnergyModel) -> RouteCost:
    """Length, climb, descent and energy of flying `start -> waypoints[0] -> ...`."""
    horizontal = climb = descent = 0.0
    px, py, pz = start
    for x, y, z in waypoints:
        horizontal += math.hypot(x - px, y - py)
        dz = z - pz
        if dz > 0:
            climb += dz
        else:
            descent += -dz
        px, py, pz = x, y, z
    return RouteCost(horizontal, climb, descent, model.move_energy(horizontal, climb, descent))


class Battery:
    def __init__(self, capacity_wh: float, charge_wh: float, reserve_pct: float, critical_pct: float):
        if capacity_wh <= 0:
            raise ValueError("capacity must be positive")
        self.capacity_wh = capacity_wh
        self.charge_wh = min(max(charge_wh, 0.0), capacity_wh)
        self.reserve_wh = reserve_pct * capacity_wh
        self.critical_wh = critical_pct * capacity_wh

    @classmethod
    def from_params(cls, p: DroneParams, battery_pct: float) -> "Battery":
        return cls(p.capacity_wh, battery_pct * p.capacity_wh, p.reserve_pct, p.critical_pct)

    @property
    def pct(self) -> float:
        return self.charge_wh / self.capacity_wh

    @property
    def empty(self) -> bool:
        return self.charge_wh <= 0.0

    def drain(self, wh: float) -> float:
        """Remove `wh` (clamped at zero). Returns the energy actually drawn."""
        if wh < 0:
            raise ValueError("cannot drain a negative amount")
        drawn = min(wh, self.charge_wh)
        self.charge_wh -= drawn
        return drawn

    def set_pct(self, pct: float) -> None:
        self.charge_wh = min(max(pct, 0.0), 1.0) * self.capacity_wh

    def reserve_violated(self, energy_needed_wh: float) -> bool:
        """Battery reserve protection (6.4): True if `battery - E_needed < reserve`."""
        return self.charge_wh - energy_needed_wh < self.reserve_wh
