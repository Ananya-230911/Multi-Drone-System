"""Read-only snapshot of the world, built after every tick (PLAN.md section 4.1 rule 5).

The visualizer, event log consumers and evaluator see the simulation only
through this object. Everything in it is an immutable copy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from src.config import Point2
from src.drone.agent import DroneView


@dataclass(frozen=True)
class Snapshot:
    t: float
    tick: int
    duration: float
    finished: bool
    scenario: str
    seed: int
    map_size: Point2
    boundary: tuple[Point2, ...]
    drones: tuple[DroneView, ...]
    recent_events: tuple[dict[str, Any], ...]

    def drone(self, drone_id: str) -> DroneView | None:
        return next((d for d in self.drones if d.id == drone_id), None)
