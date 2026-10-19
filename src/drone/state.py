"""Private state of one drone agent (PLAN.md section 6.1).

Only the owning `Drone` writes this. Everything outside the agent sees copies
through the read-only `Snapshot`.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from src.config import DroneParams, Point3
from src.drone.fsm import DroneMode


@dataclass
class DroneState:
    id: str
    x: float
    y: float
    alt: float
    home: Point3
    patrol_route: list[Point3]
    params: DroneParams
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    heading: float = 0.0  # radians, 0 = +x, counter-clockwise
    mode: DroneMode = DroneMode.PATROL
    mode_since: float = 0.0
    current_task: str | None = None
    task_epoch: int = 0
    patrol_index: int = 0  # index of the patrol waypoint currently being flown to
    peer_table: dict[str, dict] = field(default_factory=dict)  # filled from heartbeats (Phase 3)

    @property
    def pos(self) -> tuple[float, float]:
        return (self.x, self.y)

    @property
    def pos3(self) -> Point3:
        return (self.x, self.y, self.alt)
