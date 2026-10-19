"""True target(s) and their visibility schedule. Agents never read this.

Only the world's sensor model (drone/sensor.py, called by the world) looks at
ground truth. Ground-truth IDs (GT-1, ...) never reach an agent.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from src.config import Point2, TargetConfig


@dataclass
class GroundTruthTarget:
    id: str
    pos: Point2
    alt: float = 0.0
    appear_at: float = 0.0
    hidden_intervals: list[Point2] = field(default_factory=list)  # [start, end) in sim seconds
    manually_hidden: bool = False  # toggled by the operator (H key) or hide/show scenario events

    def visible(self, now: float) -> bool:
        if now < self.appear_at or self.manually_hidden:
            return False
        return not any(a <= now < b for a, b in self.hidden_intervals)


class GroundTruth:
    def __init__(self, targets: list[TargetConfig]):
        self.targets: list[GroundTruthTarget] = [
            GroundTruthTarget(t.id, t.pos, t.alt, t.appear_at, list(t.hidden_intervals)) for t in targets
        ]

    def visible(self, now: float) -> list[GroundTruthTarget]:
        return [t for t in self.targets if t.visible(now)]

    def place(self, pos: Point2, now: float, alt: float = 0.0) -> str:
        n = 1
        ids = {t.id for t in self.targets}
        while f"GT-{n}" in ids:
            n += 1
        tid = f"GT-{n}"
        self.targets.append(GroundTruthTarget(tid, (float(pos[0]), float(pos[1])), alt, appear_at=now))
        return tid

    def set_hidden(self, hidden: bool, target_id: str | None = None) -> list[str]:
        changed = []
        for t in self.targets:
            if target_id is None or t.id == target_id:
                t.manually_hidden = hidden
                changed.append(t.id)
        return changed
