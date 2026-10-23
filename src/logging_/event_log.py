"""Append-only event log written as JSON Lines (PLAN.md section 13.1).

Each record: {t, type, actor, target_id?, epoch?, data}. Serialisation is
canonical (sorted keys, floats rounded) so that two runs with the same seed
produce byte-identical files, which the determinism test relies on.
"""

from __future__ import annotations

import hashlib
import json
from collections import deque
from pathlib import Path
from typing import Any, Iterable

FLOAT_DIGITS = 6

# Section 13.1 event types, plus WAYPOINT_REACHED and HOME_REACHED (Phase 1) and
# TARGET_NEUTRALIZED (Phase 4, world event when mission.neutralize_on_complete is on).
EVENT_TYPES = frozenset(
    {
        "SCENARIO_EVENT", "MSG_SENT", "MSG_DELIVERED", "MSG_DROPPED", "OBSERVATION",
        "OBSERVATION_REJECTED", "TARGET_CREATED", "TARGET_MERGED", "TARGET_TRANSITION",
        "ILLEGAL_TRANSITION", "CONFIRMATION_RESULT", "BID_COMPUTED", "ALLOCATION_COMMITTED",
        "ALLOCATION_FAILED", "CLAIM_CONFLICT", "DUPLICATE_OWNER_SEEN", "REALLOCATION", "ROUTE_PLANNED", "ROUTE_REJECTED",
        "SAFETY_HOLD", "SAFETY_REPLAN", "SAFETY_RETURN", "SAFETY_LAND", "SEPARATION_YIELD",
        "TRACK_FRESHNESS", "TRACK_LOST", "REACQUIRE_START", "REACQUIRE_SUCCESS", "REACQUIRE_TIMEOUT",
        "INSPECTION_PROGRESS", "TARGET_LOGGED", "TASK_COMPLETED", "TASK_CANCELLED", "DRONE_MODE",
        "DRONE_FAILED", "WAYPOINT_REACHED", "HOME_REACHED", "TARGET_NEUTRALIZED",
    }
)  # fmt: skip


def _clean(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, FLOAT_DIGITS) + 0.0  # + 0.0 turns -0.0 into 0.0
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    return value


# Very frequent event types left out of `important` (the dashboard feed). They are still in the log.
NOISY = frozenset({"MSG_SENT", "MSG_DELIVERED", "MSG_DROPPED", "OBSERVATION", "WAYPOINT_REACHED",
                   "TRACK_FRESHNESS", "ROUTE_PLANNED", "INSPECTION_PROGRESS"})  # fmt: skip
IMPORTANT_KEPT = 300


class EventLog:
    def __init__(self) -> None:
        self.records: list[dict[str, Any]] = []
        self._lines: list[str] = []
        self.important: deque[dict[str, Any]] = deque(maxlen=IMPORTANT_KEPT)

    def append(
        self,
        t: float,
        etype: str,
        actor: str,
        data: dict[str, Any] | None = None,
        target_id: str | None = None,
        epoch: int | None = None,
    ) -> dict[str, Any]:
        if etype not in EVENT_TYPES:
            raise ValueError(f"unknown event type {etype!r}")
        rec: dict[str, Any] = {"t": t, "type": etype, "actor": actor}
        if target_id is not None:
            rec["target_id"] = target_id
        if epoch is not None:
            rec["epoch"] = epoch
        rec["data"] = data or {}
        rec = _clean(rec)
        self.records.append(rec)
        if etype not in NOISY:
            self.important.append(rec)
        self._lines.append(json.dumps(rec, sort_keys=True, separators=(",", ":")))
        return rec

    def extend(self, records: Iterable[dict[str, Any]]) -> None:
        for r in records:
            self.append(r["t"], r["type"], r["actor"], r.get("data"), r.get("target_id"), r.get("epoch"))

    def of_type(self, etype: str) -> list[dict[str, Any]]:
        return [r for r in self.records if r["type"] == etype]

    def tail(self, n: int) -> list[dict[str, Any]]:
        return self.records[-n:]

    def text(self) -> str:
        return "".join(line + "\n" for line in self._lines)

    def digest(self) -> str:
        return hashlib.sha256(self.text().encode("utf-8")).hexdigest()

    def write(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.text(), encoding="utf-8")
        return path
