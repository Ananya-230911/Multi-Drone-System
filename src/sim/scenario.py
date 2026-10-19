"""Scheduled and operator-injected scenario events (PLAN.md section 4.1 rule 6).

Scenario events are test inputs, not coordination: they are the only way the
outside world (config file or dashboard keys) touches an agent, and each one
is logged as SCENARIO_EVENT.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from src.config import ScenarioEventConfig

if TYPE_CHECKING:
    from src.drone.agent import Drone
    from src.logging_.event_log import EventLog

# Event types this phase applies. Others are accepted, logged and ignored.
IMPLEMENTED = {"fail_drone", "set_battery"}
PHASE_OF = {"external_cue": 3, "place_target": 2, "hide_target": 2, "show_target": 2}


@dataclass(order=True)
class _Pending:
    t: float
    seq: int
    type: str = field(compare=False)
    data: dict[str, Any] = field(compare=False)
    source: str = field(compare=False)


class Scenario:
    def __init__(self, events: list[ScenarioEventConfig]):
        self._seq = 0
        self._pending: list[_Pending] = []
        for e in events:
            self._push(e.t, e.type, dict(e.data), "config")

    def inject(self, t: float, etype: str, data: dict[str, Any] | None = None) -> None:
        """Queue an operator event (from the dashboard) to apply at the next tick at or after `t`."""
        self._push(t, etype, dict(data or {}), "operator")

    def pending(self) -> int:
        return len(self._pending)

    def apply(self, now: float, drones: dict[str, "Drone"], log: "EventLog") -> None:
        due = [p for p in self._pending if p.t <= now + 1e-9]
        if not due:
            return
        self._pending = [p for p in self._pending if p.t > now + 1e-9]
        for p in due:  # already sorted by (t, seq)
            applied, note = self._apply_one(now, p, drones)
            data = {"type": p.type, "source": p.source, "scheduled_t": p.t, "applied": applied, **p.data}
            if note:
                data["note"] = note
            log.append(now, "SCENARIO_EVENT", "SCENARIO", data)

    def _apply_one(self, now: float, p: _Pending, drones: dict[str, "Drone"]) -> tuple[bool, str]:
        if p.type not in IMPLEMENTED:
            return False, f"'{p.type}' is handled from Phase {PHASE_OF.get(p.type, '?')}"
        drone = drones.get(p.data.get("drone", ""))
        if drone is None:
            return False, f"unknown drone {p.data.get('drone')!r}"
        if p.type == "fail_drone":
            drone.inject_failure(now)
        elif p.type == "set_battery":
            drone.inject_battery(now, float(p.data["battery_pct"]))
        return True, ""

    def _push(self, t: float, etype: str, data: dict[str, Any], source: str) -> None:
        self._pending.append(_Pending(t, self._seq, etype, data, source))
        self._seq += 1
        self._pending.sort()
