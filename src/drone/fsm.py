"""Drone behaviour state machine (PLAN.md section 6.2).

All modes are declared so later phases do not change the enum, but the
transition table only contains the transitions implemented so far.
Phase 1: PATROL, RETURNING, FAILED.
"""

from __future__ import annotations

from enum import Enum


class DroneMode(str, Enum):
    PATROL = "PATROL"
    CONFIRM_HOLD = "CONFIRM_HOLD"
    ENROUTE = "ENROUTE"
    INSPECTING = "INSPECTING"
    REACQUIRING = "REACQUIRING"
    RETURNING = "RETURNING"
    SAFETY_HOLD = "SAFETY_HOLD"
    LANDED = "LANDED"
    FAILED = "FAILED"


IMPLEMENTED_MODES = frozenset({DroneMode.PATROL, DroneMode.RETURNING, DroneMode.FAILED})

# (from, to) -> trigger description. `None` as `from` means "any mode".
TRANSITIONS: dict[tuple[DroneMode | None, DroneMode], str] = {
    (DroneMode.PATROL, DroneMode.RETURNING): "battery_reserve",
    (DroneMode.RETURNING, DroneMode.PATROL): "reached_home_or_patrol",
    (None, DroneMode.FAILED): "scenario_failure",
}


class IllegalDroneTransition(RuntimeError):
    pass


def can_transition(frm: DroneMode, to: DroneMode) -> bool:
    if frm is DroneMode.FAILED:
        return False  # FAILED is terminal: an injected failure is permanent for the run
    return (frm, to) in TRANSITIONS or (None, to) in TRANSITIONS


def check_transition(frm: DroneMode, to: DroneMode) -> None:
    if not can_transition(frm, to):
        raise IllegalDroneTransition(f"illegal drone mode transition {frm.value} -> {to.value}")
