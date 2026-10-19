"""Drone behaviour state machine (PLAN.md section 6.2).

All modes are declared so later phases do not change the enum, but the
transition table only contains the transitions implemented so far.
Phase 1: PATROL, RETURNING, FAILED.
Phase 2: + CONFIRM_HOLD, SAFETY_HOLD, LANDED.
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


M = DroneMode
IMPLEMENTED_MODES = frozenset(
    {M.PATROL, M.CONFIRM_HOLD, M.RETURNING, M.SAFETY_HOLD, M.LANDED, M.FAILED}
)
MOVING = frozenset({M.PATROL, M.CONFIRM_HOLD, M.RETURNING})

# (from, to) -> trigger description. `None` as `from` means "any mode".
TRANSITIONS: dict[tuple[DroneMode | None, DroneMode], str] = {
    (M.PATROL, M.CONFIRM_HOLD): "own_detection_unconfirmed",
    (M.CONFIRM_HOLD, M.PATROL): "confirmed_cancelled_or_hold_max",
    (M.PATROL, M.RETURNING): "battery_reserve",
    (M.CONFIRM_HOLD, M.RETURNING): "battery_reserve",
    **{(m, M.SAFETY_HOLD): "monitor_hold_or_replan" for m in MOVING},
    **{(M.SAFETY_HOLD, m): "replan_succeeded" for m in MOVING},  # back to the previous mode
    (M.RETURNING, M.PATROL): "reached_home_or_patrol",
    (None, M.LANDED): "no_route_home_battery_critical",
    (None, M.FAILED): "scenario_failure",
}


class IllegalDroneTransition(RuntimeError):
    pass


def can_transition(frm: DroneMode, to: DroneMode) -> bool:
    if frm is DroneMode.FAILED:
        return False  # FAILED is terminal: an injected failure is permanent for the run
    if frm is DroneMode.LANDED and to is not DroneMode.FAILED:
        return False  # an emergency landing ends the flight
    return (frm, to) in TRANSITIONS or (None, to) in TRANSITIONS


def check_transition(frm: DroneMode, to: DroneMode) -> None:
    if not can_transition(frm, to):
        raise IllegalDroneTransition(f"illegal drone mode transition {frm.value} -> {to.value}")
