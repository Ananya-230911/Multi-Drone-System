"""Drone behaviour state machine (PLAN.md section 6.2).

All modes are declared so later phases do not change the enum, but the
transition table only contains the transitions implemented so far.
Phase 1: PATROL, RETURNING, FAILED.
Phase 2: + CONFIRM_HOLD, SAFETY_HOLD, LANDED.
Phase 4: + ENROUTE, INSPECTING, REACQUIRING (the owner's mission). All modes are now used.
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
IMPLEMENTED_MODES = frozenset(DroneMode)
MOVING = frozenset({M.PATROL, M.CONFIRM_HOLD, M.RETURNING, M.ENROUTE, M.INSPECTING, M.REACQUIRING})
MISSION = frozenset({M.ENROUTE, M.INSPECTING, M.REACQUIRING})

# (from, to) -> trigger description. `None` as `from` means "any mode".
TRANSITIONS: dict[tuple[DroneMode | None, DroneMode], str] = {
    (M.PATROL, M.CONFIRM_HOLD): "own_detection_unconfirmed",
    (M.CONFIRM_HOLD, M.PATROL): "confirmed_cancelled_or_hold_max",
    (M.PATROL, M.ENROUTE): "committed_owner_route_valid",
    (M.CONFIRM_HOLD, M.ENROUTE): "committed_owner_route_valid",
    (M.PATROL, M.INSPECTING): "committed_owner_already_in_position",
    (M.CONFIRM_HOLD, M.INSPECTING): "committed_owner_already_in_position",
    (M.ENROUTE, M.INSPECTING): "arrived_at_inspection_point",
    (M.INSPECTING, M.REACQUIRING): "track_lost",
    (M.REACQUIRING, M.INSPECTING): "fresh_observation_in_region",
    (M.INSPECTING, M.RETURNING): "logged_completed_or_task_released",
    (M.REACQUIRING, M.RETURNING): "reacquire_timeout_or_task_released",
    (M.ENROUTE, M.RETURNING): "task_released",
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
