"""Runtime safety monitor (PLAN.md section 9.4).

Checked every tick, before the drone moves. Phase 2 implements the geofence,
altitude and battery checks; the separation check needs heartbeats and is
added in Phase 3.

Routes are validated in full when they are assigned (planner + validator), and
the geofence is static in Phases 1-5, so a valid route stays valid. Each tick
the monitor checks the proposed next position and the segment the drone is
about to fly, which catches a planner or navigation bug at the step it occurs.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from src.drone.battery import Battery
from src.safety.geofence import Geofence
from src.safety.route_validator import Waypoint, validate_segment

VerdictKind = Literal["OK", "HOLD", "REPLAN", "RETURN", "LAND"]


@dataclass(frozen=True)
class SafetyVerdict:
    kind: VerdictKind
    reason: str = ""

    @property
    def ok(self) -> bool:
        return self.kind == "OK"


OK = SafetyVerdict("OK")


def check_energy(battery: Battery, e_home: float | None, lookahead_wh: float, check_reserve: bool) -> SafetyVerdict:
    """Battery checks. `e_home` is the energy of the planned route home, or None if there is no valid route."""
    if battery.empty:
        return SafetyVerdict("LAND", "battery_depleted")
    if e_home is None and battery.charge_wh < battery.critical_wh:
        return SafetyVerdict("LAND", "no_route_home_battery_critical")
    if check_reserve and e_home is not None and battery.reserve_violated(e_home + lookahead_wh):
        return SafetyVerdict("RETURN", "battery_reserve")
    return OK


def check_step(current: Waypoint, proposed: Waypoint, fence: Geofence) -> SafetyVerdict:
    """Geofence checks on the move the drone is about to make."""
    reason = fence.violation(*proposed)
    if reason:
        return SafetyVerdict("HOLD", reason)
    seg = validate_segment(current, proposed, fence)
    if not seg.ok:
        return SafetyVerdict("REPLAN", seg.reason or "segment_invalid")
    return OK
