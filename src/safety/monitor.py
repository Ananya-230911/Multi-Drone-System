"""Runtime safety monitor (PLAN.md section 9.4).

Checked every tick, before the drone moves: geofence, altitude, battery and
(from Phase 3) predicted separation from peers, using heartbeat data only.

Routes are validated in full when they are assigned (planner + validator), and
the geofence is static in Phases 1-5, so a valid route stays valid. Each tick
the monitor checks the proposed next position and the segment the drone is
about to fly, which catches a planner or navigation bug at the step it occurs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Literal

from src.config import SafetyConfig
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


@dataclass(frozen=True)
class PeerTrack:
    """A peer as known from its last heartbeat, extrapolated to now."""

    id: str
    pos: Waypoint
    vel: tuple[float, float, float]
    has_task: bool


def min_distance(rel_pos: Waypoint, rel_vel: tuple[float, float, float], horizon: float) -> float:
    """Smallest distance over [0, horizon] between two points moving linearly (relative frame)."""
    vv = sum(v * v for v in rel_vel)
    t = 0.0 if vv <= 1e-12 else min(max(-sum(p * v for p, v in zip(rel_pos, rel_vel)) / vv, 0.0), horizon)
    return math.dist((0.0, 0.0, 0.0), tuple(p + v * t for p, v in zip(rel_pos, rel_vel)))


def check_separation(me: str, pos: Waypoint, vel: tuple[float, float, float], has_task: bool,
                     peers: Iterable[PeerTrack], cfg: SafetyConfig) -> SafetyVerdict:  # fmt: skip
    """Yield (HOLD) if a predicted conflict exists and this drone has the lower priority.

    Priority: a drone with a task beats one without; otherwise the lower ID has priority.
    Separation is predicted from heartbeats, so it can be wrong under loss or latency;
    the world measures true separation separately for evaluation.
    """
    threshold = cfg.min_separation + cfg.separation_buffer
    for p in sorted(peers, key=lambda p: p.id):
        rel_pos = tuple(b - a for a, b in zip(pos, p.pos))
        rel_vel = tuple(b - a for a, b in zip(vel, p.vel))
        if min_distance(rel_pos, rel_vel, cfg.separation_horizon) >= threshold:
            continue
        if has_task != p.has_task:
            i_yield = p.has_task
        else:
            i_yield = me > p.id
        if i_yield:
            return SafetyVerdict("HOLD", f"SEPARATION:{p.id}")
    return OK
