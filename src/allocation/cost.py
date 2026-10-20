"""Bid cost and feasibility, with a full breakdown (PLAN.md sections 8.1 and 8.2).

    L_go    planned route length to the inspection point (horizontal, m)
    t_go    L_go / max_speed + climb / climb_rate + descent / climb_rate
    E_go    k_h*L_go + k_c*climb + k_d*descent
    E_insp  hover_power * inspection.duration * inspection.energy_margin
    E_home  route energy from the inspection point to home
    E_need  E_go + E_insp + E_home
    batt_after = battery - E_need
    feasible = eligible AND route_go valid AND route_home valid AND batt_after >= reserve
    cost = w_time*t_go + w_energy*(E_need/capacity) + w_battery*(1 - batt_after/capacity)
    bid  = -cost (only if feasible)

The geofence is never a cost term: it enters only through route feasibility and
route length. The inspection point sits `inspection_standoff` from the target,
where the (noise-free) confidence model is comfortably above
`inspection.min_confidence`, so a drone with a larger effective sensor radius
needs a shorter route.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from src.config import AllocationWeights, ConfidenceConfig, DroneParams, InspectionConfig, Point2, Point3
from src.drone.battery import EnergyModel, route_cost
from src.drone.sensor import effective_radius
from src.safety.planner import Planner, inspection_point
from src.safety.route_validator import route_length

TIE_EPS = 1e-9  # bids this close are equal; the lower drone ID wins
ELIGIBLE_MODES = ("PATROL", "CONFIRM_HOLD")


@dataclass(frozen=True)
class BidderState:
    """What a drone knows about itself when it bids. Built from its own private state."""

    id: str
    pos: Point3
    battery_wh: float
    params: DroneParams
    home: Point3
    mode: str
    docked: bool = False
    busy: bool = False  # already owns another task


@dataclass(frozen=True)
class CostConfig:
    weights: AllocationWeights
    inspection: InspectionConfig
    confidence: ConfidenceConfig


@dataclass(frozen=True)
class Bid:
    drone: str
    target_id: str
    feasible: bool
    value: float | None  # -cost when feasible
    reason: str  # "ok" or why the drone cannot compete
    breakdown: dict[str, Any] = field(default_factory=dict)


def inspection_standoff(r_eff: float, conf: ConfidenceConfig, insp: InspectionConfig) -> float:
    """Distance from the target at which the noise-free confidence base*(1-d/r_eff)^falloff is
    halfway between inspection.min_confidence and base.

    Aiming at the threshold itself would leave about half of the noisy observations below it, so
    the inspection timer would barely advance; the midpoint keeps a margin of
    (base - min_confidence) / 2 above the threshold.
    """
    if insp.min_confidence >= conf.base or r_eff <= 0:
        return 0.0
    aim = (insp.min_confidence + conf.base) / 2.0
    return r_eff * (1.0 - (aim / conf.base) ** (1.0 / conf.falloff_exp))


def better(bid_a: float | None, id_a: str | None, bid_b: float | None, id_b: str | None) -> bool:
    """Compare rule (8.3): higher bid wins; equal bids (within TIE_EPS) -> lower drone ID wins.

    A missing bid (no feasible winner) loses to any bid.
    """
    if bid_a is None or id_a is None:
        return False
    if bid_b is None or id_b is None:
        return True
    if bid_a > bid_b + TIE_EPS:
        return True
    if abs(bid_a - bid_b) <= TIE_EPS:
        return id_a < id_b
    return False


def evaluate(bidder: BidderState, target_id: str, target_pos: Point2, planner: Planner, cfg: CostConfig,
             excluded: bool = False) -> Bid:  # fmt: skip
    p = bidder.params

    def infeasible(reason: str, **extra: Any) -> Bid:
        return Bid(bidder.id, target_id, False, None, reason, {"battery_wh": bidder.battery_wh, **extra})

    if bidder.docked:
        return infeasible("parked_at_home")
    if bidder.mode not in ELIGIBLE_MODES:
        return infeasible(f"mode_{bidder.mode}")
    if bidder.busy:
        return infeasible("busy")
    if excluded:
        return infeasible("excluded_this_epoch")
    insp_alt = cfg.inspection.standoff_alt
    r_eff = effective_radius(p.sensor, insp_alt)
    if r_eff <= 0:
        return infeasible("sensor_not_working_at_standoff_alt")
    standoff = inspection_standoff(r_eff, cfg.confidence, cfg.inspection)
    insp = inspection_point(target_pos, r_eff, planner, insp_alt, cfg.inspection.candidate_count,
                            from_pos=bidder.pos[:2], standoff=standoff)  # fmt: skip
    if insp is None:
        return infeasible("no_inspection_point")
    go = planner.plan(bidder.pos, insp)
    if go.route is None:
        return infeasible("no_route", route_reason=go.reason)
    home = planner.plan(insp, bidder.home)
    if home.route is None:
        return infeasible("no_route_home", route_reason=home.reason)
    model = EnergyModel.from_params(p)
    rc_go = route_cost(go.route[0], go.route[1:], model)
    rc_home = route_cost(home.route[0], home.route[1:], model)
    t_go = rc_go.horizontal / p.max_speed + rc_go.climb / p.climb_rate + rc_go.descent / p.climb_rate
    e_insp = p.hover_power * cfg.inspection.duration * cfg.inspection.energy_margin
    e_need = rc_go.energy_wh + e_insp + rc_home.energy_wh
    batt_after = bidder.battery_wh - e_need
    reserve = p.reserve_pct * p.capacity_wh
    w = cfg.weights
    cost = w.time * t_go + w.energy * (e_need / p.capacity_wh) + w.battery * (1.0 - batt_after / p.capacity_wh)
    breakdown = {
        "L_go": route_length(go.route), "climb": rc_go.climb, "descent": rc_go.descent, "t_go": t_go,
        "E_go": rc_go.energy_wh, "E_insp": e_insp, "E_home": rc_home.energy_wh, "E_need": e_need,
        "battery_wh": bidder.battery_wh, "batt_after": batt_after, "reserve_wh": reserve, "cost": cost,
        "r_eff": r_eff, "standoff": standoff, "inspection_point": list(insp), "route_go": [list(w_) for w_ in go.route],
    }  # fmt: skip
    if batt_after < reserve:
        return Bid(bidder.id, target_id, False, None, "insufficient_energy", breakdown)
    return Bid(bidder.id, target_id, True, -cost, "ok", breakdown)
