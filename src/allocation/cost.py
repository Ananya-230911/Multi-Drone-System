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
# Reasons a drone cannot bid that pass with time. If every refusal is something else (no inspection point, no route,
# not enough battery ...) no later auction will do better and the target is cancelled as before.
TRANSIENT_QUEUE = frozenset({"queue_full", "mode_SAFETY_HOLD"})
TRANSIENT_SINGLE = frozenset({"busy", "mode_ENROUTE", "mode_INSPECTING", "mode_REACQUIRING", "mode_RETURNING", "mode_SAFETY_HOLD"})
QUEUE_MODES = ("PATROL", "CONFIRM_HOLD", "ENROUTE", "INSPECTING", "REACQUIRING")  # allocation.max_queue > 1; plus RETURNING when free_return


@dataclass(frozen=True)
class Stop:
    """One task a drone already holds (its active task first, then its queue), as the drone itself knows it."""

    target_pos: Point2
    hold_s: float  # inspection time still to spend there
    point: Point3 | None = None  # inspection point, when already chosen (the active task)
    active: bool = False


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
    stops: tuple[Stop, ...] = ()  # tasks already held, in the order they will be flown (queue mode only)
    free_return: bool = False  # RETURNING to the patrol loop with nothing left to do (not heading home for battery)


@dataclass(frozen=True)
class CostConfig:
    weights: AllocationWeights
    inspection: InspectionConfig
    confidence: ConfidenceConfig
    max_queue: int = 1  # allocation.max_queue; 1 = a drone holds a single task and busy drones do not bid


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

    if cfg.max_queue > 1:
        return _evaluate_queued(bidder, target_id, target_pos, planner, cfg, excluded, infeasible)
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


def _evaluate_queued(bidder: BidderState, target_id: str, target_pos: Point2, planner: Planner, cfg: CostConfig,
                     excluded: bool, infeasible) -> Bid:  # fmt: skip
    """Queue mode (allocation.max_queue > 1): the bid is the marginal cost of inserting the target into the
    tasks the drone already holds, at the cheapest position behind its active task.

        t_go    time until the drone reaches the target (waiting behind earlier tasks + travel)
        E_need  extra energy the whole plan needs because of the target (equals the old E_need for an idle drone)
        batt_after = battery - energy of the WHOLE plan including the target and the way home; vetoed below the reserve

    An idle drone has no stops, so its bid is identical to the single-task bid.
    """
    p = bidder.params
    if bidder.docked:
        return infeasible("parked_at_home")
    if bidder.mode not in QUEUE_MODES and not (bidder.mode == "RETURNING" and bidder.free_return):
        return infeasible(f"mode_{bidder.mode}")
    if len(bidder.stops) >= cfg.max_queue:
        return infeasible("queue_full", queue_len=len(bidder.stops))
    if excluded:
        return infeasible("excluded_this_epoch")
    insp_alt = cfg.inspection.standoff_alt
    r_eff = effective_radius(p.sensor, insp_alt)
    if r_eff <= 0:
        return infeasible("sensor_not_working_at_standoff_alt")
    standoff = inspection_standoff(r_eff, cfg.confidence, cfg.inspection)
    model = EnergyModel.from_params(p)
    legs: dict[tuple[Point3, Point3], Any] = {}

    def leg(a: Point3, b: Point3):
        if (a, b) not in legs:
            res = planner.plan(a, b)
            legs[(a, b)] = None if res.route is None else (res.route, route_cost(res.route[0], res.route[1:], model))
        return legs[(a, b)]

    def point_for(pos: Point3, tpos: Point2) -> Point3 | None:
        return inspection_point(tpos, r_eff, planner, insp_alt, cfg.inspection.candidate_count,
                                from_pos=pos[:2], standoff=standoff)  # fmt: skip

    held: list[tuple[Point3, float]] = []  # (inspection point, hold seconds) of the tasks already held
    prev = bidder.pos
    for st in bidder.stops:
        pt = st.point or point_for(prev, st.target_pos)
        if pt is None:
            return infeasible("held_task_unreachable")
        held.append((pt, st.hold_s))
        prev = pt

    def plan_cost(seq: list[tuple[Point3, float]], new_index: int | None):
        """(time to reach seq[new_index] or None, total energy, first-leg data of the new stop, error)"""
        cur, t, e, t_new, new_leg = bidder.pos, 0.0, 0.0, None, None
        for i, (pt, hold) in enumerate(seq):
            lg = leg(cur, pt)
            if lg is None:
                return None, None, None, "no_route"
            rc = lg[1]
            t += rc.horizontal / p.max_speed + rc.climb / p.climb_rate + rc.descent / p.climb_rate
            e += rc.energy_wh
            if i == new_index:
                t_new, new_leg = t, lg
            t += hold
            e += p.hover_power * hold * cfg.inspection.energy_margin
            cur = pt
        if seq:
            home = leg(cur, bidder.home)
            if home is None:
                return None, None, None, "no_route_home"
            e += home[1].energy_wh
        return t_new, e, new_leg, None

    _, e_without, _, err = plan_cost(held, None)
    if err:
        return infeasible("held_task_unreachable", route_reason=err)
    first = 1 if bidder.stops and bidder.stops[0].active else 0
    best = None
    last_err = "no_route"
    for k in range(first, len(held) + 1):
        before = held[k - 1][0] if k else bidder.pos
        new_pt = point_for(before, target_pos)
        if new_pt is None:
            last_err = "no_inspection_point"
            continue
        seq = held[:k] + [(new_pt, cfg.inspection.duration)] + held[k:]
        t_go, e_total, new_leg, err = plan_cost(seq, k)
        if err:
            last_err = err
            continue
        e_need = e_total - e_without
        batt_after = bidder.battery_wh - e_total
        w = cfg.weights
        cost = w.time * t_go + w.energy * (e_need / p.capacity_wh) + w.battery * (1.0 - batt_after / p.capacity_wh)
        if best is None or cost < best[0] - TIE_EPS:
            best = (cost, k, new_pt, t_go, e_need, e_total, batt_after, new_leg)
    if best is None:
        return infeasible(last_err)
    cost, k, new_pt, t_go, e_need, e_total, batt_after, (route, rc) = best
    reserve = p.reserve_pct * p.capacity_wh
    breakdown = {
        "L_go": route_length(route), "climb": rc.climb, "descent": rc.descent, "t_go": t_go,
        "E_go": rc.energy_wh, "E_insp": p.hover_power * cfg.inspection.duration * cfg.inspection.energy_margin,
        "E_home": 0.0, "E_need": e_need, "E_total": e_total, "battery_wh": bidder.battery_wh, "batt_after": batt_after,
        "reserve_wh": reserve, "cost": cost, "r_eff": r_eff, "standoff": standoff, "inspection_point": list(new_pt),
        "route_go": [list(w_) for w_ in route], "insert_at": k, "held": len(bidder.stops),
    }  # fmt: skip
    if batt_after < reserve:
        return Bid(bidder.id, target_id, False, None, "insufficient_energy", breakdown)
    return Bid(bidder.id, target_id, True, -cost, "ok", breakdown)
