"""Fix 1 (queue): marginal-cost bids for drones that already hold tasks, vetoed by the battery over the whole plan."""

import pytest

from src.allocation.cost import (TRANSIENT_QUEUE, TRANSIENT_SINGLE, BidderState, CostConfig, Stop, evaluate)
from src.config import AllocationConfig, AllocationWeights, ConfidenceConfig, DroneParams, InspectionConfig
from src.safety.geofence import Geofence
from src.safety.planner import Planner

SQUARE = [(50, 50), (1950, 50), (1950, 1950), (50, 1950)]
P = DroneParams()
SINGLE = CostConfig(AllocationWeights(), InspectionConfig(), ConfidenceConfig())
QUEUE = CostConfig(AllocationWeights(), InspectionConfig(), ConfidenceConfig(), max_queue=4)
TARGET = (1300.0, 900.0)
HOME = (100.0, 100.0, 0.0)


@pytest.fixture(scope="module")
def planner():
    return Planner(Geofence(SQUARE, [], 15, (15, 120), homes=[HOME[:2]]), 60, 1.0)


def bidder(pos=(1300.0, 600.0, 60.0), battery=80.0, mode="PATROL", **kw):
    return BidderState("D1", pos, battery, P, HOME, mode, **kw)


def busy_stops():
    """Active task (hold left 5 s) at (900, 600), one queued task at (1500, 500)."""
    return (Stop((900.0, 600.0), 5.0, (900.0, 600.0, 60.0), True), Stop((1500.0, 500.0), 10.0))


def test_idle_drone_bids_exactly_like_the_single_task_bid(planner):
    for pos in [(1300.0, 600.0, 60.0), (300.0, 1500.0, 60.0), (1800.0, 200.0, 60.0)]:
        old = evaluate(bidder(pos), "T", TARGET, planner, SINGLE)
        new = evaluate(bidder(pos), "T", TARGET, planner, QUEUE)
        assert new.feasible and new.value == old.value
        assert new.breakdown["t_go"] == old.breakdown["t_go"] and new.breakdown["E_need"] == pytest.approx(old.breakdown["E_need"])
        assert new.breakdown["insert_at"] == 0


def test_single_task_mode_still_refuses_busy_drones(planner):
    assert evaluate(bidder(busy=True), "T", TARGET, planner, SINGLE).reason == "busy"
    assert evaluate(bidder(mode="ENROUTE"), "T", TARGET, planner, SINGLE).reason == "mode_ENROUTE"


def test_busy_drone_bids_marginal_cost_behind_its_active_task(planner):
    idle = evaluate(bidder(), "T", TARGET, planner, QUEUE)
    b = evaluate(bidder(mode="INSPECTING", busy=True, stops=busy_stops()), "T", TARGET, planner, QUEUE)
    assert b.feasible and b.reason == "ok"
    assert b.breakdown["insert_at"] >= 1  # never before the task it is already flying
    assert b.breakdown["held"] == 2
    assert b.breakdown["t_go"] > idle.breakdown["t_go"]  # it first has to finish what it holds (5 s hold + the legs)
    assert b.value < idle.value  # so a free drone at the same spot wins the auction


def test_queue_is_capped_by_max_queue(planner):
    stops = busy_stops() + (Stop((1600.0, 1500.0), 10.0), Stop((400.0, 1700.0), 10.0))
    assert evaluate(bidder(mode="ENROUTE", stops=stops), "T", TARGET, planner, QUEUE).reason == "queue_full"
    three = evaluate(bidder(mode="ENROUTE", stops=stops[:3]), "T", TARGET, planner, QUEUE)
    assert three.feasible


def test_battery_reserve_is_checked_over_the_whole_queue(planner):
    ok = evaluate(bidder(mode="ENROUTE", stops=busy_stops(), battery=80.0), "T", TARGET, planner, QUEUE)
    assert ok.feasible
    need = ok.breakdown["E_total"]
    reserve = P.reserve_pct * P.capacity_wh
    short = evaluate(bidder(mode="ENROUTE", stops=busy_stops(), battery=need + reserve - 1.0), "T", TARGET, planner, QUEUE)
    assert short.feasible is False and short.reason == "insufficient_energy"
    # the same battery would have been enough for the new task alone: the queue is what makes it infeasible
    assert evaluate(bidder(battery=need + reserve - 1.0), "T", TARGET, planner, QUEUE).feasible


def test_returning_home_stays_ineligible_but_returning_to_patrol_may_bid(planner):
    assert evaluate(bidder(mode="RETURNING"), "T", TARGET, planner, QUEUE).reason == "mode_RETURNING"
    assert evaluate(bidder(mode="RETURNING", free_return=True), "T", TARGET, planner, QUEUE).feasible
    assert evaluate(bidder(mode="SAFETY_HOLD"), "T", TARGET, planner, QUEUE).reason == "mode_SAFETY_HOLD"


def test_unreachable_held_task_makes_the_bid_infeasible_not_a_crash(planner):
    far_zone = Planner(Geofence(SQUARE, [("RZ", [(1200, 800), (1400, 800), (1400, 1000), (1200, 1000)])], 15, (15, 120),
                                homes=[HOME[:2]]), 60, 1.0)
    inside = (Stop((1300.0, 900.0), 10.0),)  # a held task deep inside a no-fly zone
    b = evaluate(bidder(mode="ENROUTE", stops=inside), "T", (500.0, 500.0), far_zone, QUEUE)
    assert b.feasible is False and b.reason in ("held_task_unreachable", "no_route", "no_inspection_point")


def test_transient_reasons_never_include_geometry_or_battery():
    for bad in ("no_inspection_point", "no_route", "no_route_home", "insufficient_energy"):
        assert bad not in TRANSIENT_QUEUE and bad not in TRANSIENT_SINGLE


def test_new_allocation_keys_and_defaults():
    a = AllocationConfig()
    assert (a.max_queue, a.retry_unassigned, a.retry_period, a.retry_max_s, a.skip_completed_sites) == (4, True, 10.0, 600.0, True)
    with pytest.raises(Exception):
        AllocationConfig(max_queue=0).validate("allocation")
