import math
from dataclasses import replace

import pytest

from src.allocation.cost import TIE_EPS, BidderState, CostConfig, better, evaluate, inspection_standoff
from src.config import AllocationWeights, ConfidenceConfig, DroneParams, InspectionConfig, SensorConfig
from src.safety.geofence import Geofence
from src.safety.planner import Planner

SQUARE = [(50, 50), (1950, 50), (1950, 1950), (50, 1950)]
ZONE = ("RZ-1", [(800, 600), (1100, 600), (1100, 1000), (800, 1000)])
P = DroneParams()
CFG = CostConfig(AllocationWeights(), InspectionConfig(), ConfidenceConfig())
TARGET = (1300.0, 900.0)


@pytest.fixture(scope="module")
def planner():
    f = Geofence(SQUARE, [ZONE], 15, (15, 120), homes=[(100, 100)])
    return Planner(f, 60, 1.0)


def bidder(id="D1", pos=(1300.0, 600.0, 60.0), battery=80.0, params=P, mode="PATROL", **kw):
    return BidderState(id, pos, battery, params, (100.0, 100.0, 0.0), mode, **kw)


def test_inspection_standoff_matches_confidence_model():
    # aim halfway between min_confidence 0.6 and base 0.95: 0.95 * (1 - d/60)^1.5 = 0.775
    d = inspection_standoff(60, ConfidenceConfig(), InspectionConfig())
    assert d == pytest.approx(60 * (1 - (0.775 / 0.95) ** (2 / 3)))
    assert 0.95 * (1 - d / 60) ** 1.5 == pytest.approx(0.775)
    assert inspection_standoff(60, ConfidenceConfig(base=0.5), InspectionConfig()) == 0.0


def test_feasible_bid_has_full_breakdown(planner):
    b = evaluate(bidder(), "T", TARGET, planner, CFG)
    assert b.feasible and b.reason == "ok" and b.value == pytest.approx(-b.breakdown["cost"])
    bd = b.breakdown
    for k in ("L_go", "climb", "descent", "t_go", "E_go", "E_insp", "E_home", "E_need", "batt_after", "cost", "r_eff"):
        assert k in bd
    assert bd["E_insp"] == pytest.approx(P.hover_power * 10.0 * 1.2)
    assert bd["E_need"] == pytest.approx(bd["E_go"] + bd["E_insp"] + bd["E_home"])
    assert bd["batt_after"] == pytest.approx(80.0 - bd["E_need"])
    w = AllocationWeights()
    expect = w.time * bd["t_go"] + w.energy * bd["E_need"] / 100 + w.battery * (1 - bd["batt_after"] / 100)
    assert bd["cost"] == pytest.approx(expect)
    assert math.dist(bd["inspection_point"][:2], TARGET) == pytest.approx(bd["standoff"])


def test_closer_drone_gets_better_bid(planner):
    near = evaluate(bidder("D1", (1300.0, 700.0, 60.0)), "T", TARGET, planner, CFG)
    far = evaluate(bidder("D2", (1800.0, 200.0, 60.0)), "T", TARGET, planner, CFG)
    assert better(near.value, "D1", far.value, "D2")


def test_larger_effective_sensor_radius_shortens_route(planner):
    small = replace(P, sensor=replace(SensorConfig(), radius=30))  # r_eff 30 at 60 m
    a = evaluate(bidder(params=small), "T", TARGET, planner, CFG)
    b = evaluate(bidder(params=P), "T", TARGET, planner, CFG)  # r_eff 60
    assert b.breakdown["L_go"] < a.breakdown["L_go"]
    assert b.value > a.value


def test_route_length_uses_planned_detour(planner):
    # From west of RZ-1 the route must go around it, so L_go exceeds the straight line.
    b = evaluate(bidder(pos=(700.0, 800.0, 60.0)), "T", TARGET, planner, CFG)
    assert b.feasible and b.breakdown["L_go"] > math.dist((700, 800), TARGET)


def test_insufficient_energy_is_infeasible(planner):
    b = evaluate(bidder(battery=25.0, pos=(1800.0, 1800.0, 60.0)), "T", TARGET, planner, CFG)
    assert not b.feasible and b.reason == "insufficient_energy" and b.value is None
    assert b.breakdown["batt_after"] < b.breakdown["reserve_wh"]


def test_no_inspection_point_is_infeasible(planner):
    b = evaluate(bidder(), "T", (950.0, 800.0), planner, CFG)  # deep inside RZ-1
    assert not b.feasible and b.reason == "no_inspection_point"


def test_no_route_is_infeasible():
    wall = Geofence(SQUARE, [("WALL", [(900, 0), (1100, 0), (1100, 2000), (900, 2000)])], 15, (15, 120), homes=[(100, 100)])
    b = evaluate(bidder(pos=(500.0, 1000.0, 60.0)), "T", (1500.0, 1000.0), Planner(wall, 60, 1.0), CFG)
    assert not b.feasible and b.reason == "no_route"


@pytest.mark.parametrize("kw, reason", [
    (dict(mode="RETURNING"), "mode_RETURNING"),
    (dict(mode="SAFETY_HOLD"), "mode_SAFETY_HOLD"),
    (dict(docked=True), "parked_at_home"),
    (dict(busy=True), "busy"),
])  # fmt: skip
def test_ineligible_drones_report_why(planner, kw, reason):
    b = evaluate(bidder(**kw), "T", TARGET, planner, CFG)
    assert not b.feasible and b.reason == reason


def test_excluded_drone_cannot_bid(planner):
    assert evaluate(bidder(), "T", TARGET, planner, CFG, excluded=True).reason == "excluded_this_epoch"


def test_confirm_hold_is_eligible(planner):
    assert evaluate(bidder(mode="CONFIRM_HOLD"), "T", TARGET, planner, CFG).feasible


def test_deterministic_tie_break():
    assert better(-5.0, "D1", -5.0, "D2") and not better(-5.0, "D2", -5.0, "D1")
    assert better(-5.0, "D1", -5.0 + TIE_EPS / 2, "D2")  # within epsilon: equal, lower ID wins
    assert better(-4.0, "D3", -5.0, "D1")  # higher bid wins regardless of ID
    assert better(-100.0, "D9", None, None) and not better(None, None, -100.0, "D9")
