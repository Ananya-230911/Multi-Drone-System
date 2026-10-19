"""Phase 2 acceptance: detection, confirmation, lifecycle, geofence and route safety (headless)."""

import copy
from pathlib import Path

import pytest
import yaml

from src.config import ConfigError, config_from_dict, load_config
from src.safety.route_validator import validate_route
from src.sim.world import World

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ["default", "geofence_detour", "target_in_restricted", "false_observation", "duplicate_reports"]


def cfg(name, **sim):
    c = load_config(ROOT / "config" / f"{name}.yaml")
    if sim:
        import dataclasses

        c = dataclasses.replace(c, sim=dataclasses.replace(c.sim, **sim))
    return c


def raw(name):
    return yaml.safe_load((ROOT / "config" / f"{name}.yaml").read_text())


def records(world):
    return {d.id: list(d.targets) for d in world.snapshot.drones}


@pytest.fixture(scope="module")
def default_world():
    w = World(cfg("default"))
    w.run()
    return w


# ------------------------------------------------------------------ geofence, every tick, every scenario


@pytest.mark.parametrize("name", CONFIGS)
def test_no_position_outside_geofence_and_every_route_valid(name):
    w = World(cfg(name))
    while not w.finished:  # asserted every tick
        snap = w.step()
        for d in snap.drones:
            if d.mode != "LANDED":
                assert w.fence.violation(d.x, d.y, d.alt) is None, (snap.t, d.id, d.x, d.y, d.alt)
    assert w.geofence_violations == []
    planned = w.log.of_type("ROUTE_PLANNED")
    for ev in planned:
        route = [tuple(p) for p in ev["data"]["waypoints"]]
        assert validate_route(route, w.fence).ok, ev


# ------------------------------------------------------------------ default.yaml: detect -> confirm -> nothing more


def test_default_detector_creates_and_confirms_record(default_world):
    w = default_world
    recs = records(w)
    assert [t.id for t in recs["D2"]] == ["T-D2-001"]
    t = recs["D2"][0]
    assert t.state == "CONFIRMED" and t.confirmation == "CONFIRMED"
    gt = w.cfg.targets[0].pos
    assert abs(t.est_pos[0] - gt[0]) < 10 and abs(t.est_pos[1] - gt[1]) < 10
    assert all(not recs[d] for d in ("D1", "D3", "D4"))  # no messages yet: only the detector knows


def test_default_lifecycle_stops_at_confirmed(default_world):
    trans = [(e["data"]["from"], e["data"]["to"], e["data"]["row"]) for e in default_world.log.of_type("TARGET_TRANSITION")]
    assert trans == [("UNKNOWN", "DETECTED", 1), ("DETECTED", "CONFIRMED", 2)]


def test_default_confirm_hold_entered_and_left(default_world):
    modes = [(e["data"]["from"], e["data"]["to"], e["data"]["reason"])
             for e in default_world.log.of_type("DRONE_MODE") if e["actor"] == "D2"]  # fmt: skip
    assert modes[0][:2] == ("PATROL", "CONFIRM_HOLD")
    assert modes[1] == ("CONFIRM_HOLD", "PATROL", "record_confirmed")
    hold = next(e for e in default_world.log.of_type("DRONE_MODE") if e["data"]["to"] == "CONFIRM_HOLD")
    done = next(e for e in default_world.log.of_type("DRONE_MODE") if e["data"]["from"] == "CONFIRM_HOLD")
    assert done["t"] - hold["t"] <= default_world.cfg.confirmation.hold_max


def test_no_false_confirmations_default(default_world):
    assert default_world.false_confirmations() == []


# ------------------------------------------------------------------ false_observation.yaml


@pytest.mark.parametrize("seed", [1, 2, 3, 42])
def test_spurious_observations_never_confirm(seed):
    w = World(cfg("false_observation", seed=seed))
    w.run()
    assert sum(o.is_spurious for o in w.observations) > 100
    assert all(o.is_spurious for o in w.observations)
    all_recs = [t for ts in records(w).values() for t in ts]
    assert all_recs, "spurious observations should create (and then cancel) records"
    assert not [t for t in all_recs if t.confirmation == "CONFIRMED"]
    assert w.false_confirmations() == []
    assert not w.log.of_type("TARGET_TRANSITION") or all(
        e["data"]["to"] in ("DETECTED", "CANCELLED") for e in w.log.of_type("TARGET_TRANSITION")
    )


# ------------------------------------------------------------------ geofence_detour.yaml


def test_route_home_detours_around_wall():
    w = World(cfg("geofence_detour"))
    w.run()
    home_routes = [e for e in w.log.of_type("ROUTE_PLANNED") if e["actor"] == "D1" and e["data"]["kind"] == "home"]
    assert home_routes
    route = [tuple(p) for p in home_routes[0]["data"]["waypoints"]]
    assert len(route) >= 5  # start, two wall corners, over home, home
    straight = ((route[0][0] - route[-1][0]) ** 2 + (route[0][1] - route[-1][1]) ** 2) ** 0.5
    assert home_routes[0]["data"]["length"] > straight
    assert w.fence.horizontal_segment_violation(route[0][:2], route[-1][:2]) == "CROSSES_ZONE:RZ-WALL"
    d1 = w.snapshot.drone("D1")
    assert (d1.x, d1.y, d1.alt) == pytest.approx((150, 1000, 0)) and d1.docked
    assert [t.state for t in records(w)["D1"]] == ["CONFIRMED"]


# ------------------------------------------------------------------ target_in_restricted.yaml


def test_target_inside_zone_seen_from_edge_but_never_overflown():
    w = World(cfg("target_in_restricted"))
    w.run()
    holds = [e for e in w.log.of_type("ROUTE_PLANNED") if e["data"]["kind"] == "confirm_hold"]
    assert holds
    for h in holds:
        goal = tuple(h["data"]["waypoints"][-1])
        assert w.fence.contains_point(*goal)  # hold point is outside the zone
    recs = records(w)["D1"]
    assert recs and all(t.state == "CANCELLED" and t.confirmation == "REJECTED" for t in recs)
    assert w.geofence_violations == []


# ------------------------------------------------------------------ duplicate_reports.yaml


def test_each_detector_holds_one_local_record():
    w = World(cfg("duplicate_reports"))
    w.run()
    recs = records(w)
    for d in ("D1", "D2", "D3"):
        assert [t.id for t in recs[d]] == [f"T-{d}-001"]
        assert recs[d][0].state == "CONFIRMED"
    assert recs["D4"] == []


# ------------------------------------------------------------------ runtime monitor


def test_monitor_holds_on_invalid_step_and_replans():
    """Simulate a planner bug: hand a drone an unvalidated route straight through RZ-1."""
    from src.drone.navigation import Navigator

    w = World(cfg("default", duration=120))
    d1 = w.drones["D1"]
    for _ in range(10):
        w.step()
    d1.navigator = Navigator([(1300.0, 800.0, 60.0)], loop=False)  # bypasses planner and validator
    d1.on_patrol_loop, d1._then, d1._then_index = False, "patrol", 0
    d1.state.x, d1.state.y = 700.0, 800.0  # just west of the zone, heading east through it
    while not w.finished:
        snap = w.step()
        d = snap.drone("D1")
        assert w.fence.violation(d.x, d.y, d.alt) is None
    kinds = [e["type"] for e in w.log.records if e["actor"] == "D1"]
    assert "SAFETY_HOLD" in kinds or "SAFETY_REPLAN" in kinds
    modes = [(e["data"]["from"], e["data"]["to"]) for e in w.log.of_type("DRONE_MODE") if e["actor"] == "D1"]
    assert ("PATROL", "SAFETY_HOLD") in modes and ("SAFETY_HOLD", "PATROL") in modes
    replans = [e for e in w.log.of_type("ROUTE_PLANNED") if e["actor"] == "D1" and e["data"]["kind"] == "replan"]
    assert replans and len(replans[0]["data"]["waypoints"]) > 2  # the new route goes around the zone


def test_replan_limit_sends_drone_home():
    w = World(cfg("default", duration=60))
    for _ in range(10):
        w.step()
    d1 = w.drones["D1"]
    from src.safety.monitor import SafetyVerdict

    d1._enter_safety_hold(w.t, SafetyVerdict("REPLAN", "test"))
    d1._safety_goal = ((950.0, 800.0, 60.0), "patrol", 0)  # goal inside RZ-1: replanning always fails
    for _ in range(10):
        w.step()
    rejected = [e for e in w.log.of_type("ROUTE_REJECTED") if e["actor"] == "D1" and e["data"]["kind"] == "replan"]
    assert len(rejected) == w.cfg.navigation.replan_limit
    modes = [(e["data"]["from"], e["data"]["to"], e["data"]["reason"]) for e in w.log.of_type("DRONE_MODE") if e["actor"] == "D1"]
    assert ("SAFETY_HOLD", "RETURNING", "replan_limit") in modes


def test_depleted_battery_lands():
    r = raw("default")
    r["scenario_events"] = [{"t": 20, "type": "set_battery", "drone": "D1", "battery_pct": 0.0}]
    r["sim"]["duration"] = 60
    w = World(config_from_dict(r))
    w.run()
    d1 = w.snapshot.drone("D1")
    assert d1.mode == "LANDED" and d1.alt == pytest.approx(0)
    assert [e["data"]["reason"] for e in w.log.of_type("SAFETY_LAND")] == ["battery_depleted"]


# ------------------------------------------------------------------ startup validation


def test_patrol_route_through_zone_is_a_config_error():
    r = raw("default")
    r["drones"][0]["patrol"] = [[200, 300, 60], [1300, 800, 60]]  # crosses RZ-1
    with pytest.raises(ConfigError, match="D1: patrol route is invalid"):
        World(config_from_dict(r))


def test_home_outside_boundary_is_a_config_error():
    r = raw("default")
    r["drones"][0]["home"] = [10, 10, 0]
    with pytest.raises(ConfigError, match="home"):
        World(config_from_dict(r))


def test_patrol_below_altitude_floor_is_a_config_error():
    r = raw("default")
    r["drones"][0]["patrol"][1][2] = 10
    with pytest.raises(ConfigError, match="ALT_BELOW_MIN"):
        World(config_from_dict(r))


# ------------------------------------------------------------------ ground-truth scenario events


def test_hidden_target_produces_no_observations():
    r = raw("default")
    r["targets"][0]["hidden_intervals"] = [[70, 120]]  # D2 passes the target at ~83 s
    w = World(config_from_dict(r))
    w.run()
    assert not [o for o in w.observations if 70 <= o.t < 120]
    assert not records(w)["D2"] or records(w)["D2"][0].created_at >= 120


def test_place_target_event_is_detected_by_sensor():
    r = raw("default")
    r["targets"] = []
    r["scenario_events"] = [{"t": 1, "type": "place_target", "pos": [600, 400]}]  # on D1's patrol leg
    r["sim"]["duration"] = 120
    w = World(config_from_dict(r))
    w.run()
    scen = w.log.of_type("SCENARIO_EVENT")[0]["data"]
    assert scen["applied"] and scen["target"] == "GT-1"
    assert [t.state for t in records(w)["D1"]] == ["CONFIRMED"]


def test_hide_and_show_events():
    r = raw("default")
    r["scenario_events"] = [{"t": 5, "type": "hide_target"}, {"t": 200, "type": "show_target"}]
    w = World(config_from_dict(r))
    w.run()
    assert not [o for o in w.observations if 5 <= o.t < 200]
    assert [e["data"]["targets"] for e in w.log.of_type("SCENARIO_EVENT")] == [["GT-1"], ["GT-1"]]


# ------------------------------------------------------------------ determinism with sensor randomness


def test_same_seed_identical_log_and_different_seed_differs():
    a, b, c = World(cfg("false_observation", duration=120)), World(cfg("false_observation", duration=120)), World(
        cfg("false_observation", duration=120, seed=7)
    )
    for w in (a, b, c):
        w.run()
    assert a.log.digest() == b.log.digest()
    assert a.log.digest() != c.log.digest()


def test_agents_never_receive_ground_truth():
    from src.sim.ground_truth import GroundTruth, GroundTruthTarget
    from src.tracking.observation import Observation

    w = World(cfg("default", duration=100))
    w.run()
    for agent in w.drones.values():
        for name, value in vars(agent).items():
            assert not isinstance(value, (GroundTruth, GroundTruthTarget, World)), name
        assert not any(isinstance(v, Observation) for v in vars(agent.tracker).values())
