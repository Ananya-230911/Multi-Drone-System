"""Phase 1 acceptance: patrol, battery, failure, return-on-reserve, determinism (headless)."""

import copy
import math
from pathlib import Path

import pytest
import yaml

from src.config import config_from_dict, load_config
from src.drone.agent import Drone
from src.logging_.event_log import EVENT_TYPES
from src.main import main as cli_main
from src.sim.world import World, make_rng

ROOT = Path(__file__).resolve().parents[2]
DEFAULT = ROOT / "config" / "default.yaml"


@pytest.fixture
def raw():
    return yaml.safe_load(DEFAULT.read_text())


def run(cfg, until=None):
    w = World(cfg)
    history = []
    while not w.finished and (until is None or w.t < until):
        history.append(w.step())
    return w, history


def test_all_drones_patrol_and_reach_waypoints():
    w, hist = run(load_config(DEFAULT), until=100)
    for d in hist[0].drones:
        path = [s.drone(d.id) for s in hist]
        travelled = sum(math.dist((a.x, a.y), (b.x, b.y)) for a, b in zip(path, path[1:]))
        assert travelled > 500, d.id  # 100 s at up to 15 m/s
    assert all(d.mode == "PATROL" for d in hist[200].drones)
    reached = {r["actor"] for r in w.log.of_type("WAYPOINT_REACHED") if r["data"]["index"] != 0}
    assert reached == {"D1", "D2", "D3", "D4"}


def test_kinematic_limits_respected_every_tick():
    cfg = load_config(DEFAULT)
    w, hist = run(cfg)
    for snap in hist:
        for d in snap.drones:
            assert d.speed <= cfg.drone_defaults.max_speed + 1e-6
            assert abs(d.vz) <= cfg.drone_defaults.climb_rate + 1e-6


def test_battery_decreases_while_flying():
    w, hist = run(load_config(DEFAULT), until=60)
    for i in range(1, len(hist)):
        for prev, cur in zip(hist[i - 1].drones, hist[i].drones):
            assert cur.battery_pct <= prev.battery_pct
            if cur.speed > 0:
                assert cur.battery_pct < prev.battery_pct
    first, last = hist[0].drones, hist[-1].drones
    assert all(b.battery_pct < a.battery_pct for a, b in zip(first, last))


def test_climbing_patrol_uses_more_energy_than_level_patrol(raw):
    flat = copy.deepcopy(raw)
    hilly = copy.deepcopy(raw)
    # Same horizontal square for D1; the hilly version alternates between 30 m and 90 m.
    hilly["drones"][0]["patrol"] = [[200, 300, 60], [600, 300, 90], [600, 500, 30], [200, 500, 90]]
    used = {}
    for name, r in (("flat", flat), ("hilly", hilly)):
        w, _ = run(config_from_dict(r), until=120)
        d1 = w.snapshot.drone("D1")
        used[name] = r["drones"][0]["battery_pct"] - d1.battery_pct
    assert used["hilly"] > used["flat"]


def test_failed_drone_freezes(raw):
    raw["scenario_events"] = [{"t": 50, "type": "fail_drone", "drone": "D2"}]
    w, hist = run(config_from_dict(raw))
    frozen = [s.drone("D2") for s in hist if s.t >= 50]
    assert frozen[0].mode == "FAILED"
    first = frozen[0]
    for d in frozen:
        assert (d.x, d.y, d.alt, d.battery_wh) == (first.x, first.y, first.alt, first.battery_wh)
        assert d.speed == 0
    scen = w.log.of_type("SCENARIO_EVENT")
    assert scen and scen[0]["data"]["applied"] and scen[0]["t"] == 50
    assert [r["t"] for r in w.log.of_type("DRONE_FAILED")] == [50]
    assert not [r for r in w.log.records if r["actor"] == "D2" and r["t"] > 50]
    # other drones keep patrolling
    assert w.snapshot.drone("D1").mode == "PATROL"


def test_operator_fail_goes_through_scenario():
    w = World(load_config(DEFAULT))
    for _ in range(100):
        w.step()
    w.scenario.inject(w.t, "fail_drone", {"drone": "D4"})
    w.step()
    assert w.snapshot.drone("D4").mode == "FAILED"
    ev = w.log.of_type("SCENARIO_EVENT")[-1]
    assert ev["data"]["source"] == "operator" and ev["data"]["drone"] == "D4"


def test_low_battery_drone_returns_home_and_never_drops_below_reserve():
    cfg = load_config(DEFAULT)
    w, hist = run(cfg)
    returns = [r for r in w.log.of_type("DRONE_MODE") if r["data"]["to"] == "RETURNING" and r["data"]["reason"] == "battery_reserve"]
    assert [r["actor"] for r in returns] == ["D3"]  # D3 starts at 30%
    d3 = w.snapshot.drone("D3")
    assert (d3.x, d3.y, d3.alt) == pytest.approx(cfg.drones[2].home)
    assert d3.docked
    reserve = cfg.drone_defaults.reserve_pct
    assert min(s.drone("D3").battery_pct for s in hist) >= reserve


def test_set_battery_triggers_return_and_recharge_resumes_patrol(raw):
    raw["scenario_events"] = [
        {"t": 30, "type": "set_battery", "drone": "D1", "battery_pct": 0.21},
        {"t": 150, "type": "set_battery", "drone": "D1", "battery_pct": 0.9},
    ]
    w, hist = run(config_from_dict(raw))
    modes = [(r["t"], r["data"]["to"]) for r in w.log.of_type("DRONE_MODE") if r["actor"] == "D1"]
    assert modes[0][1] == "RETURNING" and 30 <= modes[0][0] <= 30.2
    assert modes[1] == (150.0, "PATROL")
    assert w.snapshot.drone("D1").mode == "PATROL" and not w.snapshot.drone("D1").docked


def test_same_seed_gives_identical_event_log():
    a, _ = run(load_config(DEFAULT))
    b, _ = run(load_config(DEFAULT))
    assert a.log.text() == b.log.text()
    assert a.log.digest() == b.log.digest()
    assert len(a.log.records) > 50


def test_event_log_is_time_ordered_and_typed():
    w, _ = run(load_config(DEFAULT))
    ts = [r["t"] for r in w.log.records]
    assert ts == sorted(ts)
    assert {r["type"] for r in w.log.records} <= EVENT_TYPES


def test_cli_headless_writes_identical_logs(tmp_path):
    args = ["--config", str(DEFAULT), "--headless", "--duration", "60", "--output", str(tmp_path)]
    assert cli_main(args + ["--run-id", "a"]) == 0
    assert cli_main(args + ["--run-id", "b"]) == 0
    a = (tmp_path / "a" / "events.jsonl").read_bytes()
    b = (tmp_path / "b" / "events.jsonl").read_bytes()
    assert a and a == b
    assert (tmp_path / "a" / "summary.json").is_file()


def test_cli_reports_config_errors(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("sim: {dt: -1}\ndrones: []\n")
    assert cli_main(["--config", str(bad), "--headless"]) == 2


def test_rng_streams_are_seeded_per_component():
    assert make_rng(42, "sensor:D1").random() == make_rng(42, "sensor:D1").random()
    assert make_rng(42, "sensor:D1").random() != make_rng(42, "sensor:D2").random()
    assert make_rng(42, "sensor:D1").random() != make_rng(43, "sensor:D1").random()


def test_drones_hold_no_references_to_world_or_other_drones():
    w = World(load_config(DEFAULT))
    for agent in w.drones.values():
        for name, value in vars(agent).items():
            assert not isinstance(value, (World, Drone)), name
            if isinstance(value, (list, tuple, dict)):
                items = value.values() if isinstance(value, dict) else value
                assert not any(isinstance(v, (World, Drone)) for v in items), name
