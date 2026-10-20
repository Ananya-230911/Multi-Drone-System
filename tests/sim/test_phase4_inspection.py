"""Phase 4 acceptance: owner mission (fly, lock, inspect/engage, log), recovery, reallocation (headless)."""

import csv
import dataclasses
from pathlib import Path

import pytest
import yaml

from src.config import config_from_dict, load_config
from src.main import main as cli_main
from src.safety.route_validator import validate_route
from src.sim.world import World

ROOT = Path(__file__).resolve().parents[2]


def raw(name):
    return yaml.safe_load((ROOT / "config" / f"{name}.yaml").read_text())


def run(name=None, r=None, **sim):
    if r is None:
        c = load_config(ROOT / "config" / f"{name}.yaml")
        if sim:
            c = dataclasses.replace(c, sim=dataclasses.replace(c.sim, **sim))
    else:
        c = config_from_dict(r)
    w = World(c)
    w.run()
    return w


def rows(w):
    return {r["target_id"]: r for r in w.target_log_rows()}


@pytest.fixture(scope="module")
def default_w():
    return run("default", duration=150)


# ------------------------------------------------------------------ default.yaml


def test_owner_flies_valid_route_inspects_logs_and_returns(default_w):
    w = default_w
    d2_modes = [e["data"]["to"] for e in w.log.of_type("DRONE_MODE") if e["actor"] == "D2"]
    i = d2_modes.index("ENROUTE")
    assert d2_modes[i:i + 4] == ["ENROUTE", "INSPECTING", "RETURNING", "PATROL"]
    for e in w.log.of_type("ROUTE_PLANNED"):
        if e["actor"] == "D2" and e["data"]["kind"] == "mission":
            assert validate_route([tuple(p) for p in e["data"]["waypoints"]], w.fence).ok
    owner_rows = [e["data"]["row"] for e in w.log.of_type("TARGET_TRANSITION") if e["actor"] == "D2" and e["data"]["row"]]
    assert owner_rows[-4:] == [7, 9, 14, 15]  # ENROUTE, TRACKING, LOGGED, COMPLETED
    assert w.geofence_violations == [] and w.separation_violations == []


def test_target_log_has_one_completed_row(default_w):
    r = rows(default_w)
    assert list(r) == ["T-D2-001"]
    row = r["T-D2-001"]
    assert row["final_state"] == "COMPLETED" and row["owner"] == "D2" and not row["duplicate_inspection"]
    assert row["inspection_duration"] >= default_w.cfg.inspection.duration - 1e-6
    assert row["confirmed_at"] < row["inspection_start"] < row["logged_at"]
    assert row["mean_confidence"] >= default_w.cfg.inspection.min_confidence
    assert abs(row["est_x"] - 1300) < 10 and abs(row["est_y"] - 900) < 10


def test_every_agent_ends_with_completed_view(default_w):
    for d in default_w.snapshot.drones:
        assert {t.id: t.state for t in d.targets}["T-D2-001"] == "COMPLETED", d.id


def test_target_log_csv_written_by_cli(tmp_path):
    args = ["--config", str(ROOT / "config" / "default.yaml"), "--headless", "--duration", "120",
            "--output", str(tmp_path), "--run-id", "x"]  # fmt: skip
    assert cli_main(args) == 0
    with (tmp_path / "x" / "target_log.csv").open() as fh:
        data = list(csv.DictReader(fh))
    assert len(data) == 1 and data[0]["final_state"] == "COMPLETED" and data[0]["owner"] == "D2"
    assert int(data[0]["messages_used"]) > 0


# ------------------------------------------------------------------ tracking_loss.yaml


def test_track_lost_reacquired_and_timer_paused():
    w = run("tracking_loss", duration=140)
    types = [e["type"] for e in w.log.records if e["actor"] == "D2"]
    for t in ("TRACK_LOST", "REACQUIRE_START", "REACQUIRE_SUCCESS", "TARGET_LOGGED"):
        assert t in types
    lost = w.log.of_type("TRACK_LOST")[0]["t"]
    found = w.log.of_type("REACQUIRE_SUCCESS")[0]["t"]
    assert 93 < lost < 101 <= found  # hidden interval [93, 101)
    progress_at_loss = max(e["data"]["seconds"] for e in w.log.of_type("INSPECTION_PROGRESS") if e["t"] <= lost)
    assert 0 < progress_at_loss < w.cfg.inspection.duration  # paused, not reset or finished
    row = rows(w)["T-D2-001"]
    assert row["final_state"] == "COMPLETED"
    assert row["logged_at"] - row["inspection_start"] > w.cfg.inspection.duration + (found - lost) - 1
    modes = [e["data"]["to"] for e in w.log.of_type("DRONE_MODE") if e["actor"] == "D2"]
    assert "REACQUIRING" in modes


def test_reacquire_timeout_cancels_or_reallocates():
    for action, expect in (("cancel", "CANCELLED"), ("reallocate", None)):
        r = raw("tracking_loss")
        r["targets"][0]["hidden_intervals"] = [[93, 400]]  # never comes back
        r["sim"]["duration"] = 160
        r["reacquire"]["timeout_action"] = action
        w = run(r=r)
        assert w.log.of_type("REACQUIRE_TIMEOUT"), action
        if action == "cancel":
            row = rows(w)["T-D2-001"]
            assert row["final_state"] == "CANCELLED" and row["cancel_reason"] == "reacquire_timeout"
        else:
            realloc = [e for e in w.log.of_type("REALLOCATION") if e["data"].get("reason") == "reacquire_timeout"]
            assert realloc and realloc[0]["data"]["released_by"] == "D2"
        assert not w.log.of_type("TARGET_LOGGED")


# ------------------------------------------------------------------ target_in_restricted.yaml


def test_target_in_restricted_zone_cancelled_no_feasible_bidders():
    w = run("target_in_restricted")
    r = rows(w)["T-EXT-001"]
    assert r["final_state"] == "CANCELLED" and r["cancel_reason"] == "no_feasible_bidders"
    reasons = {e["data"]["reason"] for e in w.log.of_type("BID_COMPUTED") if e["target_id"] == "T-EXT-001"}
    assert reasons <= {"no_inspection_point", "mode_RETURNING"}
    assert not w.log.of_type("TARGET_LOGGED") and w.geofence_violations == []


# ------------------------------------------------------------------ external_cue.yaml


@pytest.fixture(scope="module")
def cue_w():
    return run("external_cue")


def test_cue_verified_by_owner_on_site_then_logged(cue_w):
    w = cue_w
    r = rows(w)["T-EXT-001"]
    assert r["final_state"] == "COMPLETED"
    owner = r["owner"]
    confirm = [e for e in w.log.of_type("CONFIRMATION_RESULT") if e["target_id"] == "T-EXT-001" and e["actor"] == owner]
    commit = next(e["t"] for e in w.log.of_type("ALLOCATION_COMMITTED") if e["target_id"] == "T-EXT-001")
    assert confirm and confirm[0]["data"]["result"] == "CONFIRMED" and confirm[0]["t"] > commit
    assert confirm[0]["t"] <= r["logged_at"]


def test_cue_at_empty_location_cancelled_and_never_logged(cue_w):
    w = cue_w
    r = rows(w)["T-EXT-002"]
    assert r["final_state"] == "CANCELLED" and r["cancel_reason"] == "cue_not_confirmed"
    assert not [e for e in w.log.of_type("TARGET_LOGGED") if e["target_id"] == "T-EXT-002"]


# ------------------------------------------------------------------ mission.yaml (engage, neutralize)


@pytest.fixture(scope="module")
def mission_w():
    return run("mission")


def test_mission_parameters_decide_the_drone(mission_w):
    bids = {e["actor"]: e["data"] for e in mission_w.log.of_type("BID_COMPUTED") if e["epoch"] == 0}
    assert bids["D1"]["feasible"] is False and bids["D1"]["reason"] == "insufficient_energy"
    assert bids["D2"]["breakdown"]["descent"] == pytest.approx(50.0)  # 110 m -> 60 m
    assert bids["D4"]["breakdown"]["standoff"] < bids["D3"]["breakdown"]["standoff"]  # smaller sensor flies closer
    winner = {e["data"]["winner"] for e in mission_w.log.of_type("ALLOCATION_COMMITTED")}
    assert winner == {"D2"} and mission_w.baseline_results()[0]["match"]


def test_mission_target_neutralized_and_logged(mission_w):
    w = mission_w
    [n] = w.log.of_type("TARGET_NEUTRALIZED")
    assert n["data"]["ground_truth"] == "GT-1" and n["data"]["by"] == "D2"
    logged = w.log.of_type("TARGET_LOGGED")[0]
    assert logged["t"] == n["t"]
    assert not [o for o in w.observations if o.t > n["t"]]  # nothing left to observe
    assert rows(w)[logged["target_id"]]["final_state"] == "COMPLETED"
    assert w.geofence_violations == [] and w.separation_violations == []


@pytest.mark.parametrize("event", [
    {"t": 30, "type": "fail_drone", "drone": "D2"},
    {"t": 30, "type": "set_battery", "drone": "D2", "battery_pct": 0.21},
])  # fmt: skip
def test_owner_lost_mid_mission_others_swap_in(event):
    r = raw("mission")
    r["sim"]["duration"] = 200
    r["scenario_events"] = [event]
    w = run(r=r)
    realloc = w.log.of_type("REALLOCATION")
    assert realloc and realloc[0]["epoch"] == 1
    new_owner = {e["data"]["winner"] for e in w.log.of_type("ALLOCATION_COMMITTED") if e["epoch"] == 1}
    assert len(new_owner) == 1 and "D2" not in new_owner  # previous owner excluded in epoch 1
    excluded = [e for e in w.log.of_type("BID_COMPUTED") if e["epoch"] == 1 and e["actor"] == "D2"]
    assert all(not e["data"]["feasible"] for e in excluded)
    [row] = w.target_log_rows()
    assert row["final_state"] == "COMPLETED" and row["owner"] in new_owner
    assert row["epoch"] == 1 and row["reallocations"] == 1
    assert w.log.of_type("TARGET_NEUTRALIZED")[0]["data"]["by"] in new_owner


def test_max_reallocations_cancels_task():
    r = raw("mission")
    r["sim"]["duration"] = 200
    r["allocation"] = {"max_reallocations": 0}
    r["scenario_events"] = [{"t": 30, "type": "fail_drone", "drone": "D2"}]
    w = run(r=r)
    [row] = w.target_log_rows()
    assert row["final_state"] == "CANCELLED" and row["cancel_reason"] == "max_reallocations"
    assert not w.log.of_type("TARGET_LOGGED")


# ------------------------------------------------------------------ logging invariant


@pytest.mark.parametrize("name", ["default", "tracking_loss", "external_cue", "low_battery", "limited_comms",
                                  "duplicate_reports", "geofence_detour", "mission"])  # fmt: skip
def test_never_logged_without_confirmation_and_full_inspection(name):
    w = run(name, duration=200)
    for e in w.log.of_type("TARGET_LOGGED"):
        assert e["data"]["inspection_duration"] >= w.cfg.inspection.duration - 1e-6
        assert e["data"]["confirmed_at"] is not None and e["data"]["confirmed_at"] <= e["t"]
    for row in w.target_log_rows():
        if row["final_state"] == "COMPLETED":
            assert row["logged_at"] is not None and row["inspection_duration"] >= w.cfg.inspection.duration - 1e-6
        else:
            assert row["logged_at"] is None
    assert w.geofence_violations == []
