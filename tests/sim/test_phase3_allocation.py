"""Phase 3 acceptance: communication, decentralized auction, consensus, ownership (headless)."""

import dataclasses
import math
from pathlib import Path

import pytest
import yaml

from src.config import config_from_dict, load_config
from src.sim.world import World

ROOT = Path(__file__).resolve().parents[2]


def cfg(name, **sim):
    c = load_config(ROOT / "config" / f"{name}.yaml")
    return dataclasses.replace(c, sim=dataclasses.replace(c.sim, **sim)) if sim else c


def raw(name):
    return yaml.safe_load((ROOT / "config" / f"{name}.yaml").read_text())


def commits(w):
    return {e["actor"]: e["data"]["winner"] for e in w.log.of_type("ALLOCATION_COMMITTED")}


def views(w):
    return {d.id: {t.id: t for t in d.targets} for d in w.snapshot.drones}


@pytest.fixture(scope="module")
def worlds():
    out = {}
    for name, dur in [("default", 120), ("low_battery", 40), ("limited_comms", 60), ("duplicate_reports", 40),
                      ("external_cue", 40)]:  # fmt: skip
        w = World(cfg(name, duration=dur))
        w.run()
        out[name] = w
    return out


# ------------------------------------------------------------------ default.yaml


def test_default_every_live_agent_commits_same_owner_matching_baseline(worlds):
    w = worlds["default"]
    c = commits(w)
    assert set(c) == {"D1", "D2", "D3", "D4"} and set(c.values()) == {"D2"}
    [res] = w.baseline_results()
    assert res["baseline_winner"] == "D2" and res["match"] and res["agree"]


def test_default_lifecycle_reaches_allocated_on_every_agent(worlds):
    w = worlds["default"]
    allocated = {e["actor"] for e in w.log.of_type("TARGET_TRANSITION")
                 if e["target_id"] == "T-D2-001" and e["data"]["to"] in ("ALLOCATED", "ENROUTE")}  # fmt: skip
    assert allocated == {"D1", "D2", "D3", "D4"}
    for d, recs in views(w).items():  # and then the owner's mission completes it (Phase 4)
        t = recs["T-D2-001"]
        assert t.state == "COMPLETED" and t.owner == "D2" and t.confirmation == "CONFIRMED", d


def test_default_claim_from_owner_and_ack_from_others(worlds):
    sent = [(e["actor"], e["data"]["msg_type"]) for e in worlds["default"].log.of_type("MSG_SENT")
            if e["data"]["msg_type"] in ("CLAIM", "ACK")]  # fmt: skip
    assert ("D2", "CLAIM") in sent
    assert {a for a, t in sent if t == "ACK"} == {"D1", "D3", "D4"}
    assert not [a for a, t in sent if t == "CLAIM" and a != "D2"]


def test_default_ineligible_drone_reports_why(worlds):
    bids = {e["actor"]: e["data"] for e in worlds["default"].log.of_type("BID_COMPUTED")}
    assert bids["D3"]["feasible"] is False and bids["D3"]["reason"] == "mode_RETURNING"


def test_only_the_owner_flies_the_mission(worlds):
    w = worlds["default"]
    flyers = {e["actor"] for e in w.log.of_type("DRONE_MODE") if e["data"]["to"] in ("ENROUTE", "INSPECTING")}
    assert flyers == {"D2"}
    assert w.snapshot.drone("D2").current_task is None  # task finished, back on patrol


def test_detector_is_not_automatically_owner_everyone_bids(worlds):
    bidders = {e["actor"] for e in worlds["default"].log.of_type("BID_COMPUTED")}
    assert bidders == {"D1", "D2", "D3", "D4"}


# ------------------------------------------------------------------ low_battery.yaml


def test_low_battery_nearest_drone_infeasible_and_another_wins(worlds):
    w = worlds["low_battery"]
    bids = {e["actor"]: e for e in w.log.of_type("BID_COMPUTED")}
    cue = tuple(bids["D1"]["data"]["target_est"])
    start = w.allocation_starts[(bids["D1"]["target_id"], 0)]
    dist = {v.id: math.dist((v.x, v.y), cue) for v in start["drones"]}
    assert min(dist, key=dist.get) == "D1"  # nearest
    assert bids["D1"]["data"]["feasible"] is False and bids["D1"]["data"]["reason"] == "insufficient_energy"
    c = commits(w)
    assert len(c) == 4 and set(c.values()) == {"D2"}
    assert w.baseline_results()[0]["match"]


# ------------------------------------------------------------------ limited_comms.yaml


def test_limited_comms_agreement_through_relay(worlds):
    w = worlds["limited_comms"]
    c = commits(w)
    assert set(c) == {"D1", "D2", "D3", "D4"} and set(c.values()) == {"D1"}
    direct = [e for e in w.log.of_type("MSG_DELIVERED") if e["data"]["from"] == "D1" and e["actor"] == "D4"]
    assert direct == []  # D4 never heard D1 directly
    learned = next(e for e in w.log.of_type("TARGET_CREATED") if e["actor"] == "D4")
    assert learned["data"]["relayed_from"] == "D3"
    assert w.baseline_results()[0]["match"]


# ------------------------------------------------------------------ duplicate_reports.yaml


def test_duplicate_reports_converge_on_one_target_id(worlds):
    w = worlds["duplicate_reports"]
    own = sorted({e["target_id"] for e in w.log.of_type("TARGET_CREATED") if "relayed_from" not in e["data"]})
    assert own == ["T-D1-001", "T-D2-001", "T-D3-001"]  # created concurrently by three drones
    assert w.log.of_type("TARGET_MERGED")
    for d, recs in views(w).items():
        active = [t for t in recs.values() if t.state != "CANCELLED"]
        assert [t.id for t in active] == ["T-D1-001"], d
    c = commits(w)
    assert len(c) == 4 and len(set(c.values())) == 1


# ------------------------------------------------------------------ external_cue.yaml


def test_external_cue_allocated_while_unconfirmed(worlds):
    w = worlds["external_cue"]  # 40 s run: the owner is still flying to the cue
    trans = [e for e in w.log.of_type("TARGET_TRANSITION") if e["target_id"] == "T-EXT-001"]
    rows = {e["data"]["row"] for e in trans if e["data"]["to"] == "ALLOCATED" and e["data"]["row"] is not None}
    assert rows == {4}  # each agent's own commit: DETECTED -> ALLOCATED (verify-and-inspect task)
    commits = [e["t"] for e in w.log.of_type("ALLOCATION_COMMITTED") if e["target_id"] == "T-EXT-001"]
    assert len(commits) == 4
    confirmed = [e["t"] for e in w.log.of_type("CONFIRMATION_RESULT") if e["target_id"] == "T-EXT-001"]
    assert not confirmed or min(confirmed) > max(commits)  # coordinates alone never confirmed it
    assert w.baseline_results()[0]["match"]


# ------------------------------------------------------------------ heartbeats, failures, separation


def test_heartbeats_fill_peer_tables(worlds):
    w = worlds["default"]
    snap = w.snapshot
    d1 = snap.drone("D1")
    assert set(d1.alive_peers) >= {"D2", "D3"}
    assert len(w.log.of_type("MSG_SENT")) > 0


def test_failed_drone_drops_out_of_alive_peers_and_auction_does_not_wait():
    r = raw("default")
    r["sim"]["duration"] = 110
    r["scenario_events"] = [{"t": 30, "type": "fail_drone", "drone": "D4"}]
    w = World(config_from_dict(r))
    w.run()
    assert "D4" not in w.snapshot.drone("D1").alive_peers
    c = commits(w)
    assert "D4" not in c and set(c.values()) == {"D2"}
    committed = [e for e in w.log.of_type("ALLOCATION_COMMITTED")]
    assert all(e["data"]["how"] == "consensus" for e in committed)  # no timeout needed


def test_separation_yield_rule_applied_and_true_separation_measured():
    """Two drones on crossing paths at the same altitude and time: the higher ID (no task) yields."""
    r = raw("default")
    r["sim"]["duration"] = 40
    r["targets"] = []
    r["drones"] = r["drones"][:2]
    r["drones"][0].update(pos=[1000, 1300], patrol=[[1000, 1300, 60], [1600, 1300, 60], [1600, 1250, 60], [1000, 1250, 60]])
    r["drones"][1].update(pos=[1300, 1600], patrol=[[1300, 1600, 60], [1300, 1100, 60], [1350, 1100, 60], [1350, 1600, 60]])
    w = World(config_from_dict(r))
    w.run()
    yields = [(e["actor"], e["data"]["to"]) for e in w.log.of_type("SEPARATION_YIELD")]
    assert yields and yields[0] == ("D2", "D1")
    assert w.separation_violations == []  # measured from true positions


def test_same_seed_identical_log_with_network_randomness():
    a, b = World(cfg("limited_comms", duration=30)), World(cfg("limited_comms", duration=30))
    a.run()
    b.run()
    assert a.log.digest() == b.log.digest()
