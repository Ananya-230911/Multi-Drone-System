"""Phase 5: owner failure, network partition and heal, and the batch evaluation."""

import dataclasses

import pytest

from src import evaluate
from src.config import load_config
from src.sim.world import World


def run(name, seed=42, heal=None):
    cfg = load_config(f"config/{name}.yaml")
    if heal is not None:
        net = cfg.network
        net = dataclasses.replace(net, partitions=[dataclasses.replace(net.partitions[0], end=heal)])
        cfg = dataclasses.replace(cfg, network=net)
    cfg = dataclasses.replace(cfg, sim=dataclasses.replace(cfg.sim, seed=seed))
    w = World(cfg)
    w.run()
    return w


def types(w, t, **data):
    return [e for e in w.log.of_type(t) if all(e["data"].get(k) == v for k, v in data.items())]


@pytest.mark.parametrize("seed", [1, 2, 3, 4])
def test_owner_failure_is_noticed_and_task_is_reallocated(seed):
    w = run("drone_failure", seed)
    first = [e for e in types(w, "ALLOCATION_COMMITTED") if e["epoch"] == 0 and e["actor"] == e["data"]["winner"]]
    assert first and first[0]["data"]["winner"] == "D1"  # D1 is the best bidder, so it owns the task first
    failed = w.log.of_type("DRONE_FAILED")[0]["t"]
    assert types(w, "REALLOCATION", reason="owner_timeout", previous_owner="D1")
    second = [e for e in types(w, "ALLOCATION_COMMITTED") if e["epoch"] == 1]
    assert second and min(e["t"] for e in second) - failed < 20.0  # heartbeat timeout + one auction
    rows = w.target_log_rows()
    assert [r["final_state"] for r in rows] == ["COMPLETED"] and rows[0]["owner"] != "D1"
    assert rows[0]["reallocations"] == 1 and not rows[0]["duplicate_inspection"]
    assert not w.geofence_violations and not w.separation_violations and w.log.of_type("TARGET_NEUTRALIZED")


@pytest.mark.parametrize("heal", [12, 20, 30])
def test_partition_elects_two_owners_then_one_releases(heal):
    w = run("partition", heal=heal)
    winners = {e["data"]["winner"] for e in types(w, "ALLOCATION_COMMITTED") if e["t"] < heal}
    assert len(winners) == 2  # each side of the split elected its own owner
    assert w.log.of_type("DUPLICATE_OWNER_SEEN")
    lost = types(w, "CLAIM_CONFLICT", result="lost")
    assert lost and all(e["t"] >= heal for e in lost)  # nothing is resolved while the radio is cut
    rows = w.target_log_rows()
    assert len(rows) == 1 and rows[0]["final_state"] == "COMPLETED" and not rows[0]["duplicate_inspection"]
    assert rows[0]["owner"] not in {e["actor"] for e in lost}
    assert not w.geofence_violations and not w.separation_violations


def test_partition_late_heal_still_logs_the_target_once():
    w = run("partition", heal=40)  # both owners are already at the target when the radio comes back
    rows = w.target_log_rows()
    assert len(rows) == 1 and rows[0]["final_state"] == "COMPLETED" and len(w.log.of_type("TARGET_LOGGED")) == 1
    assert not w.geofence_violations


def test_peers_never_merge_records_in_opposite_directions():
    """Regression: a peer's summary naming a smaller ID must rename our record, not alias the smaller ID to ours."""
    w = run("false_observation", seed=3)  # used to leave T-D3-031 <-> T-D3-032 aliased to each other
    for a in w.drones.values():
        for k, v in a.tracker.alias_of.items():
            assert k > v, f"{a.id}: {k} -> {v} (the smallest ID must survive)"
    assert w.target_log_rows() is not None  # no endless alias loop


def test_evaluation_checks_and_summary(tmp_path):
    runs = evaluate.evaluate(["drone_failure", "partition", "default"], seeds=2)
    assert all(r["ok"] for r in runs), [r["problems"] for r in runs if not r["ok"]]
    s = {r["scenario"]: r for r in evaluate.summarize(runs)}
    assert s["drone_failure"]["t_realloc_after_failure"] is not None and s["default"]["baseline_agreement_pct"] == 100.0
    assert s["partition"]["geofence_violations"] == 0
    paths = evaluate.plots(list(s.values()), runs, tmp_path)
    assert len(paths) == 4 and all(p.stat().st_size > 5000 for p in paths)
    assert "| default |" in evaluate.markdown(list(s.values()))


def test_expectations_catch_a_broken_run():
    bad = {"geofence_violations": 0, "separation_violations": 0, "completed": 0, "duplicate_inspections": 1,
           "owner_timeouts": 0, "owner": None, "t_realloc_after_failure": None, "duplicate_owner_seen": 0,
           "claim_conflicts_lost": 0}  # fmt: skip
    assert len(evaluate.check("drone_failure", bad)) == 3
    assert any("geofence" in p for p in evaluate.check("default", {**bad, "geofence_violations": 2}))
