"""The web map with several targets at once: each is allocated, inspected and neutralized, and reported per target."""

from src.webmap.runner import run
from tests.unit.test_webmap import two_targets


def test_two_targets_at_once_are_both_neutralized_by_different_drones():
    res = run(two_targets())
    assert len(res["targets"]) == 2
    per = res["summary"]["per_target"]
    assert [p["state"] for p in per] == ["COMPLETED", "COMPLETED"]
    assert all(p["neutralized_at"] is not None for p in per)
    assert per[0]["owner"] != per[1]["owner"]  # the drones shared the work
    assert len(res["summary"]["neutralized"]) == 2
    last = res["frames"][-1]
    assert last["nz"] == [True, True] and len(last["tv"]) == 2
    assert res["summary"]["geofence_violations"] == 0 and res["summary"]["separation_violations"] == 0
    assert all(rnd["winner"] for rnd in res["allocations"]) and len(res["allocations"]) >= 2


def test_single_target_result_has_no_multi_fields():
    from tests.unit.test_webmap import request

    res = run(request())
    assert "tv" not in res["frames"][0] and len(res["targets"]) == 1
    assert res["summary"]["per_target"][0]["state"] == "COMPLETED"


def test_announcement_time_and_the_target_each_bid_round_is_about():
    r = two_targets()
    r["targets"][1]["at"] = 30
    res = run(r)
    assert [t["cue_t"] for t in res["targets"]] == [5.0, 35.0]
    by_target = {}
    for rnd in res["allocations"]:
        by_target.setdefault(rnd["target_index"], []).append(rnd["t"])
    assert set(by_target) == {0, 1}
    assert min(by_target[0]) < 10 and min(by_target[1]) >= 35.0  # target 2's bidding starts only when it is announced
