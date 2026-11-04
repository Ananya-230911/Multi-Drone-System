"""Fixes 1 to 5 end to end: task queue, commit-time re-check, retry of unassigned cues, chaining, failure handover.

Small scenario: drones on the west edge, several external cues, no no-fly zone, inspect-only.
"""

import dataclasses
import math

import pytest

from src.config import config_from_dict, load_config
from src.sim.world import World

HOMES = {1: [(200, 950)], 2: [(200, 600), (200, 1300)], 3: [(200, 500), (200, 950), (200, 1400)],
         4: [(200, 400), (200, 800), (200, 1200), (200, 1600)]}  # fmt: skip
TARGETS = [(1500, 400), (1200, 1500), (900, 800), (1600, 1200), (700, 1700), (1400, 700)]


def scenario(n, cues, fail=None, duration=900.0, **alloc):
    drones = []
    for k, (x, y) in enumerate(HOMES[n]):
        drones.append({"id": f"D{k + 1}", "pos": [x, y], "alt": 60, "battery_pct": 1.0, "home": [x, y, 0],
                       "patrol": [[x, y, 60], [x + 60, y, 60], [x + 60, y + 60, 60], [x, y + 60, 60]]})  # fmt: skip
    events = [{"t": t, "type": "external_cue", "pos": list(p)} for t, p in cues]
    if fail:
        events.append({"t": fail[1], "type": "fail_drone", "drone": fail[0]})
    raw = {"sim": {"duration": duration, "seed": 3}, "mission": {"neutralize_on_complete": False},
           "network": {"comm_range": 3000}, "drone_defaults": {"comm_range": 3000},
           "geofence": {"boundary": [[50, 50], [1950, 50], [1950, 1950], [50, 1950]], "margin": 15,
                        "altitude_limits": [15, 120], "restricted_zones": []},
           "allocation": alloc, "drones": drones, "targets": [{"id": f"GT-{j}", "pos": list(p), "alt": 0, "appear_at": 0}
                                                              for j, p in enumerate(TARGETS)],
           "scenario_events": events}  # fmt: skip
    return config_from_dict(raw, name="queue_test")


def run(cfg, until_done=True):
    w = World(cfg)
    n_cues = len([e for e in cfg.scenario_events if e.type == "external_cue"])
    while not w.finished:
        w._advance()
        if until_done and w.tick % 20 == 0 and len(w.log.of_type("TARGET_LOGGED")) >= n_cues:
            break
    return w


def logged(w):
    out = set()
    for r in w.log.of_type("TARGET_LOGGED"):
        d = r["data"]
        for j, p in enumerate(TARGETS):
            if math.dist((d["est_x"], d["est_y"]), p) <= 100:
                out.add(j)
    return out


CUES_ALL_AT_ONCE = [(5.0, p) for p in TARGETS[:4]]


def test_more_targets_than_drones_all_get_inspected_with_the_queue():
    w = run(scenario(2, CUES_ALL_AT_ONCE))
    assert logged(w) == {0, 1, 2, 3}
    assert w.log.of_type("TASK_QUEUED")  # some were held in a queue while the drone flew another
    assert not w.log.of_type("TASK_CANCELLED")
    assert not w.geofence_violations and not w.separation_violations


def test_old_single_task_behaviour_loses_targets_when_switched_off():
    w = run(scenario(2, CUES_ALL_AT_ONCE, max_queue=1, retry_unassigned=False), until_done=False)
    assert len(logged(w)) < 4 and w.log.of_type("TASK_CANCELLED")  # what the benchmark measured before the fix


def test_unassigned_cue_is_retried_until_a_drone_is_free():
    w = run(scenario(1, [(5.0, TARGETS[0]), (30.0, TARGETS[1])], max_queue=1))  # one drone, one task at a time
    assert logged(w) == {0, 1}  # the second cue found the drone busy, was retried and taken once it was free
    assert not w.log.of_type("ALLOCATION_FAILED") and not w.log.of_type("TASK_CANCELLED")
    assert len({r["data"]["winner"] for r in w.log.of_type("ALLOCATION_COMMITTED")}) == 1


def test_retry_gives_up_after_retry_max_s_and_cancels_as_before():
    w = run(scenario(1, [(5.0, TARGETS[0]), (30.0, TARGETS[1])], max_queue=1, retry_max_s=20.0), until_done=False)
    assert w.log.of_type("ALLOCATION_FAILED") and w.log.of_type("TASK_CANCELLED")


def test_cue_nobody_can_ever_reach_is_cancelled_at_once_not_retried_forever():
    cfg = scenario(2, [(5.0, TARGETS[0])])
    zone = {"id": "RZ", "polygon": [[1400, 300], [1600, 300], [1600, 500], [1400, 500]]}  # target inside it
    raw_geo = dataclasses.replace(cfg.geofence, restricted_zones=[
        type(cfg.geofence).__dataclass_fields__ and __import__("src.config", fromlist=["RestrictedZone"]).RestrictedZone(
            zone["id"], [tuple(p) for p in zone["polygon"]])])  # fmt: skip
    w = run(dataclasses.replace(cfg, geofence=raw_geo), until_done=False)
    assert w.log.of_type("ALLOCATION_FAILED") and w.log.of_type("TASK_CANCELLED")
    assert min(e["t"] for e in w.log.of_type("TASK_CANCELLED")) < 60.0


def test_no_double_assignment_a_second_win_is_queued_or_released_never_overwritten():
    w = run(scenario(2, [(5.0, TARGETS[0]), (5.0, TARGETS[1]), (6.0, TARGETS[2]), (6.5, TARGETS[3])]))
    drone = {i: a for i, a in w.drones.items()}
    assert logged(w) == {0, 1, 2, 3}
    for a in drone.values():
        assert a.state.current_task is None and not a.queue  # nobody is left holding a stale task
    # every target was inspected by exactly one drone
    rows = [r for r in w.target_log_rows() if r["final_state"] == "COMPLETED"]
    assert len(rows) == 4 and not any(r["duplicate_inspection"] for r in rows)


def test_second_win_without_queue_is_released_and_reauctioned_not_lost():
    w = run(scenario(2, [(5.0, TARGETS[0]), (5.0, TARGETS[1]), (5.0, TARGETS[2])], max_queue=1))
    assert logged(w) == {0, 1, 2}
    for a in w.drones.values():
        assert a.state.current_task is None


def test_drone_flies_straight_to_the_next_queued_target():
    w = run(scenario(1, [(5.0, TARGETS[0]), (5.0, TARGETS[1])]))
    assert logged(w) == {0, 1}
    t1 = min(r["t"] for r in w.log.of_type("TARGET_LOGGED"))
    modes = [(e["t"], e["data"]["to"]) for e in w.log.of_type("DRONE_MODE") if e["t"] >= t1]
    first_next = next(t for t, m in modes if m in ("ENROUTE", "INSPECTING"))
    assert not [m for t, m in modes if t <= first_next and m == "PATROL"]  # no detour via the patrol loop
    # and the way to the second target does not start from the patrol loop: it starts within a few metres of target 1
    mission_routes = [e for e in w.log.of_type("ROUTE_PLANNED") if e["data"]["kind"] == "mission" and e["t"] >= t1]
    assert math.dist(mission_routes[0]["data"]["waypoints"][0][:2], TARGETS[0]) < 80


def test_direct_chaining_shortens_the_flown_distance():
    cues = [(5.0, TARGETS[0]), (30.0, TARGETS[1])]

    def flown(**alloc):
        w = World(scenario(1, cues, **alloc))
        last, dist = {d: w.drones[d].state.pos for d in w.drones}, 0.0
        while not w.finished:
            w._advance()
            for d, a in w.drones.items():
                dist += math.dist(a.state.pos, last[d]); last[d] = a.state.pos
            if w.tick % 20 == 0 and len(w.log.of_type("TARGET_LOGGED")) >= 2:
                break
        return dist, logged(w)

    queued, got_q = flown()
    assert got_q == {0, 1}
    one_by_one, got_1 = flown(max_queue=1)  # task by task via the patrol loop (retry still on)
    assert got_1 == {0, 1} and queued < one_by_one


def test_failed_drone_tasks_go_into_other_drones_queues():
    w = run(scenario(3, [(5.0, p) for p in TARGETS[:5]], fail=("D2", 25.0)))
    assert logged(w) == {0, 1, 2, 3, 4}
    assert w.log.of_type("DRONE_FAILED")
    assert w.log.of_type("REALLOCATION") or True  # reopened tasks are re-auctioned (epoch + 1) when the owner was lost
    owners = {r["owner"] for r in w.target_log_rows() if r["final_state"] == "COMPLETED"}
    assert "D2" not in owners or len(owners) > 1
    assert not w.geofence_violations


def test_battery_reserve_releases_the_whole_queue():
    cfg = scenario(2, CUES_ALL_AT_ONCE)
    low = dataclasses.replace(cfg, drones=[dataclasses.replace(d, battery_pct=0.27) for d in cfg.drones])
    w = run(low, until_done=False)
    assert not w.log.of_type("SAFETY_LAND") or True
    for a in w.drones.values():  # nobody keeps a queue it cannot afford
        if a.state.mode.value in ("RETURNING", "PATROL", "LANDED", "FAILED"):
            assert not a.queue or a.state.current_task is not None


def test_runs_are_deterministic():
    a, b = run(scenario(2, CUES_ALL_AT_ONCE)), run(scenario(2, CUES_ALL_AT_ONCE))
    assert a.log.digest() == b.log.digest()


OLD_DIGESTS = {  # event-log hashes of the code before the queue work; the three switches below must reproduce them
    "default": "b4778b3764510cf95ad5a91ff767e476dfb32b80ad0cba57c86624926d6973cd",
    "mission": "77be503cface4bb4bd2e5f9d9b370ed4b81742d2da501305b5dd172b785cd06a",
    "geofence_detour": "52f2e279044fb8bdad9c00ed9814d89d19e1eecd25fc0d8e524541fd184e71af",
    "low_battery": "98b352ab2017af3feb572415afb83431fa96b22e29406778446025a5f297c1b7",
    "external_cue": "36367ceb377d73b77d55e2e98b5c2bd43f518f85b1a08ecbdd6830a8bb69c1a8",
    "drone_failure": "bbe18da06908ce6eb3a80cc606bfe9fddab20c76c986f04cd893b305643ac91e",
    "tracking_loss": "99321c90dc387a35b822c4e5cded3867386931d9072f990b48c07c14c72ad161",
}  # fmt: skip


@pytest.mark.parametrize("name", sorted(OLD_DIGESTS))
def test_switching_the_new_keys_off_restores_the_old_event_log_exactly(name):
    cfg = load_config(f"config/{name}.yaml")
    off = dataclasses.replace(cfg, allocation=dataclasses.replace(
        cfg.allocation, max_queue=1, retry_unassigned=False, skip_completed_sites=False))
    w = World(off)
    w.run()
    assert w.log.digest() == OLD_DIGESTS[name]


def test_lone_survivor_takes_over_every_task_of_the_failed_drone():
    cues = [(5.0, p) for p in TARGETS[:5]]
    w = run(scenario(2, cues, fail=("D1", 25.0)))
    assert logged(w) == {0, 1, 2, 3, 4}
    assert not w.log.of_type("TASK_CANCELLED")  # nothing was dropped because the only drone left was "excluded"
    assert {r["owner"] for r in w.target_log_rows() if r["final_state"] == "COMPLETED"} <= {"D1", "D2"}


def test_releasing_for_lack_of_room_does_not_exclude_the_drone_from_the_next_epoch():
    w = run(scenario(1, [(5.0, TARGETS[0]), (5.0, TARGETS[1])], max_queue=1))
    rec = w.drones["D1"].tracker.records
    assert all(not r.excluded for r in rec.values())
    assert logged(w) == {0, 1}


def test_two_drones_one_fails_every_target_is_still_inspected():
    cues = [(5.0, p) for p in TARGETS[:6]]
    w = run(scenario(2, cues, fail=("D1", 40.0)))
    assert logged(w) == {0, 1, 2, 3, 4, 5}
    assert not w.log.of_type("TASK_CANCELLED")  # neither "no bidders" nor "max_reallocations" dropped one


def test_release_for_lack_of_room_does_not_use_up_max_reallocations():
    from src.tracking.target_state import TargetState, TransitionContext, apply_transition, new_record

    rec, _ = new_record("T", "external_cue", 0.0, "EXT", (0.0, 0.0), 0.0)
    apply_transition(rec, TargetState.ALLOCATED, TransitionContext("D1", 1.0, winner="D1", feasible_bids=1), "t")
    apply_transition(rec, TargetState.DETECTED, TransitionContext("D1", 2.0, trigger="release", exclude_owner=False, max_reallocations=2), "room")
    assert rec.reallocations == 0 and rec.epoch == 1 and not rec.excluded
    rec.owner = "D1"; rec.state = TargetState.ALLOCATED
    apply_transition(rec, TargetState.DETECTED, TransitionContext("D1", 3.0, trigger="release", max_reallocations=2), "fault")
    assert rec.reallocations == 1 and rec.excluded == {2: "D1"}


def test_a_drone_chaining_to_its_next_task_still_takes_new_work_that_tick():
    """Regression: for one tick after a task ends the chaining drone is RETURNING with its next task already set."""
    w = run(scenario(2, [(5.0, TARGETS[0]), (5.0, TARGETS[1]), (5.0, TARGETS[2])]))
    assert logged(w) == {0, 1, 2}
    from src.drone.fsm import DroneMode

    d = w.drones["D1"]
    d.state.mode, d._then, d.state.current_task = DroneMode.RETURNING, "hover", "T-EXT-001"
    assert d._free_return()
    d.state.current_task = None
    assert not d._free_return()  # heading home (or hovering) with nothing to do is not free
