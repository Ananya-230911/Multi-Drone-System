import math

import pytest

from src.drone.mission import spiral_points
from src.logging_.target_log import LogEntry, merge
from src.tracking.target_state import (
    Confirmation,
    TargetRecord,
    TargetState,
    TransitionContext,
    apply_transition,
    can_transition,
)

S = TargetState


def test_square_spiral_grows_and_stops_at_length():
    pts = spiral_points((0.0, 0.0), 10.0, 100.0)
    assert pts[:4] == [(10.0, 0.0), (10.0, 10.0), (-10.0, 10.0), (-10.0, -10.0)]
    total = sum(math.dist(a, b) for a, b in zip([(0.0, 0.0)] + pts, pts))
    assert 100.0 <= total <= 100.0 + 50.0
    radii = [max(abs(x), abs(y)) for x, y in pts]
    assert radii == sorted(radii)


def rec(**kw):
    r = TargetRecord("T-D1-001", "sensor", 0.0, "D1", (0.0, 0.0))
    for k, v in kw.items():
        setattr(r, k, v)
    return r


def test_enroute_to_tracking_lost_only_when_arrived():
    r = rec(state=S.ENROUTE, owner="D1")
    assert not can_transition(r, S.TRACKING_LOST, TransitionContext("D1", 1.0, track_lost=True))[0]
    assert can_transition(r, S.TRACKING_LOST, TransitionContext("D1", 1.0, track_lost=True, in_region=True))[0]


def test_cue_rejected_on_site_from_enroute():
    r = rec(state=S.ENROUTE, owner="D1", source="external_cue", confirmation=Confirmation.REJECTED)
    ev = apply_transition(r, S.CANCELLED, TransitionContext("D1", 1.0, in_region=True), "cue_not_confirmed")
    assert ev.row == 17 and r.cancel_reason == "cue_not_confirmed"


def test_logging_requires_confirmation_and_inspection_conditions():
    r = rec(state=S.TRACKING, owner="D1", confirmation=Confirmation.PENDING)
    assert not can_transition(r, S.LOGGED, TransitionContext("D1", 1.0, inspection_conditions_met=True))[0]
    r.confirmation = Confirmation.CONFIRMED
    assert not can_transition(r, S.LOGGED, TransitionContext("D1", 1.0, inspection_conditions_met=False))[0]
    assert not can_transition(r, S.LOGGED, TransitionContext("D2", 1.0, inspection_conditions_met=True))[0]  # not owner
    assert can_transition(r, S.LOGGED, TransitionContext("D1", 1.0, inspection_conditions_met=True))[0]


def test_max_reallocations_cancels_active_task():
    r = rec(state=S.ENROUTE, owner="D1", confirmation=Confirmation.CONFIRMED, reallocations=2)
    assert not can_transition(r, S.CONFIRMED, TransitionContext("D1", 1.0, trigger="release"))[0]
    ev = apply_transition(r, S.CANCELLED, TransitionContext("D1", 1.0, trigger="max_reallocations"), "max_reallocations")
    assert ev.row == 6


def entry(tid, writer, state="COMPLETED", logged=10.0, owner="D1", aliases=""):
    return LogEntry(tid, aliases, "sensor", 1.0, "D1", 2.0, 3.0, owner, 0, 4.0 if logged else None, logged,
                    10.0 if logged else None, 0.8 if logged else None, 100.0, 100.0, 0.0, state, None, 0, writer)  # fmt: skip


def test_merge_resolves_aliases_and_marks_duplicate_inspections():
    logbooks = {"D1": [entry("T-D1-001", "D1", logged=10.0, owner="D1")],
                "D3": [entry("T-D3-001", "D3", logged=12.0, owner="D3")]}  # after a partition  # fmt: skip
    views = {"D2": [entry("T-D1-001", "D2", state="COMPLETED", logged=None, owner="D1")]}
    rows = merge(logbooks, views, {"T-D3-001": "T-D1-001"}, {"T-D1-001": 40})
    assert [(r["target_id"], r["owner"], r["duplicate_inspection"]) for r in rows] == [
        ("T-D1-001", "D1", False), ("T-D1-001", "D3", True)]
    assert rows[0]["messages_used"] == 40 and "T-D3-001" in rows[0]["aliases"]


def test_merge_without_logged_entry_uses_most_advanced_view():
    views = {"D1": [entry("T-D1-001", "D1", state="ALLOCATED", logged=None)],
             "D2": [entry("T-D1-001", "D2", state="CANCELLED", logged=None)]}  # fmt: skip
    [row] = merge({}, views, {}, {})
    assert row["final_state"] == "CANCELLED" and row["logged_at"] is None
