import pytest

from src.config import AssociationConfig, ConfirmationConfig, TrackConfig
from src.tracking.observation import AgentObservation
from src.tracking.target_state import Confirmation, TargetState, TransitionContext, apply_transition
from src.tracking.tracker import Tracker

ASSOC = AssociationConfig()  # radius 30
CONF = ConfirmationConfig()  # window 5, timeout 15, 3 obs, mean >= 0.6, 1 observer
TRACK = TrackConfig()  # smoothing 0.3, stale 1.5, lost 3.0

_n = 0


def obs(t, x, y, conf=0.9, observer="D1", source="sensor"):
    global _n
    _n += 1
    return AgentObservation(f"{observer}-{_n}", observer, source, t, (x, y), 0.0, conf)


def tracker(**confirm):
    return Tracker("D1", ASSOC, ConfirmationConfig(**{**CONF.__dict__, **confirm}), TRACK)


def test_first_observation_creates_detected_record():
    tr = tracker()
    u = tr.ingest(obs(1.0, 100, 100), 1.0)
    assert u.kind == "created" and u.target_id == "T-D1-001" and u.transition.row == 1
    rec = tr.record("T-D1-001")
    assert rec.state is TargetState.DETECTED and rec.confirmation is Confirmation.PENDING and rec.source == "sensor"


def test_association_radius():
    tr = tracker()
    tr.ingest(obs(1.0, 100, 100), 1.0)
    assert tr.ingest(obs(1.5, 125, 100), 1.5).kind == "associated"  # 25 m
    assert tr.ingest(obs(2.0, 170, 100), 2.0).kind == "created"  # far from the estimate
    assert sorted(tr.records) == ["T-D1-001", "T-D1-002"]


def test_estimate_is_exponential_average():
    tr = tracker()
    tr.ingest(obs(1.0, 100, 100), 1.0)
    tr.ingest(obs(1.5, 110, 100), 1.5)
    assert tr.record("T-D1-001").est_pos == pytest.approx((103.0, 100.0))  # 0.7 * 100 + 0.3 * 110


def test_duplicate_records_merge_into_smallest_id_and_alias_redirects():
    tr = tracker()
    tr.ingest(obs(1.0, 100, 100), 1.0)  # T-D1-001
    tr.ingest(obs(1.1, 135, 100), 1.1)  # 35 m away -> T-D1-002
    assert len(tr.records) == 2
    # Observations near 002 pull its estimate toward 001 until they are within 30 m.
    merges = []
    for k in range(10):
        merges += tr.ingest(obs(1.2 + 0.1 * k, 118, 100), 1.2 + 0.1 * k).merges
    assert merges == [("T-D1-001", "T-D1-002")]
    assert list(tr.records) == ["T-D1-001"]
    survivor = tr.record("T-D1-002")  # alias resolves
    assert survivor.id == "T-D1-001" and "T-D1-002" in survivor.aliases
    assert len(survivor.observations) == 12


def test_merge_keeps_more_advanced_state():
    tr = tracker()
    tr.ingest(obs(1.0, 100, 100), 1.0)
    tr.ingest(obs(1.1, 135, 100), 1.1)
    r2 = tr.records["T-D1-002"]
    r2.confirmation = Confirmation.CONFIRMED
    apply_transition(r2, TargetState.CONFIRMED, TransitionContext("D1", 1.1), "test")
    for k in range(10):
        tr.ingest(obs(1.2 + 0.1 * k, 118, 100), 1.2 + 0.1 * k)
    s = tr.record("T-D1-001")
    assert s.state is TargetState.CONFIRMED and s.confirmation is Confirmation.CONFIRMED


def test_no_merge_into_terminal_records():
    tr = tracker()
    tr.ingest(obs(1.0, 100, 100), 1.0)
    r = tr.records["T-D1-001"]
    r.confirmation = Confirmation.REJECTED
    apply_transition(r, TargetState.CANCELLED, TransitionContext("D1", 1.0), "test")
    u = tr.ingest(obs(2.0, 105, 100), 2.0)
    assert u.kind == "created" and u.target_id == "T-D1-002" and not u.merges
    assert tr.records["T-D1-001"].state is TargetState.CANCELLED


def test_ignore_new_observations_near_recently_completed_target():
    tr = tracker()
    tr.ingest(obs(1.0, 100, 100), 1.0)
    r = tr.records["T-D1-001"]
    r.state, r.terminal_at = TargetState.COMPLETED, 10.0
    assert tr.ingest(obs(20.0, 105, 100), 20.0).kind == "ignored"
    assert tr.ingest(obs(80.0, 105, 100), 80.0).kind == "created"  # after ignore_after_complete (60 s)


def test_confirmation_criteria_met():
    tr = tracker()
    for k in range(3):
        tr.ingest(obs(1.0 + 0.5 * k, 100, 100, conf=0.7), 1.0 + 0.5 * k)
    events = tr.update(2.0)
    assert tr.record("T-D1-001").state is TargetState.CONFIRMED
    types = [e.type for e in events]
    assert "CONFIRMATION_RESULT" in types and "TARGET_TRANSITION" in types
    tr_ev = next(e for e in events if e.type == "TARGET_TRANSITION")
    assert tr_ev.data["row"] == 2


def test_low_mean_confidence_does_not_confirm():
    tr = tracker()
    for k in range(6):
        tr.ingest(obs(1.0 + 0.5 * k, 100, 100, conf=0.55), 1.0 + 0.5 * k)
    tr.update(4.0)
    assert tr.record("T-D1-001").confirmation is Confirmation.PENDING


def test_window_expiry_rejects_and_cancels():
    tr = tracker()
    tr.ingest(obs(1.0, 100, 100, conf=0.9), 1.0)
    tr.ingest(obs(1.5, 100, 100, conf=0.9), 1.5)  # only 2 observations, then nothing
    tr.update(10.0)
    assert tr.record("T-D1-001").confirmation is Confirmation.PENDING
    events = tr.update(16.1)  # 15 s timeout after creation
    rec = tr.record("T-D1-001")
    assert rec.confirmation is Confirmation.REJECTED and rec.state is TargetState.CANCELLED
    assert any(e.type == "TARGET_TRANSITION" and e.data["row"] == 3 for e in events)


def test_old_observations_leave_the_window():
    tr = tracker()
    tr.ingest(obs(1.0, 100, 100, conf=0.9), 1.0)
    tr.ingest(obs(1.5, 100, 100, conf=0.9), 1.5)
    tr.ingest(obs(7.0, 100, 100, conf=0.9), 7.0)  # 3 in total, but never 3 inside a 5 s window
    tr.update(7.0)
    assert tr.record("T-D1-001").confirmation is Confirmation.PENDING


def test_spurious_like_observations_never_confirm():
    tr = tracker()
    t = 0.0
    for k in range(40):  # confidence capped at 0.6 like U(0.2, 0.6)
        t += 0.25
        tr.ingest(obs(t, 100, 100, conf=0.2 + 0.4 * (k % 10) / 10), t)
        tr.update(t)
    assert all(r.confirmation is not Confirmation.CONFIRMED for r in tr.records.values())


def test_distinct_observers_requirement():
    tr = tracker(min_distinct_observers=2)
    for k in range(4):
        tr.ingest(obs(1.0 + 0.5 * k, 100, 100, conf=0.9), 1.0 + 0.5 * k)
    tr.update(3.0)
    assert tr.record("T-D1-001").confirmation is Confirmation.PENDING
    tr.ingest(obs(3.0, 100, 100, conf=0.9, observer="D2"), 3.0)
    tr.update(3.0)
    assert tr.record("T-D1-001").confirmation is Confirmation.CONFIRMED


def test_external_cue_is_not_evidence():
    tr = tracker()
    u = tr.ingest(obs(1.0, 100, 100, conf=1.0, observer="EXT", source="external_cue"), 1.0)
    assert u.target_id == "T-EXT-001"
    tr.ingest(obs(1.1, 100, 100, conf=1.0, observer="EXT", source="external_cue"), 1.1)
    tr.ingest(obs(1.2, 100, 100, conf=1.0, observer="EXT", source="external_cue"), 1.2)
    tr.update(1.2)
    assert tr.record("T-EXT-001").confirmation is Confirmation.PENDING


def test_freshness_transitions():
    tr = tracker()
    tr.ingest(obs(10.0, 100, 100), 10.0)
    assert tr.freshness("T-D1-001", 11.5) == "FRESH"  # 1.5 s
    assert tr.freshness("T-D1-001", 11.6) == "STALE"
    assert tr.freshness("T-D1-001", 13.0) == "STALE"  # 3.0 s
    assert tr.freshness("T-D1-001", 13.1) == "LOST"
    ev = [e for e in tr.update(13.1) if e.type == "TRACK_FRESHNESS"]
    assert ev and ev[-1].data == {"from": "FRESH", "to": "LOST"}


def test_merge_remote_is_phase3():
    with pytest.raises(NotImplementedError):
        tracker().merge_remote(None)
