import pytest

from src.tracking.target_state import (
    TABLE,
    Confirmation,
    IllegalTransition,
    TargetRecord,
    TargetState,
    TransitionContext,
    apply_transition,
    can_transition,
    new_record,
    should_adopt_remote,
)

S, C = TargetState, Confirmation


def rec(state=S.DETECTED, source="sensor", confirmation=C.PENDING, owner=None, **kw):
    r = TargetRecord(id="T-D1-001", source=source, created_at=0.0, created_by="D1", est_pos=(0.0, 0.0))
    r.state, r.confirmation, r.owner = state, confirmation, owner
    for k, v in kw.items():
        setattr(r, k, v)
    return r


def ctx(actor="D1", **kw):
    return TransitionContext(actor=actor, now=1.0, **kw)


def test_row1_new_record_is_detected():
    r, ev = new_record("T-D1-001", "sensor", 5.0, "D1", (1.0, 2.0), 0.0)
    assert r.state is S.DETECTED and r.confirmation is C.PENDING and r.epoch == 0
    assert (ev.frm, ev.to, ev.row) == (S.UNKNOWN, S.DETECTED, 1)


# (row, record kwargs, target state, context kwargs)
LEGAL = [
    (2, dict(confirmation=C.CONFIRMED), S.CONFIRMED, {}),
    (2, dict(confirmation=C.CONFIRMED, source="external_cue"), S.CONFIRMED, {}),
    (3, dict(confirmation=C.REJECTED), S.CANCELLED, {}),
    (4, dict(source="external_cue"), S.ALLOCATED, dict(winner="D2")),
    (5, dict(state=S.CONFIRMED, confirmation=C.CONFIRMED), S.ALLOCATED, dict(winner="D2", feasible_bids=2)),
    (6, dict(state=S.CONFIRMED, confirmation=C.CONFIRMED), S.CANCELLED, dict(trigger="allocation_timeout")),
    (6, dict(source="external_cue"), S.CANCELLED, dict(trigger="max_reallocations")),
    (7, dict(state=S.ALLOCATED, owner="D1"), S.ENROUTE, dict(route_valid=True)),
    (8, dict(state=S.ALLOCATED, owner="D1"), S.TRACKING, dict(inspection_point_valid=True, track_fresh=True)),
    (9, dict(state=S.ENROUTE, owner="D1"), S.TRACKING, dict(in_region=True, track_fresh=True)),
    (10, dict(state=S.TRACKING, owner="D1"), S.TRACKING_LOST, dict(track_lost=True)),
    (11, dict(state=S.TRACKING_LOST, owner="D1"), S.REACQUIRING, dict(battery_ok=True, route_valid=True)),
    (12, dict(state=S.REACQUIRING, owner="D1"), S.TRACKING, dict(in_region=True, track_fresh=True)),
    (13, dict(state=S.REACQUIRING, owner="D1"), S.CANCELLED, dict(trigger="reacquire_timeout")),
    (14, dict(state=S.TRACKING, owner="D1", confirmation=C.CONFIRMED), S.LOGGED, dict(inspection_conditions_met=True)),
    (15, dict(state=S.LOGGED, owner="D1", confirmation=C.CONFIRMED), S.COMPLETED, dict(logged=True)),
    (16, dict(state=S.ENROUTE, owner="D1", confirmation=C.CONFIRMED), S.CONFIRMED, dict(trigger="release")),
    (16, dict(state=S.TRACKING, owner="D2", confirmation=C.CONFIRMED), S.CONFIRMED, dict(trigger="owner_timeout")),
    (16, dict(state=S.REACQUIRING, owner="D1", source="external_cue"), S.DETECTED,
     dict(trigger="reacquire_timeout", reacquire_action="reallocate")),
    (17, dict(state=S.TRACKING, owner="D1", source="external_cue", confirmation=C.REJECTED), S.CANCELLED, {}),
]  # fmt: skip


@pytest.mark.parametrize("row, rkw, to, ckw", LEGAL, ids=[f"row{r[0]}-{r[2].value}" for r in LEGAL])
def test_every_legal_row(row, rkw, to, ckw):
    r = rec(**rkw)
    ev = apply_transition(r, to, ctx(**ckw), "test")
    assert ev.row == row and r.state is to


def test_every_table_row_is_covered():
    assert {r.n for r in TABLE} == {row for row, *_ in LEGAL}


ILLEGAL = [
    (dict(), S.TRACKING, {}),  # skipping straight from DETECTED
    (dict(), S.CONFIRMED, {}),  # confirmation still PENDING
    (dict(source="external_cue", confirmation=C.REJECTED), S.CANCELLED, {}),  # row 3 excludes cues
    (dict(), S.ALLOCATED, dict(winner="D2")),  # row 4 only for cues
    (dict(state=S.CONFIRMED, confirmation=C.CONFIRMED), S.ALLOCATED, dict(winner="D2", feasible_bids=0)),
    (dict(state=S.ALLOCATED, owner="D2"), S.ENROUTE, dict(route_valid=True)),  # D1 is not the owner
    (dict(state=S.ALLOCATED, owner="D1"), S.ENROUTE, dict(route_valid=False)),
    (dict(state=S.TRACKING, owner="D1", confirmation=C.PENDING), S.LOGGED, dict(inspection_conditions_met=True)),
    (dict(state=S.TRACKING, owner="D1", confirmation=C.CONFIRMED), S.LOGGED, dict(inspection_conditions_met=False)),
    (dict(state=S.REACQUIRING, owner="D1"), S.CANCELLED, dict(trigger="reacquire_timeout", reacquire_action="reallocate")),
    (dict(state=S.ENROUTE, owner="D2", confirmation=C.CONFIRMED), S.CONFIRMED, dict(trigger="release")),  # not owner
    (dict(state=S.ENROUTE, owner="D1", confirmation=C.CONFIRMED, reallocations=2), S.CONFIRMED, dict(trigger="release")),
    (dict(state=S.CONFIRMED, confirmation=C.CONFIRMED), S.DETECTED, {}),
    (dict(state=S.LOGGED, owner="D1"), S.TRACKING, dict(in_region=True, track_fresh=True)),
]  # fmt: skip


@pytest.mark.parametrize("rkw, to, ckw", ILLEGAL)
def test_illegal_transitions_raise_and_keep_state(rkw, to, ckw):
    r = rec(**rkw)
    before = r.state
    ok, why = can_transition(r, to, ctx(**ckw))
    assert not ok and why
    with pytest.raises(IllegalTransition):
        apply_transition(r, to, ctx(**ckw), "test")
    assert r.state is before


@pytest.mark.parametrize("terminal", [S.COMPLETED, S.CANCELLED])
@pytest.mark.parametrize("to", list(TargetState))
def test_terminal_states_are_sticky(terminal, to):
    r = rec(state=terminal, owner="D1", confirmation=C.CONFIRMED)
    assert not can_transition(r, to, ctx(trigger="release", winner="D1", feasible_bids=1))[0]


def test_reopen_increments_epoch_and_excludes_owner():
    r = rec(state=S.TRACKING, owner="D2", confirmation=C.CONFIRMED)
    ev = apply_transition(r, S.CONFIRMED, ctx(trigger="owner_timeout"), "owner_timeout")
    assert r.epoch == 1 and ev.epoch == 1 and r.reallocations == 1
    assert r.owner is None and r.excluded == {1: "D2"}


def test_allocation_sets_owner():
    r = rec(state=S.CONFIRMED, confirmation=C.CONFIRMED)
    apply_transition(r, S.ALLOCATED, ctx(winner="D3", feasible_bids=1), "consensus")
    assert r.owner == "D3"


def test_cancel_records_reason_and_time():
    r = rec(confirmation=C.REJECTED)
    apply_transition(r, S.CANCELLED, ctx(), "confirmation_rejected")
    assert r.cancel_reason == "confirmation_rejected" and r.terminal_at == 1.0


def test_history_is_kept():
    r = rec(confirmation=C.CONFIRMED)
    apply_transition(r, S.CONFIRMED, ctx(), "criteria_met")
    assert [h.to for h in r.history] == [S.CONFIRMED]


# --- remote adoption rule (used from Phase 3)


def test_adopt_higher_epoch():
    r = rec(state=S.ALLOCATED, owner="D2", confirmation=C.CONFIRMED)
    assert should_adopt_remote(r, 1, "D3", S.ALLOCATED, "D2", ctx())


def test_adopt_equal_epoch_only_from_owner_with_legal_transition():
    r = rec(state=S.ALLOCATED, owner="D2", confirmation=C.CONFIRMED)
    assert should_adopt_remote(r, 0, "D2", S.ENROUTE, "D2", ctx(route_valid=True))
    assert not should_adopt_remote(r, 0, "D3", S.ENROUTE, "D2", ctx(route_valid=True))  # not the owner
    assert not should_adopt_remote(r, 0, "D2", S.LOGGED, "D2", ctx())  # not legal from ALLOCATED


def test_adopt_terminal_when_epoch_not_lower():
    r = rec(state=S.TRACKING, owner="D2", confirmation=C.CONFIRMED, epoch=1)
    assert should_adopt_remote(r, 1, "D3", S.COMPLETED, "D2", ctx())
    assert not should_adopt_remote(r, 0, "D3", S.COMPLETED, "D2", ctx())


def test_stale_epoch_not_adopted():
    r = rec(state=S.ALLOCATED, owner="D2", confirmation=C.CONFIRMED, epoch=2)
    assert not should_adopt_remote(r, 1, "D2", S.ENROUTE, "D2", ctx(route_valid=True))
