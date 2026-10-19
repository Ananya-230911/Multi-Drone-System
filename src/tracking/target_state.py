"""Target lifecycle state machine (PLAN.md section 7.8).

Each agent holds its own local view of each target record. This module holds
the transition table (rows 1-17), its guards and who may cause each
transition. Anything not in the table is illegal: `apply_transition` raises and
the caller logs ILLEGAL_TRANSITION and keeps its state.

UNKNOWN is the implicit state before a record exists (row 1 is the record
being created). COMPLETED and CANCELLED are terminal and sticky.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable

from src.config import Point2


class TargetState(str, Enum):
    UNKNOWN = "UNKNOWN"
    DETECTED = "DETECTED"
    CONFIRMED = "CONFIRMED"
    ALLOCATED = "ALLOCATED"
    ENROUTE = "ENROUTE"
    TRACKING = "TRACKING"
    TRACKING_LOST = "TRACKING_LOST"
    REACQUIRING = "REACQUIRING"
    LOGGED = "LOGGED"
    COMPLETED = "COMPLETED"
    CANCELLED = "CANCELLED"


class Confirmation(str, Enum):
    PENDING = "PENDING"
    CONFIRMED = "CONFIRMED"
    REJECTED = "REJECTED"


TERMINAL = frozenset({TargetState.COMPLETED, TargetState.CANCELLED})
# Progress order, used to pick "the more advanced" state when merging duplicate records.
STATE_RANK = {s: i for i, s in enumerate(TargetState)}
STATE_RANK[TargetState.CANCELLED] = STATE_RANK[TargetState.COMPLETED]


@dataclass
class TransitionEvent:
    target_id: str
    frm: TargetState
    to: TargetState
    row: int
    actor: str
    t: float
    reason: str
    epoch: int


@dataclass
class TargetRecord:
    id: str
    source: str  # sensor | external_cue | relayed
    created_at: float
    created_by: str
    est_pos: Point2
    est_alt: float | None = None
    state: TargetState = TargetState.DETECTED
    confirmation: Confirmation = Confirmation.PENDING
    epoch: int = 0
    owner: str | None = None
    reallocations: int = 0
    excluded: dict[int, str] = field(default_factory=dict)  # epoch -> previous owner excluded in that epoch
    aliases: set[str] = field(default_factory=set)
    # (t, confidence, observer, obs_id, is_sensor_evidence)
    observations: list[tuple[float, float, str, str, bool]] = field(default_factory=list)
    t_last_obs: float | None = None
    last_confidence: float = 0.0
    freshness: str = "FRESH"
    confirmed_at: float | None = None
    terminal_at: float | None = None
    cancel_reason: str | None = None
    history: list[TransitionEvent] = field(default_factory=list)

    @property
    def terminal(self) -> bool:
        return self.state in TERMINAL

    @property
    def is_cue(self) -> bool:
        return self.source == "external_cue"


@dataclass
class TransitionContext:
    """Facts the caller has established; guards only read these."""

    actor: str
    now: float
    trigger: str = ""  # e.g. "release", "owner_timeout", "allocation_timeout", "reacquire_timeout"
    winner: str | None = None  # rows 4 / 5
    feasible_bids: int = 0  # row 5
    route_valid: bool = False  # rows 7, 11
    inspection_point_valid: bool = False  # row 8
    track_fresh: bool = False  # rows 8, 9, 12
    track_lost: bool = False  # row 10
    in_region: bool = False  # rows 9, 12
    battery_ok: bool = False  # row 11
    inspection_conditions_met: bool = False  # row 14
    logged: bool = False  # row 15
    reacquire_action: str = "cancel"  # row 13 vs 16
    max_reallocations: int = 2  # row 16


class IllegalTransition(RuntimeError):
    pass


Guard = Callable[[TargetRecord, TransitionContext], bool]
S = TargetState
C = Confirmation


def _owner(rec: TargetRecord, ctx: TransitionContext) -> bool:
    return rec.owner is not None and ctx.actor == rec.owner


@dataclass(frozen=True)
class Row:
    n: int
    frm: frozenset[TargetState]
    to: TargetState
    guard: Guard
    owner_only: bool = False


def _reopen_guard(rec: TargetRecord, ctx: TransitionContext) -> bool:
    if rec.reallocations >= ctx.max_reallocations:
        return False
    if ctx.trigger == "release" or ctx.trigger == "route_unreachable" or ctx.trigger == "battery_reserve":
        return _owner(rec, ctx)  # the owner releases its own task
    if ctx.trigger == "reacquire_timeout":
        return _owner(rec, ctx) and ctx.reacquire_action == "reallocate"
    return ctx.trigger == "owner_timeout"  # any agent may reopen on owner heartbeat timeout


_ACTIVE = frozenset({S.ALLOCATED, S.ENROUTE, S.TRACKING, S.TRACKING_LOST, S.REACQUIRING})

TABLE: tuple[Row, ...] = (
    # row 1 (UNKNOWN -> DETECTED) is record creation, see `new_record`.
    # Row 2 guard "source != external_cue or evidence is sensor observations": confirmation is only
    # ever computed from sensor observations (tracker.py), so CONFIRMED evidence always qualifies.
    Row(2, frozenset({S.DETECTED}), S.CONFIRMED, lambda r, c: r.confirmation is C.CONFIRMED),
    Row(3, frozenset({S.DETECTED}), S.CANCELLED, lambda r, c: r.confirmation is C.REJECTED and not r.is_cue),
    Row(4, frozenset({S.DETECTED}), S.ALLOCATED, lambda r, c: r.is_cue and c.winner is not None),
    Row(5, frozenset({S.CONFIRMED}), S.ALLOCATED, lambda r, c: c.feasible_bids >= 1 and c.winner is not None),
    Row(6, frozenset({S.CONFIRMED}), S.CANCELLED, lambda r, c: c.trigger in ("allocation_timeout", "max_reallocations")),
    Row(6, frozenset({S.DETECTED}), S.CANCELLED,
        lambda r, c: r.is_cue and c.trigger in ("allocation_timeout", "max_reallocations")),
    Row(7, frozenset({S.ALLOCATED}), S.ENROUTE, lambda r, c: c.route_valid, owner_only=True),
    Row(8, frozenset({S.ALLOCATED}), S.TRACKING, lambda r, c: c.inspection_point_valid and c.track_fresh, owner_only=True),
    Row(9, frozenset({S.ENROUTE}), S.TRACKING, lambda r, c: c.in_region and c.track_fresh, owner_only=True),
    Row(10, frozenset({S.TRACKING}), S.TRACKING_LOST, lambda r, c: c.track_lost, owner_only=True),
    Row(11, frozenset({S.TRACKING_LOST}), S.REACQUIRING, lambda r, c: c.battery_ok and c.route_valid, owner_only=True),
    Row(12, frozenset({S.REACQUIRING}), S.TRACKING, lambda r, c: c.in_region and c.track_fresh, owner_only=True),
    Row(13, frozenset({S.REACQUIRING}), S.CANCELLED,
        lambda r, c: c.trigger == "reacquire_timeout" and c.reacquire_action == "cancel", owner_only=True),
    Row(14, frozenset({S.TRACKING}), S.LOGGED,
        lambda r, c: r.confirmation is C.CONFIRMED and c.inspection_conditions_met, owner_only=True),
    Row(15, frozenset({S.LOGGED}), S.COMPLETED, lambda r, c: c.logged, owner_only=True),
    Row(16, _ACTIVE, S.CONFIRMED, lambda r, c: not r.is_cue and _reopen_guard(r, c)),
    Row(16, _ACTIVE, S.DETECTED, lambda r, c: r.is_cue and _reopen_guard(r, c)),
    Row(17, frozenset({S.TRACKING, S.REACQUIRING}), S.CANCELLED,
        lambda r, c: r.is_cue and r.confirmation is C.REJECTED, owner_only=True),
)  # fmt: skip


def new_record(
    target_id: str, source: str, now: float, creator: str, pos: Point2, alt: float | None
) -> tuple[TargetRecord, TransitionEvent]:
    """Row 1: UNKNOWN -> DETECTED, i.e. the record comes into existence."""
    rec = TargetRecord(id=target_id, source=source, created_at=now, created_by=creator, est_pos=pos, est_alt=alt)
    ev = TransitionEvent(target_id, S.UNKNOWN, S.DETECTED, 1, creator, now, f"created_from_{source}", 0)
    rec.history.append(ev)
    return rec, ev


def find_row(rec: TargetRecord, to: TargetState, ctx: TransitionContext) -> tuple[Row | None, str]:
    if rec.terminal:
        return None, f"{rec.state.value} is terminal"
    candidates = [r for r in TABLE if rec.state in r.frm and r.to is to]
    if not candidates:
        return None, f"no row {rec.state.value} -> {to.value}"
    for row in candidates:
        if row.owner_only and not _owner(rec, ctx):
            continue
        if row.guard(rec, ctx):
            return row, ""
    if all(r.owner_only for r in candidates) and not _owner(rec, ctx):
        return None, f"only the owner may cause {rec.state.value} -> {to.value}"
    return None, f"guard failed for {rec.state.value} -> {to.value}"


def can_transition(rec: TargetRecord, to: TargetState, ctx: TransitionContext) -> tuple[bool, str]:
    row, why = find_row(rec, to, ctx)
    return row is not None, why or f"row {row.n}"


def apply_transition(rec: TargetRecord, to: TargetState, ctx: TransitionContext, reason: str) -> TransitionEvent:
    row, why = find_row(rec, to, ctx)
    if row is None:
        raise IllegalTransition(f"{rec.id}: {why}")
    frm = rec.state
    if row.n in (4, 5):
        rec.owner = ctx.winner
    elif row.n == 16:
        if rec.owner is not None:
            rec.excluded[rec.epoch + 1] = rec.owner
        rec.epoch += 1
        rec.reallocations += 1
        rec.owner = None
    rec.state = to
    if to is S.CONFIRMED and rec.confirmed_at is None:
        rec.confirmed_at = ctx.now
    if to in TERMINAL:
        rec.terminal_at = ctx.now
        if to is S.CANCELLED:
            rec.cancel_reason = reason
    ev = TransitionEvent(rec.id, frm, to, row.n, ctx.actor, ctx.now, reason, rec.epoch)
    rec.history.append(ev)
    return ev


def should_adopt_remote(
    rec: TargetRecord, msg_epoch: int, sender: str, to: TargetState, local_winner: str | None, ctx: TransitionContext
) -> bool:
    """Rule for adopting a peer's TARGET_STATE (used from Phase 3).

    Adopt if (a) the message epoch is higher than local, or (b) the epoch is equal,
    the sender is the owner in the local winner table and the transition is legal
    from the local state. Terminal states are adopted whenever the epoch is >= local.
    """
    if rec.terminal:
        return False
    if to in TERMINAL:
        return msg_epoch >= rec.epoch
    if msg_epoch > rec.epoch:
        return True
    if msg_epoch == rec.epoch and sender == local_winner:
        owner_ctx = TransitionContext(**{**ctx.__dict__, "actor": sender})
        return can_transition(rec, to, owner_ctx)[0]
    return False
