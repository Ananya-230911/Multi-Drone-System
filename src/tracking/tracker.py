"""Per-agent target records: association, duplicate merge, confirmation, freshness
(PLAN.md sections 7.4, 7.5 and 7.7).

Each agent owns one Tracker. In Phase 2 records come only from the agent's own
sensor; peer reports (`merge_remote`) arrive with messages in Phase 3.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Literal

from src.config import AssociationConfig, ConfirmationConfig, TrackConfig
from src.tracking.observation import AgentObservation
from src.tracking.target_state import (
    STATE_RANK,
    Confirmation,
    TargetRecord,
    TargetState,
    TransitionContext,
    TransitionEvent,
    apply_transition,
    new_record,
)

Freshness = Literal["FRESH", "STALE", "LOST"]


@dataclass
class TrackUpdate:
    kind: Literal["created", "associated", "ignored"]
    target_id: str | None
    transition: TransitionEvent | None = None  # row 1 when created
    merges: list[tuple[str, str]] = field(default_factory=list)  # (survivor, alias)


@dataclass
class TrackerEvent:
    """Something the agent should log: CONFIRMATION_RESULT, TARGET_TRANSITION or TRACK_FRESHNESS."""

    type: str
    target_id: str
    epoch: int
    data: dict


class Tracker:
    def __init__(
        self,
        owner_id: str,
        assoc: AssociationConfig,
        confirm: ConfirmationConfig,
        track: TrackConfig,
    ):
        self.owner_id = owner_id
        self.assoc = assoc
        self.confirm = confirm
        self.track = track
        self.records: dict[str, TargetRecord] = {}
        self.alias_of: dict[str, str] = {}  # alias -> survivor
        self.seen_obs: set[str] = set()
        self._seq = 0

    # ------------------------------------------------------------------ lookup

    def resolve(self, target_id: str) -> str:
        while target_id in self.alias_of:
            target_id = self.alias_of[target_id]
        return target_id

    def record(self, target_id: str) -> TargetRecord | None:
        return self.records.get(self.resolve(target_id))

    def freshness(self, target_id: str, now: float) -> Freshness:
        rec = self.record(target_id)
        if rec is None or rec.t_last_obs is None:
            return "LOST"
        age = now - rec.t_last_obs
        if age <= self.track.stale_after:
            return "FRESH"
        if age <= self.track.lost_after:
            return "STALE"
        return "LOST"

    # ------------------------------------------------------------------ ingest

    def ingest(self, obs: AgentObservation, now: float) -> TrackUpdate:
        """Attach a validated observation to a record, creating one if none is near enough."""
        self.seen_obs.add(obs.obs_id)
        rec = self._nearest_active(obs.pos)
        if rec is None:
            if self._near_recently_completed(obs.pos, now):
                return TrackUpdate("ignored", None)
            rec, ev = self._create(obs, now)
            self._attach(rec, obs)
            return TrackUpdate("created", rec.id, transition=ev, merges=self._merge_duplicates(rec.id))
        self._attach(rec, obs)
        return TrackUpdate("associated", rec.id, merges=self._merge_duplicates(rec.id))

    def _nearest_active(self, pos) -> TargetRecord | None:
        best: tuple[float, str] | None = None
        for rid in sorted(self.records):
            rec = self.records[rid]
            if rec.terminal:
                continue
            d = math.dist(rec.est_pos, pos)
            if d <= self.assoc.radius and (best is None or d < best[0]):
                best = (d, rid)
        return None if best is None else self.records[best[1]]

    def _near_recently_completed(self, pos, now: float) -> bool:
        return any(
            rec.state is TargetState.COMPLETED
            and rec.terminal_at is not None
            and now - rec.terminal_at <= self.assoc.ignore_after_complete
            and math.dist(rec.est_pos, pos) <= self.assoc.radius
            for rec in self.records.values()
        )

    def _create(self, obs: AgentObservation, now: float) -> tuple[TargetRecord, TransitionEvent]:
        self._seq += 1
        creator = "EXT" if obs.source == "external_cue" else self.owner_id
        tid = f"T-{creator}-{self._seq:03d}"
        rec, ev = new_record(tid, obs.source, now, self.owner_id, obs.pos, obs.alt)
        self.records[tid] = rec
        return rec, ev

    def _attach(self, rec: TargetRecord, obs: AgentObservation) -> None:
        evidence = obs.source in ("sensor", "relayed")  # an external cue is never evidence
        rec.observations.append((obs.t, obs.confidence, obs.observer, obs.obs_id, evidence))
        if len(rec.observations) > 1:
            s = self.track.smoothing  # weight of the new observation in the exponential average
            rec.est_pos = ((1 - s) * rec.est_pos[0] + s * obs.pos[0], (1 - s) * rec.est_pos[1] + s * obs.pos[1])
        if evidence:
            rec.t_last_obs = obs.t if rec.t_last_obs is None else max(rec.t_last_obs, obs.t)
            rec.last_confidence = obs.confidence

    # ------------------------------------------------------------------ duplicates

    def _merge_duplicates(self, changed_id: str) -> list[tuple[str, str]]:
        """Merge any non-terminal record within the association radius of `changed_id`.

        The lexicographically smallest ID survives, so every agent converges on the same ID.
        """
        merges = []
        changed_id = self.resolve(changed_id)
        while True:
            rec = self.records.get(changed_id)
            if rec is None or rec.terminal:
                return merges
            other = next(
                (
                    o for oid, o in sorted(self.records.items())
                    if oid != changed_id and not o.terminal and math.dist(o.est_pos, rec.est_pos) <= self.assoc.radius
                ),
                None,
            )  # fmt: skip
            if other is None:
                return merges
            survivor, alias = sorted([rec.id, other.id])
            self._merge(survivor, alias)
            merges.append((survivor, alias))
            changed_id = survivor

    def _merge(self, survivor_id: str, alias_id: str) -> None:
        s, a = self.records[survivor_id], self.records.pop(alias_id)
        ns, na = max(len(s.observations), 1), max(len(a.observations), 1)  # weights for the merged estimate
        known = {o[3] for o in s.observations}
        s.observations = sorted(s.observations + [o for o in a.observations if o[3] not in known])
        s.est_pos = ((s.est_pos[0] * ns + a.est_pos[0] * na) / (ns + na), (s.est_pos[1] * ns + a.est_pos[1] * na) / (ns + na))
        if STATE_RANK[a.state] > STATE_RANK[s.state]:
            s.state, s.owner, s.epoch = a.state, a.owner, a.epoch
        if a.confirmation is Confirmation.CONFIRMED or s.confirmation is Confirmation.CONFIRMED:
            s.confirmation = Confirmation.CONFIRMED
            s.confirmed_at = min(t for t in (s.confirmed_at, a.confirmed_at) if t is not None)
        s.created_at = min(s.created_at, a.created_at)
        s.aliases |= a.aliases | {alias_id}
        if a.t_last_obs is not None and (s.t_last_obs is None or a.t_last_obs > s.t_last_obs):
            s.t_last_obs, s.last_confidence = a.t_last_obs, a.last_confidence
        self.alias_of[alias_id] = survivor_id
        for al in a.aliases:
            self.alias_of[al] = survivor_id

    # ------------------------------------------------------------------ confirmation / lifecycle

    def check_confirmation(self, target_id: str, now: float) -> Confirmation:
        """Section 7.5 criteria over the sliding window. Does not change the record."""
        rec = self.record(target_id)
        if rec is None:
            return Confirmation.REJECTED
        if rec.confirmation is not Confirmation.PENDING:
            return rec.confirmation
        c = self.confirm
        window = [o for o in rec.observations if o[4] and now - c.window <= o[0] <= now]
        if (
            len(window) >= c.min_observations
            and sum(o[1] for o in window) / len(window) >= c.min_mean_confidence
            and len({o[2] for o in window}) >= c.min_distinct_observers
        ):
            return Confirmation.CONFIRMED
        if now - rec.created_at > c.timeout:
            return Confirmation.REJECTED
        return Confirmation.PENDING

    def update(self, now: float) -> list[TrackerEvent]:
        """Confirmation results, the lifecycle transitions they cause, and freshness changes."""
        events: list[TrackerEvent] = []
        for tid in sorted(self.records):
            rec = self.records[tid]
            if rec.terminal:
                continue
            if rec.confirmation is Confirmation.PENDING:
                result = self.check_confirmation(tid, now)
                if result is not Confirmation.PENDING:
                    rec.confirmation = result
                    window = [o for o in rec.observations if o[4] and now - self.confirm.window <= o[0] <= now]
                    events.append(TrackerEvent("CONFIRMATION_RESULT", tid, rec.epoch, {
                        "result": result.value,
                        "observations_in_window": len(window),
                        "mean_confidence": sum(o[1] for o in window) / len(window) if window else 0.0,
                        "observers": sorted({o[2] for o in window}),
                    }))  # fmt: skip
                    ctx = TransitionContext(actor=self.owner_id, now=now)
                    if result is Confirmation.CONFIRMED and rec.state is TargetState.DETECTED:
                        events.append(self._transition_event(apply_transition(rec, TargetState.CONFIRMED, ctx, "criteria_met")))
                    elif result is Confirmation.REJECTED and rec.state is TargetState.DETECTED and not rec.is_cue:
                        events.append(self._transition_event(apply_transition(rec, TargetState.CANCELLED, ctx, "confirmation_rejected")))
            fresh = self.freshness(tid, now)
            if fresh != rec.freshness and not rec.terminal:
                events.append(TrackerEvent("TRACK_FRESHNESS", tid, rec.epoch, {"from": rec.freshness, "to": fresh}))
                rec.freshness = fresh
        return events

    @staticmethod
    def _transition_event(ev: TransitionEvent) -> TrackerEvent:
        return TrackerEvent("TARGET_TRANSITION", ev.target_id, ev.epoch, {
            "from": ev.frm.value, "to": ev.to.value, "row": ev.row, "reason": ev.reason, "by": ev.actor,
        })  # fmt: skip

    def merge_remote(self, msg) -> None:
        """OBSERVATION / TARGET_STATE / MERGE messages from peers arrive with the network in Phase 3."""
        raise NotImplementedError("peer messages are added in Phase 3")
