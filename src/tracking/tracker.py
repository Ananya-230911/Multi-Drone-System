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
    TERMINAL,
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
        # A cue cannot be checked until a drone is on site (7.9); its on-site timeout is Phase 4.
        if now - rec.created_at > c.timeout and not rec.is_cue:
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
                    if result is Confirmation.CONFIRMED:
                        rec.evidence = [(o[0], o[1], o[2], o[3]) for o in window]
                        rec.confirmed_at = now  # also for a cue confirmed on site, while ENROUTE / TRACKING
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

    # ------------------------------------------------------------------ peers (Phase 3)

    def summary(self, target_id: str, with_evidence: bool = False) -> dict:
        """What this agent tells peers about a record (TARGET_DETECTED, OBSERVATION, TARGET_STATE, BID)."""
        rec = self.record(target_id)
        assert rec is not None
        d = {
            "target_id": rec.id, "aliases": sorted(rec.aliases), "est_pos": [rec.est_pos[0], rec.est_pos[1]],
            "alt": rec.est_alt, "source": rec.source, "state": rec.state.value, "confirmation": rec.confirmation.value,
            "epoch": rec.epoch, "owner": rec.owner, "created_at": rec.created_at, "created_by": rec.created_by,
            "reallocations": rec.reallocations, "excluded": dict(rec.excluded), "cancel_reason": rec.cancel_reason,
            "version": len(rec.history) if rec.owner == self.owner_id else rec.owner_version,
        }  # fmt: skip
        if with_evidence and rec.evidence:
            d["evidence"] = [list(o) for o in rec.evidence]
        return d

    def apply_summary(self, summ: dict, sender: str, now: float, known_observers: frozenset[str]) -> RemoteUpdate:
        """Fold a peer's view of a record into ours: create it if new, link aliases, check its evidence."""
        out = RemoteUpdate()
        tid = self.resolve(summ["target_id"])
        if tid not in self.records:
            tid = next((self.resolve(a) for a in summ.get("aliases", []) if self.resolve(a) in self.records), tid)
        rec = self.records.get(tid)
        if rec is None:
            if summ.get("state") in ("COMPLETED", "CANCELLED"):
                return out  # nothing to do for a record we never had that is already over
            pos = (float(summ["est_pos"][0]), float(summ["est_pos"][1]))
            rec, ev = new_record(summ["target_id"], summ["source"], float(summ.get("created_at", now)),
                                 summ.get("created_by", sender), pos, summ.get("alt"))  # fmt: skip
            rec.history[-1].reason = f"relayed_from:{sender}"
            rec.relayed = True
            self.records[rec.id] = rec
            out.created, out.transition = rec.id, ev
        names = summ.get("aliases", []) + [summ["target_id"]]
        smaller = min((al for al in names if self.resolve(al) == al and al not in self.records and al < rec.id), default=None)
        if smaller is not None:  # peers already merged under a smaller ID: the smallest ID survives everywhere
            old = rec.id
            self.records.pop(old)
            rec.id = smaller
            rec.aliases.add(old)
            self.records[smaller] = rec
            self.alias_of[old] = smaller
            for al in rec.aliases:
                if al != smaller:
                    self.alias_of[al] = smaller
            out.merges.append((smaller, old))
        out.target_id = rec.id
        for al in names:
            if al != rec.id and self.resolve(al) == al and al not in self.records:
                self.alias_of[al] = rec.id
                rec.aliases.add(al)
        if summ.get("evidence") and rec.confirmation is Confirmation.PENDING and not rec.terminal:
            ok, info = self.check_evidence(summ["evidence"], now, known_observers)
            if ok:
                ev_obs = [(float(t), float(c), str(o), str(i)) for t, c, o, i in summ["evidence"]]
                known = {o[3] for o in rec.observations}
                rec.observations = sorted(rec.observations + [(t, c, o, i, True) for t, c, o, i in ev_obs if i not in known])
                self.seen_obs.update(o[3] for o in ev_obs)
                rec.confirmation, rec.evidence = Confirmation.CONFIRMED, ev_obs
                rec.confirmed_at = rec.confirmed_at if rec.confirmed_at is not None else now
                out.events.append(TrackerEvent("CONFIRMATION_RESULT", rec.id, rec.epoch, {
                    "result": "CONFIRMED", "via": f"peer_evidence:{sender}", **info}))  # fmt: skip
                if rec.state is TargetState.DETECTED:
                    ctx = TransitionContext(actor=self.owner_id, now=now)
                    out.events.append(self._transition_event(
                        apply_transition(rec, TargetState.CONFIRMED, ctx, f"peer_evidence:{sender}")))  # fmt: skip
        adopted = self._adopt_remote_state(rec, summ, sender, now)
        if adopted is not None:
            out.events.append(adopted)
        out.merges += self._merge_duplicates(rec.id)
        out.target_id = self.resolve(rec.id)
        return out

    def _adopt_remote_state(self, rec: TargetRecord, summ: dict, sender: str, now: float) -> TrackerEvent | None:
        """Section 7.8 adoption rule for a peer's view of the lifecycle.

        - a terminal state is adopted whenever its epoch is >= ours;
        - a higher epoch (a reopen elsewhere) is adopted;
        - at the same epoch, progress reported by the owner itself is adopted (the owner is the
          only agent allowed to move the task forward after allocation).
        """
        if rec.terminal:
            return None
        to = TargetState(summ["state"])
        r_epoch = int(summ.get("epoch", 0))
        if to is rec.state and r_epoch == rec.epoch:
            return None
        owner = summ.get("owner")
        version = int(summ.get("version", 0))
        if to in TERMINAL:
            ok = r_epoch >= rec.epoch
        elif r_epoch > rec.epoch:
            ok = True
        else:
            ok = (r_epoch == rec.epoch and owner == sender and rec.owner in (None, sender)
                  and to not in (TargetState.DETECTED, TargetState.CONFIRMED)
                  and version > rec.owner_version)  # newer than what we adopted from the owner  # fmt: skip
        if not ok:
            return None
        rec.owner_version = version
        frm = rec.state
        rec.state, rec.epoch, rec.owner = to, r_epoch, owner
        rec.reallocations = max(rec.reallocations, int(summ.get("reallocations", 0)))
        rec.excluded.update({int(k): v for k, v in summ.get("excluded", {}).items()})
        if to is TargetState.CONFIRMED and rec.confirmed_at is None:
            rec.confirmed_at = now
        if to in TERMINAL:
            rec.terminal_at = now
            rec.cancel_reason = summ.get("cancel_reason")
        return TrackerEvent("TARGET_TRANSITION", rec.id, rec.epoch, {
            "from": frm.value, "to": to.value, "row": None, "reason": f"adopted_from:{sender}", "by": self.owner_id,
        })  # fmt: skip

    def check_evidence(self, evidence: list, now: float, known_observers: frozenset[str]) -> tuple[bool, dict]:
        """The 7.5 criteria applied to a peer's evidence, in a window ending at its newest observation."""
        c = self.confirm
        obs = []
        for item in evidence:
            t, conf, observer, obs_id = float(item[0]), float(item[1]), str(item[2]), str(item[3])
            if t > now + 1e-9 or not 0.0 <= conf <= 1.0 or observer not in known_observers:
                return False, {"rejected": "invalid_evidence"}
            obs.append((t, conf, observer, obs_id))
        if not obs:
            return False, {"rejected": "empty"}
        t_end = max(o[0] for o in obs)
        if now - t_end > c.window:
            return False, {"rejected": "stale_evidence"}
        window = {o[3]: o for o in obs if t_end - c.window <= o[0] <= t_end}.values()
        n = len(window)
        mean = sum(o[1] for o in window) / n if n else 0.0
        ok = n >= c.min_observations and mean >= c.min_mean_confidence and len({o[2] for o in window}) >= c.min_distinct_observers
        return ok, {"observations_in_window": n, "mean_confidence": mean, "observers": sorted({o[2] for o in window})}

    def ingest_remote(self, obs: AgentObservation, target_id: str, now: float) -> list[tuple[str, str]] | None:
        """A peer's OBSERVATION, already associated by the peer: attach it by ID. None if the ID is unknown."""
        rec = self.record(target_id)
        if rec is None or rec.terminal:
            return None
        self.seen_obs.add(obs.obs_id)
        self._attach(rec, obs)
        return self._merge_duplicates(rec.id)

    def apply_merge(self, survivor: str, alias: str) -> list[tuple[str, str]]:
        """A peer merged two records. Returns the (survivor, alias) renames applied locally."""
        s, a = self.resolve(survivor), self.resolve(alias)
        if s == a:
            return []
        keep, drop = sorted([s, a])
        if keep in self.records and drop in self.records:
            if self.records[keep].terminal or self.records[drop].terminal:
                return []
            self._merge(keep, drop)
            return [(keep, drop)]
        if drop in self.records:  # we only know the record under the losing ID: rename it
            rec = self.records.pop(drop)
            rec.id = keep
            rec.aliases.add(drop)
            self.records[keep] = rec
            self.alias_of[drop] = keep
            for al in rec.aliases:
                if al != keep:
                    self.alias_of[al] = keep
            return [(keep, drop)]
        self.alias_of[drop] = keep
        return []

    def add_cue(self, cue_id: str, pos, now: float) -> tuple[TargetRecord, TransitionEvent]:
        """External cue (7.1, 7.9): an unconfirmed record that is never evidence by itself."""
        rec, ev = new_record(cue_id, "external_cue", now, "EXT", (float(pos[0]), float(pos[1])), 0.0)
        self.records[cue_id] = rec
        return rec, ev


@dataclass
class RemoteUpdate:
    target_id: str | None = None
    created: str | None = None
    transition: TransitionEvent | None = None
    merges: list[tuple[str, str]] = field(default_factory=list)
    events: list[TrackerEvent] = field(default_factory=list)
