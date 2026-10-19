"""Messaging, peer table and task allocation for a drone agent (PLAN.md sections 8 and 10).

Mixed into `Drone`. Everything here works only from the drone's own state, its
inbox and static shared configuration. Information about other drones and
about targets arrives only as messages; what the drone learns, it re-broadcasts
once, so information travels multi-hop when the radio range is short.

Sharing:
- HEARTBEAT every heartbeat.period -> peer table (alive = heard within miss_limit periods)
- TARGET_DETECTED when a record is created (own sensor, or first heard from a peer)
- OBSERVATION for own sensor observations, at most obs_share_hz per target
- TARGET_STATE when a record becomes CONFIRMED, with its evidence
- MERGE when duplicate records are merged
- BID / CLAIM / ACK for allocation (auction.py)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from src.allocation.auction import Action, Auction
from src.allocation.cost import BidderState, CostConfig, evaluate
from src.comms.messages import Message, Outbox
from src.safety.monitor import PeerTrack
from src.tracking.observation import AgentObservation, validate
from src.tracking.target_state import Confirmation, IllegalTransition, TargetState, TransitionContext, apply_transition

if TYPE_CHECKING:  # pragma: no cover
    from src.drone.agent import Drone


@dataclass
class PeerInfo:
    """Last heartbeat from a peer."""

    pos: tuple[float, float, float]
    vel: tuple[float, float, float]
    battery_pct: float
    mode: str
    current_task: str | None
    docked: bool
    t_sent: float
    t_received: float


class CoordinationMixin:
    def _init_coordination(self: "Drone") -> None:
        sh = self.shared
        self.outbox = Outbox(self.id)
        self.inbox: list[Message] = []
        self.peers: dict[str, PeerInfo] = {}
        self._seen_msgs: set[tuple[str, int]] = set()
        self._last_heartbeat: float | None = None
        self._last_share: dict[str, float] = {}
        self._announced: set[str] = set()  # targets we already sent TARGET_DETECTED for
        self._state_announced: set[tuple[str, str]] = set()  # (target, state) TARGET_STATE already sent
        self.auction = Auction(self.id, sh.allocation)
        self._cost_cfg = CostConfig(sh.allocation.weights, sh.inspection, sh.confidence)
        self._yielding_to: str | None = None

    # ------------------------------------------------------------------ peers

    def alive_peers(self: "Drone", now: float) -> set[str]:
        limit = self.shared.heartbeat.miss_limit * self.shared.heartbeat.period
        return {p for p, info in self.peers.items() if now - info.t_received <= limit}

    def peer_tracks(self: "Drone", now: float) -> list[PeerTrack]:
        """Alive, airborne peers extrapolated from their last heartbeat (for the separation check)."""
        out = []
        for pid in sorted(self.alive_peers(now)):
            info = self.peers[pid]
            if info.docked or info.pos[2] <= 0.0 or info.mode in ("LANDED", "FAILED"):
                continue
            age = now - info.t_sent
            pos = tuple(p + v * age for p, v in zip(info.pos, info.vel))
            out.append(PeerTrack(pid, pos, info.vel, info.current_task is not None))
        return out

    def _heartbeat(self: "Drone", now: float) -> None:
        period = self.shared.heartbeat.period
        if self._last_heartbeat is not None and now - self._last_heartbeat < period - 1e-9:
            return
        self._last_heartbeat = now
        s = self.state
        self.outbox.send("HEARTBEAT", now, {
            "pos": [s.x, s.y, s.alt], "vel": [s.vx, s.vy, s.vz], "battery_pct": self.battery.pct,
            "mode": s.mode.value, "current_task": s.current_task, "epoch": s.task_epoch, "docked": self.docked,
        })  # fmt: skip

    # ------------------------------------------------------------------ inbox

    def _read_inbox(self: "Drone", now: float) -> None:
        msgs, self.inbox = self.inbox, []
        for msg in msgs:
            if msg.key in self._seen_msgs:
                continue  # duplicate
            self._seen_msgs.add(msg.key)
            if now - msg.t_sent > self.shared.max_message_age:
                continue  # stale
            handler = getattr(self, f"_on_{msg.type.lower()}", None)
            if handler is not None:
                handler(msg, now)

    def _on_heartbeat(self: "Drone", msg: Message, now: float) -> None:
        p = msg.payload
        self.peers[msg.sender] = PeerInfo(tuple(p["pos"]), tuple(p["vel"]), p["battery_pct"], p["mode"],
                                          p.get("current_task"), bool(p.get("docked")), msg.t_sent, now)  # fmt: skip

    def _on_target_detected(self: "Drone", msg: Message, now: float) -> None:
        self._apply_summary(msg.payload["summary"], msg.sender, now)

    def _on_target_state(self: "Drone", msg: Message, now: float) -> None:
        self._apply_summary(msg.payload["summary"], msg.sender, now)

    def _on_observation(self: "Drone", msg: Message, now: float) -> None:
        tid = self._apply_summary(msg.payload["summary"], msg.sender, now)
        o = msg.payload["observation"]
        obs = AgentObservation(o["obs_id"], o["observer"], "relayed", o["t"], tuple(o["pos"]), o["alt"], o["confidence"])
        ok, _ = validate(obs, now, self.fence, self.shared.observation, self._known_observers, self.tracker.seen_obs)
        if ok and tid is not None:
            merges = self.tracker.ingest_remote(obs, tid, now)
            self._handle_merges(now, merges or [])

    def _on_merge(self: "Drone", msg: Message, now: float) -> None:
        self._handle_merges(now, self.tracker.apply_merge(msg.payload["survivor"], msg.payload["alias"]))

    def _on_bid(self: "Drone", msg: Message, now: float) -> None:
        tid = self._apply_summary(msg.payload["summary"], msg.sender, now)
        if tid is None:
            return
        self._maybe_start_auction(now, tid)
        self._run_actions(now, self.auction.on_bid(tid, msg.epoch or 0, msg.sender, msg.payload, now))

    def _on_claim(self: "Drone", msg: Message, now: float) -> None:
        self._on_commit_msg(msg, now, is_claim=True)

    def _on_ack(self: "Drone", msg: Message, now: float) -> None:
        self._on_commit_msg(msg, now, is_claim=False)

    def _on_commit_msg(self: "Drone", msg: Message, now: float, is_claim: bool) -> None:
        tid = self._apply_summary(msg.payload["summary"], msg.sender, now)
        if tid is None:
            return
        self._maybe_start_auction(now, tid)
        p = msg.payload
        self._run_actions(now, self.auction.on_commit_msg(tid, msg.epoch or 0, msg.sender, p.get("winner_id"),
                                                          p.get("winning_bid"), is_claim, now))  # fmt: skip

    def _on_release(self: "Drone", msg: Message, now: float) -> None:
        tid = self.tracker.resolve(msg.target_id or "")
        self._emit(now, "CLAIM_CONFLICT" if msg.payload.get("reason") == "conflict_lost" else "REALLOCATION",
                   {"from": msg.sender, "reason": msg.payload.get("reason"), "note": "release received"},
                   target_id=tid, epoch=msg.epoch)  # fmt: skip

    # ------------------------------------------------------------------ records from peers

    def _apply_summary(self: "Drone", summ: dict, sender: str, now: float) -> str | None:
        upd = self.tracker.apply_summary(summ, sender, now, self._known_observers)
        if upd.created and upd.transition is not None:
            self._emit(now, "TARGET_CREATED", {"source": summ["source"], "pos": summ["est_pos"], "relayed_from": sender},
                       target_id=upd.created, epoch=0)  # fmt: skip
            self._emit(now, "TARGET_TRANSITION", {"from": "UNKNOWN", "to": "DETECTED", "row": 1,
                                                  "reason": upd.transition.reason, "by": self.id},
                       target_id=upd.created, epoch=0)  # fmt: skip
            self._announce_target(now, upd.created)  # relay once
        self._on_tracker_events(now, upd.events)
        self._handle_merges(now, upd.merges)
        return upd.target_id

    def _announce_target(self: "Drone", now: float, tid: str) -> None:
        tid = self.tracker.resolve(tid)
        if tid in self._announced:
            return
        self._announced.add(tid)
        self.outbox.send("TARGET_DETECTED", now, {"summary": self.tracker.summary(tid)}, target_id=tid, epoch=0)

    def _on_tracker_events(self: "Drone", now: float, events: list) -> None:
        for ev in events:
            self._emit(now, ev.type, ev.data, target_id=ev.target_id, epoch=ev.epoch)
            if ev.type == "TARGET_TRANSITION" and ev.data["to"] == "CONFIRMED":
                self._announce_state(now, ev.target_id, ev.data)

    def _announce_state(self: "Drone", now: float, tid: str, data: dict) -> None:
        tid = self.tracker.resolve(tid)
        key = (tid, data["to"])
        if key in self._state_announced or self.tracker.record(tid) is None:
            return
        self._state_announced.add(key)
        rec = self.tracker.record(tid)
        self.outbox.send("TARGET_STATE", now, {
            "from": data["from"], "to": data["to"], "reason": data.get("reason"),
            "summary": self.tracker.summary(tid, with_evidence=True),
        }, target_id=tid, epoch=rec.epoch)  # fmt: skip

    def _share_observation(self: "Drone", now: float, tid: str, obs: AgentObservation) -> None:
        last = self._last_share.get(tid)
        if last is not None and now - last < 1.0 / self.shared.observation.obs_share_hz - 1e-9:
            return
        self._last_share[tid] = now
        self.outbox.send("OBSERVATION", now, {
            "summary": self.tracker.summary(tid),
            "observation": {"obs_id": obs.obs_id, "observer": obs.observer, "t": obs.t, "pos": list(obs.pos),
                            "alt": obs.alt, "confidence": obs.confidence},
        }, target_id=tid, epoch=0)  # fmt: skip

    def _handle_merges(self: "Drone", now: float, merges: list[tuple[str, str]]) -> None:
        for survivor, alias in merges:
            self._emit(now, "TARGET_MERGED", {"survivor": survivor, "alias": alias}, target_id=survivor)
            self.auction.merge(survivor, alias)
            if self.state.current_task == alias:
                self.state.current_task = survivor
            if self.hold_target == alias:
                self.hold_target = survivor
            if alias in self._held:
                self._held.add(survivor)
            if alias in self._announced:
                self._announced.add(survivor)
            self.outbox.send("MERGE", now, {"survivor": survivor, "alias": alias}, target_id=survivor)

    # ------------------------------------------------------------------ allocation

    def _allocatable(self: "Drone", tid: str) -> bool:
        rec = self.tracker.record(tid)
        if rec is None or rec.terminal:
            return False
        return rec.state is TargetState.CONFIRMED or (rec.state is TargetState.DETECTED and rec.is_cue)

    def _maybe_start_auction(self: "Drone", now: float, tid: str) -> None:
        tid = self.tracker.resolve(tid)
        rec = self.tracker.record(tid)
        if rec is None or not self._allocatable(tid):
            return
        e = self.auction.entry(tid)
        if e is not None and e.epoch >= rec.epoch:
            return
        s = self.state
        bidder = BidderState(self.id, s.pos3, self.battery.charge_wh, s.params, s.home, s.mode.value, self.docked,
                             busy=s.current_task is not None and s.current_task != tid)  # fmt: skip
        bid = evaluate(bidder, tid, rec.est_pos, self.planner, self._cost_cfg, excluded=rec.excluded.get(rec.epoch) == self.id)
        self._emit(now, "BID_COMPUTED", {
            "feasible": bid.feasible, "bid": bid.value, "reason": bid.reason, "target_est": list(rec.est_pos),
            "breakdown": {k: v for k, v in bid.breakdown.items() if k != "route_go"},
        }, target_id=tid, epoch=rec.epoch)  # fmt: skip
        self._run_actions(now, self.auction.start(tid, rec.epoch, now, bid))

    def _allocation_step(self: "Drone", now: float) -> None:
        for tid in sorted(self.tracker.records):
            self._maybe_start_auction(now, tid)
        self._run_actions(now, self.auction.tick(now, self.alive_peers(now)))

    def _run_actions(self: "Drone", now: float, actions: list[Action]) -> None:
        for a in actions:
            getattr(self, f"_act_{a.kind}")(now, a)

    def _bid_payload(self: "Drone", tid: str) -> dict[str, Any]:
        e = self.auction.entry(tid)
        assert e is not None
        return {
            "winner_id": e.winner, "winning_bid": e.winning_bid, "own_bid": e.own.value, "feasible": e.own.feasible,
            "reason": e.own.reason, "committed": e.committed,
            "breakdown": {k: v for k, v in e.own.breakdown.items() if k != "route_go"},
            "summary": self.tracker.summary(tid, with_evidence=True),
        }  # fmt: skip

    def _act_broadcast(self: "Drone", now: float, a: Action) -> None:
        e = self.auction.entry(a.target_id)
        self.outbox.send("BID", now, self._bid_payload(a.target_id), target_id=a.target_id, epoch=e.epoch)

    def _send_commit(self: "Drone", now: float, tid: str) -> None:
        e = self.auction.entry(tid)
        mtype = "CLAIM" if e.winner == self.id else "ACK"
        self.outbox.send(mtype, now, {"winner_id": e.winner, "winning_bid": e.winning_bid,
                                      "summary": self.tracker.summary(tid, with_evidence=True)},
                         target_id=tid, epoch=e.epoch)  # fmt: skip

    def _act_commit(self: "Drone", now: float, a: Action) -> None:
        tid = a.target_id
        rec = self.tracker.record(tid)
        e = self.auction.entry(tid)
        feasible = sum(1 for b in e.bids.values() if b.feasible)
        self._emit(now, "ALLOCATION_COMMITTED", {
            "winner": a.winner, "winning_bid": e.winning_bid, "how": a.data["how"], "rounds": a.data["rounds"],
            "heard": a.data["heard"], "feasible_bids": feasible,
            "bids": {d: {"feasible": b.feasible, "bid": b.value, "reason": b.reason} for d, b in sorted(e.bids.items())},
        }, target_id=tid, epoch=e.epoch)  # fmt: skip
        if rec is not None and not rec.terminal and rec.state in (TargetState.CONFIRMED, TargetState.DETECTED):
            ctx = TransitionContext(actor=self.id, now=now, winner=a.winner, feasible_bids=max(feasible, 1))
            try:
                ev = apply_transition(rec, TargetState.ALLOCATED, ctx, f"consensus:{a.data['how']}")
                self._emit(now, "TARGET_TRANSITION", {"from": ev.frm.value, "to": ev.to.value, "row": ev.row,
                                                      "reason": ev.reason, "by": self.id, "owner": a.winner},
                           target_id=tid, epoch=ev.epoch)  # fmt: skip
            except IllegalTransition as exc:
                self._emit(now, "ILLEGAL_TRANSITION", {"to": "ALLOCATED", "error": str(exc)}, target_id=tid, epoch=e.epoch)
        if a.winner == self.id:
            self.state.current_task = tid
            self.state.task_epoch = e.epoch
        self._send_commit(now, tid)

    def _act_resend_commit(self: "Drone", now: float, a: Action) -> None:
        self._send_commit(now, a.target_id)

    def _act_no_bidders(self: "Drone", now: float, a: Action) -> None:
        tid = a.target_id
        rec = self.tracker.record(tid)
        e = self.auction.entry(tid)
        self._emit(now, "ALLOCATION_FAILED", {
            "reason": "no_feasible_bidders",
            "bids": {d: {"feasible": b.feasible, "reason": b.reason} for d, b in sorted(e.bids.items())},
        }, target_id=tid, epoch=e.epoch)  # fmt: skip
        if rec is not None and not rec.terminal:
            ctx = TransitionContext(actor=self.id, now=now, trigger="allocation_timeout")
            try:
                ev = apply_transition(rec, TargetState.CANCELLED, ctx, "no_feasible_bidders")
                self._emit(now, "TARGET_TRANSITION", {"from": ev.frm.value, "to": ev.to.value, "row": ev.row,
                                                      "reason": ev.reason, "by": self.id}, target_id=tid, epoch=ev.epoch)  # fmt: skip
                self._emit(now, "TASK_CANCELLED", {"reason": "no_feasible_bidders"}, target_id=tid, epoch=ev.epoch)
            except IllegalTransition as exc:
                self._emit(now, "ILLEGAL_TRANSITION", {"to": "CANCELLED", "error": str(exc)}, target_id=tid, epoch=e.epoch)

    def _act_conflict_lost(self: "Drone", now: float, a: Action) -> None:
        tid = a.target_id
        e = self.auction.entry(tid)
        self._emit(now, "CLAIM_CONFLICT", {"result": "lost", "winner": a.winner}, target_id=tid, epoch=e.epoch)
        if self.state.current_task == tid:
            self.state.current_task = None
        rec = self.tracker.record(tid)
        if rec is not None:
            rec.owner = a.winner
        self.outbox.send("RELEASE", now, {"reason": "conflict_lost", "winner_id": a.winner}, target_id=tid, epoch=e.epoch)

    def _act_owner_changed(self: "Drone", now: float, a: Action) -> None:
        e = self.auction.entry(a.target_id)
        self._emit(now, "CLAIM_CONFLICT", {"result": "owner_changed", "winner": a.winner, **a.data},
                   target_id=a.target_id, epoch=e.epoch)  # fmt: skip
        rec = self.tracker.record(a.target_id)
        if rec is not None:
            rec.owner = a.winner

    def _act_conflict_won(self: "Drone", now: float, a: Action) -> None:
        e = self.auction.entry(a.target_id)
        self._emit(now, "CLAIM_CONFLICT", {"result": "won", **a.data}, target_id=a.target_id, epoch=e.epoch)
        self._send_commit(now, a.target_id)  # tell the challenger it lost

    # ------------------------------------------------------------------ external cue (scenario injection)

    def inject_cue(self: "Drone", now: float, cue_id: str, pos) -> None:
        if self.tracker.record(cue_id) is not None:
            return
        rec, ev = self.tracker.add_cue(cue_id, pos, now)
        self._emit(now, "TARGET_CREATED", {"source": "external_cue", "pos": list(rec.est_pos)}, target_id=cue_id, epoch=0)
        self._emit(now, "TARGET_TRANSITION", {"from": "UNKNOWN", "to": "DETECTED", "row": 1, "reason": ev.reason, "by": self.id},
                   target_id=cue_id, epoch=0)  # fmt: skip
        self._handle_merges(now, self.tracker._merge_duplicates(cue_id))
        assert rec.confirmation is Confirmation.PENDING  # a cue is never evidence by itself
