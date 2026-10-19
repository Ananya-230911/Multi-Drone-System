"""Decentralized single-task auction with max-consensus (PLAN.md sections 8.3 to 8.5).

Each agent keeps, per target, a winner table entry (winner_id, winning_bid, epoch).

1. Start: when the local record becomes allocatable, the agent computes its own bid
   and writes itself in if feasible and better than the table.
2. Broadcast: BID {winner_id, winning_bid, own_bid, feasible, ...} every round_period.
3. Compare: higher epoch wins; same epoch -> higher bid; equal bid -> lower ID.
   A change triggers an immediate re-broadcast (this is the multi-hop relay).
4. Commit: table unchanged for `consensus_rounds` rounds AND a BID (or CLAIM/ACK)
   for this epoch heard from every peer believed alive. After `allocation_timeout`,
   commit to the table if it has a feasible winner, else NO_BIDDERS.
5. The winner sends CLAIM, the others ACK.
6. After commit, same-epoch bids are ignored, except conflicting CLAIMs: both sides
   apply the compare rule and the loser releases.

This class holds only the agent's own tables. It returns "actions" for the agent
to turn into messages and lifecycle transitions; it never touches the network.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from src.allocation.cost import Bid, better
from src.config import AllocationConfig


@dataclass
class BidInfo:
    drone: str
    feasible: bool
    value: float | None
    reason: str
    breakdown: dict[str, Any]
    t: float


@dataclass
class AuctionEntry:
    target_id: str
    epoch: int
    started_at: float
    own: Bid
    winner: str | None = None
    winning_bid: float | None = None
    bids: dict[str, BidInfo] = field(default_factory=dict)  # last own_bid heard from each drone (incl. self)
    heard: set[str] = field(default_factory=set)  # peers heard from in this epoch (BID, CLAIM or ACK)
    stable_rounds: int = 0
    changed: bool = False  # table changed since the last round
    next_round: float = 0.0
    committed: bool = False
    commit_t: float | None = None
    status: str = "BIDDING"  # BIDDING | COMMITTED | NO_BIDDERS
    last_resend: float = -1e9  # last time CLAIM/ACK was re-sent to a peer that is behind


@dataclass(frozen=True)
class Action:
    kind: str  # "broadcast" | "commit" | "no_bidders" | "resend_commit" | "conflict_lost" | "conflict_won"
    target_id: str
    winner: str | None = None
    data: dict[str, Any] = field(default_factory=dict)


class Auction:
    def __init__(self, me: str, cfg: AllocationConfig):
        self.me = me
        self.cfg = cfg
        self.entries: dict[str, AuctionEntry] = {}

    # ------------------------------------------------------------------ start

    def start(self, target_id: str, epoch: int, now: float, own: Bid) -> list[Action]:
        e = AuctionEntry(target_id, epoch, now, own, next_round=now + self.cfg.round_period)
        e.bids[self.me] = BidInfo(self.me, own.feasible, own.value, own.reason, own.breakdown, now)
        if own.feasible:
            e.winner, e.winning_bid = self.me, own.value
        self.entries[target_id] = e
        return [Action("broadcast", target_id)]

    def entry(self, target_id: str) -> AuctionEntry | None:
        return self.entries.get(target_id)

    # ------------------------------------------------------------------ incoming

    def on_bid(self, target_id: str, epoch: int, sender: str, payload: dict[str, Any], now: float) -> list[Action]:
        e = self.entries.get(target_id)
        if e is None or epoch < e.epoch:
            return []  # unknown (agent starts its own auction first) or stale epoch
        if epoch > e.epoch:
            return []  # higher epochs are handled by the agent's reopen logic (Phase 4/5)
        e.heard.add(sender)
        e.bids[sender] = BidInfo(sender, bool(payload.get("feasible")), payload.get("own_bid"),
                                 payload.get("reason", ""), payload.get("breakdown", {}), now)  # fmt: skip
        if e.committed:
            # The sender is behind: repeat our CLAIM / ACK at most once per round.
            if not payload.get("committed") and now - e.last_resend >= self.cfg.round_period:
                e.last_resend = now
                return [Action("resend_commit", target_id, e.winner)]
            return []
        return self._consider(e, payload.get("winner_id"), payload.get("winning_bid"))

    def on_commit_msg(self, target_id: str, epoch: int, sender: str, winner: str | None, winning_bid: float | None,
                      is_claim: bool, now: float) -> list[Action]:  # fmt: skip
        """A peer's CLAIM (it won) or ACK (it acknowledges `winner`)."""
        e = self.entries.get(target_id)
        if e is None or epoch != e.epoch:
            return []
        e.heard.add(sender)
        if not e.committed:
            return self._consider(e, winner, winning_bid)
        if not is_claim or winner == e.winner:
            return []
        # Conflicting CLAIM after commit (8.5): both sides apply the compare rule.
        if better(winning_bid, winner, e.winning_bid, e.winner):
            lost = e.winner == self.me
            old = e.winner
            e.winner, e.winning_bid = winner, winning_bid
            return [Action("conflict_lost" if lost else "owner_changed", target_id, winner, {"previous": old})]
        if e.winner == self.me:
            return [Action("conflict_won", target_id, self.me, {"challenger": sender})]
        return []

    def _consider(self, e: AuctionEntry, winner: str | None, winning_bid: float | None) -> list[Action]:
        if better(winning_bid, winner, e.winning_bid, e.winner):
            e.winner, e.winning_bid = winner, winning_bid
            e.changed = True
            return [Action("broadcast", e.target_id)]  # immediate re-broadcast: multi-hop relay
        return []

    # ------------------------------------------------------------------ rounds

    def tick(self, now: float, alive_peers: set[str]) -> list[Action]:
        actions: list[Action] = []
        for tid in sorted(self.entries):
            e = self.entries[tid]
            if e.committed or e.status == "NO_BIDDERS":
                continue
            if now + 1e-9 < e.next_round:
                continue
            e.next_round += self.cfg.round_period
            e.stable_rounds = 0 if e.changed else e.stable_rounds + 1
            e.changed = False
            heard_all = alive_peers <= e.heard
            if e.winner is not None and e.stable_rounds >= self.cfg.consensus_rounds and heard_all:
                actions.append(self._commit(e, now, "consensus"))
            elif now - e.started_at >= self.cfg.allocation_timeout:
                if e.winner is not None:
                    actions.append(self._commit(e, now, "timeout"))
                else:
                    e.status = "NO_BIDDERS"
                    actions.append(Action("no_bidders", tid))
            else:
                actions.append(Action("broadcast", tid))
        return actions

    def _commit(self, e: AuctionEntry, now: float, how: str) -> Action:
        e.committed, e.commit_t, e.status = True, now, "COMMITTED"
        return Action("commit", e.target_id, e.winner, {"how": how, "rounds": e.stable_rounds, "heard": sorted(e.heard)})

    # ------------------------------------------------------------------ duplicates

    def merge(self, survivor: str, alias: str) -> None:
        """Duplicate records merged: fold the alias table into the survivor's with the compare rule."""
        a = self.entries.pop(alias, None)
        if a is None:
            return
        s = self.entries.get(survivor)
        if s is None:
            a.target_id = survivor
            self.entries[survivor] = a
            return
        if a.epoch > s.epoch:
            a.target_id = survivor
            self.entries[survivor] = a
            return
        if a.epoch < s.epoch:
            return
        s.heard |= a.heard
        for k, v in a.bids.items():
            s.bids.setdefault(k, v)
        if better(a.winning_bid, a.winner, s.winning_bid, s.winner):
            s.winner, s.winning_bid, s.changed = a.winner, a.winning_bid, True
        s.committed = s.committed or a.committed
        if s.committed:
            s.status = "COMMITTED"
