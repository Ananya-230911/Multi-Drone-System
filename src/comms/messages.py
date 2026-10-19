"""Message types exchanged between drones (PLAN.md section 10.1).

Every message carries `sender`, `seq` and `t_sent`, plus `target_id` / `epoch`
where relevant. The payload is a plain dict so it can be logged, and so the
same structures can move unchanged to another transport later (section 17).
Receivers treat payloads as read-only.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

MSG_TYPES = (
    "HEARTBEAT",  # pos, alt, vel, battery_pct, mode, current_task, epoch
    "TARGET_DETECTED",  # target summary (est_pos, alt, source, ...)
    "OBSERVATION",  # one observation, without ground-truth flags, plus the target summary
    "MERGE",  # survivor_id, alias_id
    "TARGET_STATE",  # from, to, reason, target summary (+ evidence when confirming)
    "BID",  # winner_id, winning_bid, own_bid, feasible, reason, breakdown, target summary
    "CLAIM",  # the committed winner announces ownership
    "ACK",  # a non-winner acknowledges the committed winner
    "RELEASE",  # an owner gives up the task
    "TARGET_LOGGED",  # (Phase 4)
)


@dataclass(frozen=True)
class Message:
    type: str
    sender: str
    seq: int
    t_sent: float
    payload: dict[str, Any] = field(default_factory=dict)
    target_id: str | None = None
    epoch: int | None = None

    def __post_init__(self) -> None:
        if self.type not in MSG_TYPES:
            raise ValueError(f"unknown message type {self.type!r}")

    @property
    def key(self) -> tuple[str, int]:
        """Message identity: receivers drop a (sender, seq) they have already seen."""
        return (self.sender, self.seq)


class Outbox:
    """Per-drone message factory: numbers messages and queues them for the network."""

    def __init__(self, sender: str):
        self.sender = sender
        self._seq = 0
        self.queue: list[Message] = []

    def send(self, mtype: str, now: float, payload: dict[str, Any] | None = None,
             target_id: str | None = None, epoch: int | None = None) -> Message:  # fmt: skip
        self._seq += 1
        msg = Message(mtype, self.sender, self._seq, now, payload or {}, target_id, epoch)
        self.queue.append(msg)
        return msg

    def drain(self) -> list[Message]:
        out, self.queue = self.queue, []
        return out
