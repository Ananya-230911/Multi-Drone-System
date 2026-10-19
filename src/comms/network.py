"""Simulated radio (PLAN.md section 10.2).

Every message is a broadcast. Per recipient:
- range: only drones within min(network.comm_range, sender comm_range) at send time;
- partitions: scheduled groups that cannot talk to each other (drones not listed
  in any group can talk to everyone);
- packet loss: independent drop with probability `packet_loss`;
- latency: uniform in [min, max] seconds, so messages can arrive reordered.
FAILED drones neither send nor receive. Every send, delivery and drop is logged.

There is no relaying in the network: multi-hop propagation happens because
agents re-broadcast what they learn (section 8.3, rule 3).
"""

from __future__ import annotations

import heapq
import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from src.comms.messages import Message
from src.config import NetworkConfig, Point3
from src.logging_.event_log import EventLog

HISTORY = 400  # recent message outcomes kept for the dashboard


@dataclass(order=True)
class _InFlight:
    deliver_at: float
    order: int
    recipient: str = field(compare=False)
    msg: Message = field(compare=False)
    latency: float = field(compare=False)


@dataclass(frozen=True)
class MessageRecord:
    """One message outcome for one recipient, for the dashboard message log and flashes."""

    t: float
    type: str
    sender: str
    recipient: str
    status: str  # DELIVERED, or the drop reason
    target_id: str | None = None


class CommNetwork:
    def __init__(self, cfg: NetworkConfig, rng: np.random.Generator, comm_ranges: dict[str, float], log: EventLog):
        self.cfg = cfg
        self.rng = rng
        self.comm_ranges = dict(comm_ranges)
        self.log = log
        self._queue: list[_InFlight] = []
        self._order = 0
        self.history: deque[MessageRecord] = deque(maxlen=HISTORY)
        self.stats = {"sent": 0, "delivered": 0, "dropped": 0}
        self.sent_by_type: dict[str, int] = {}

    def in_partition(self, a: str, b: str, now: float) -> bool:
        for part in self.cfg.partitions:
            if part.start <= now < part.end:
                ga = next((i for i, g in enumerate(part.groups) if a in g), None)
                gb = next((i for i, g in enumerate(part.groups) if b in g), None)
                if ga is not None and gb is not None and ga != gb:
                    return True
        return False

    def send(self, msg: Message, now: float, positions: dict[str, Point3], failed: set[str]) -> None:
        if msg.sender in failed:
            return
        self.stats["sent"] += 1
        self.sent_by_type[msg.type] = self.sent_by_type.get(msg.type, 0) + 1
        rng_m = min(self.cfg.comm_range, self.comm_ranges.get(msg.sender, self.cfg.comm_range))
        src = positions[msg.sender]
        recipients = []
        for rid in sorted(positions):
            if rid == msg.sender:
                continue
            reason = None
            if rid in failed:
                reason = "recipient_failed"
            elif math.dist(src, positions[rid]) > rng_m:
                reason = "out_of_range"
            elif self.in_partition(msg.sender, rid, now):
                reason = "partition"
            elif self.rng.random() < self.cfg.packet_loss:
                reason = "packet_loss"
            if reason:
                self._drop(now, msg, rid, reason)
                continue
            lo, hi = self.cfg.latency
            lat = float(self.rng.uniform(lo, hi)) if hi > lo else lo
            self._order += 1
            heapq.heappush(self._queue, _InFlight(now + lat, self._order, rid, msg, lat))
            recipients.append(rid)
        data = {"msg_type": msg.type, "seq": msg.seq, "recipients": recipients}
        self.log.append(now, "MSG_SENT", msg.sender, data, target_id=msg.target_id, epoch=msg.epoch)

    def deliver(self, now: float, inboxes: dict[str, list[Message]], failed: set[str]) -> None:
        while self._queue and self._queue[0].deliver_at <= now + 1e-9:
            item = heapq.heappop(self._queue)
            if item.recipient in failed:
                self._drop(now, item.msg, item.recipient, "recipient_failed")
                continue
            inboxes[item.recipient].append(item.msg)
            self.stats["delivered"] += 1
            self.log.append(now, "MSG_DELIVERED", item.recipient,
                            {"msg_type": item.msg.type, "seq": item.msg.seq, "from": item.msg.sender,
                             "latency": item.latency},
                            target_id=item.msg.target_id, epoch=item.msg.epoch)  # fmt: skip
            self.history.append(MessageRecord(now, item.msg.type, item.msg.sender, item.recipient, "DELIVERED", item.msg.target_id))

    def pending(self) -> int:
        return len(self._queue)

    def _drop(self, now: float, msg: Message, rid: str, reason: str) -> None:
        self.stats["dropped"] += 1
        self.log.append(now, "MSG_DROPPED", msg.sender, {"msg_type": msg.type, "seq": msg.seq, "to": rid, "reason": reason},
                        target_id=msg.target_id, epoch=msg.epoch)  # fmt: skip
        self.history.append(MessageRecord(now, msg.type, msg.sender, rid, reason, msg.target_id))
