from dataclasses import replace

import numpy as np
import pytest

from src.comms.messages import Message, Outbox
from src.comms.network import CommNetwork
from src.config import NetworkConfig, Partition
from src.logging_.event_log import EventLog

POS = {"D1": (0.0, 0.0, 60.0), "D2": (500.0, 0.0, 60.0), "D3": (2000.0, 0.0, 60.0)}


def net(**kw):
    cfg = replace(NetworkConfig(packet_loss=0.0), **kw)
    return CommNetwork(cfg, np.random.default_rng(0), {d: 1500.0 for d in POS}, EventLog())


def deliver_all(n, until=5.0, failed=frozenset()):
    inbox = {d: [] for d in POS}
    t = 0.0
    while t < until:
        t = round(t + 0.05, 9)
        n.deliver(t, inbox, set(failed))
    return inbox


def msg(seq=1, sender="D1", mtype="HEARTBEAT", t=0.0):
    return Message(mtype, sender, seq, t, {})


def test_message_identity_and_types():
    ob = Outbox("D1")
    a, b = ob.send("HEARTBEAT", 0.0), ob.send("BID", 0.1, {"x": 1}, target_id="T", epoch=0)
    assert (a.key, b.key) == (("D1", 1), ("D1", 2))
    assert ob.drain() == [a, b] and ob.drain() == []
    with pytest.raises(ValueError):
        Message("GOSSIP", "D1", 1, 0.0)


def test_range_limits_recipients():
    n = net()
    n.send(msg(), 0.0, POS, set())
    inbox = deliver_all(n)
    assert len(inbox["D2"]) == 1 and inbox["D3"] == []
    drops = n.log.of_type("MSG_DROPPED")
    assert [(d["data"]["to"], d["data"]["reason"]) for d in drops] == [("D3", "out_of_range")]


def test_latency_bounds_and_reordering():
    n = net(latency=(0.1, 1.0))
    for seq in range(1, 41):
        n.send(msg(seq), 0.0, POS, set())
    inbox = deliver_all(n)
    lat = [e["data"]["latency"] for e in n.log.of_type("MSG_DELIVERED")]
    assert len(lat) == 40 and all(0.1 <= x <= 1.0 for x in lat)
    seqs = [m.seq for m in inbox["D2"]]
    assert sorted(seqs) == list(range(1, 41)) and seqs != sorted(seqs)  # arrived out of order


def test_packet_loss_rate():
    n = net(packet_loss=0.2)
    for seq in range(1, 2001):
        n.send(msg(seq), 0.0, {"D1": POS["D1"], "D2": POS["D2"]}, set())
    lost = sum(1 for e in n.log.of_type("MSG_DROPPED") if e["data"]["reason"] == "packet_loss")
    assert lost / 2000 == pytest.approx(0.2, abs=0.03)


def test_partition_blocks_only_during_interval():
    n = net(partitions=[Partition(10.0, 20.0, [["D1"], ["D2", "D3"]])])
    assert n.in_partition("D1", "D2", 15.0) and not n.in_partition("D2", "D3", 15.0)
    assert not n.in_partition("D1", "D2", 9.9) and not n.in_partition("D1", "D2", 20.0)
    n.send(msg(1, t=15.0), 15.0, POS, set())
    assert any(e["data"]["reason"] == "partition" for e in n.log.of_type("MSG_DROPPED"))


def test_failed_drones_neither_send_nor_receive():
    n = net()
    n.send(msg(1, sender="D1"), 0.0, POS, {"D1"})
    assert n.stats["sent"] == 0
    n.send(msg(2, sender="D2"), 0.0, POS, set())
    inbox = deliver_all(n, failed={"D1"})  # D1 failed while the message was in flight
    assert inbox["D1"] == []
    assert any(e["data"]["reason"] == "recipient_failed" for e in n.log.of_type("MSG_DROPPED"))


def test_same_seed_same_outcomes():
    def outcomes():
        n = net(packet_loss=0.3, latency=(0.05, 0.5))
        for seq in range(1, 50):
            n.send(msg(seq), 0.0, POS, set())
        deliver_all(n)
        return n.log.text()

    assert outcomes() == outcomes()


def test_every_send_delivery_and_drop_is_logged():
    n = net(packet_loss=0.5)
    for seq in range(1, 21):
        n.send(msg(seq), 0.0, POS, set())
    deliver_all(n)
    sent = len(n.log.of_type("MSG_SENT"))
    delivered = len(n.log.of_type("MSG_DELIVERED"))
    dropped = len(n.log.of_type("MSG_DROPPED"))
    assert sent == 20 and delivered + dropped == 40  # 2 possible recipients per message
