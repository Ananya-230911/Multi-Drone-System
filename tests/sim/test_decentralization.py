"""Decentralization rules (PLAN.md 4.1): agents hold no references to other agents, the world or
ground truth, and everything they know about peers and targets arrives through their inbox."""

import dataclasses
from pathlib import Path

import pytest

from src.comms.messages import Message
from src.comms.network import CommNetwork
from src.config import load_config
from src.drone.agent import Drone
from src.drone.sensor import Sensor
from src.sim.ground_truth import GroundTruth, GroundTruthTarget
from src.sim.scenario import Scenario
from src.sim.world import World

ROOT = Path(__file__).resolve().parents[2]
FORBIDDEN = (Drone, World, GroundTruth, GroundTruthTarget, CommNetwork, Sensor, Scenario)


def cfg(name="default", radio=True, **sim):
    c = load_config(ROOT / "config" / f"{name}.yaml")
    c = dataclasses.replace(c, sim=dataclasses.replace(c.sim, **sim))
    if not radio:
        c = dataclasses.replace(c, network=dataclasses.replace(c.network, comm_range=1.0))
    return c


def walk(obj, depth=0, seen=None):
    """Yield every object reachable from `obj` through attributes and containers (bounded depth)."""
    seen = seen if seen is not None else set()
    if id(obj) in seen or depth > 4:
        return
    seen.add(id(obj))
    yield obj
    if isinstance(obj, (str, bytes, int, float, bool, type(None))):
        return
    if isinstance(obj, dict):
        children = list(obj.keys()) + list(obj.values())
    elif isinstance(obj, (list, tuple, set, frozenset)):
        children = list(obj)
    elif hasattr(obj, "__dict__"):
        children = list(vars(obj).values())
    else:
        children = []
    for c in children:
        yield from walk(c, depth + 1, seen)


@pytest.fixture(scope="module")
def world():
    w = World(cfg(duration=100))
    w.run()
    return w


def test_agents_hold_no_references_to_other_agents_world_or_ground_truth(world):
    for did, agent in world.drones.items():
        for obj in walk(agent):
            if obj is agent:
                continue
            assert not isinstance(obj, FORBIDDEN), (did, type(obj).__name__)


def test_peer_table_entries_come_from_delivered_heartbeats(world):
    delivered = {(e["actor"], e["data"]["from"]) for e in world.log.of_type("MSG_DELIVERED")
                 if e["data"]["msg_type"] == "HEARTBEAT"}  # fmt: skip
    for did, agent in world.drones.items():
        for peer in agent.peers:
            assert (did, peer) in delivered


def test_bids_from_peers_come_from_delivered_messages(world):
    delivered = {(e["actor"], e["data"]["from"]) for e in world.log.of_type("MSG_DELIVERED")
                 if e["data"]["msg_type"] in ("BID", "CLAIM", "ACK")}  # fmt: skip
    for did, agent in world.drones.items():
        for entry in agent.auction.entries.values():
            for peer in entry.bids:
                assert peer == did or (did, peer) in delivered


def test_without_radio_agents_know_nothing_about_each_other():
    w = World(cfg(radio=False, duration=100))
    w.run()
    for did, agent in w.drones.items():
        assert agent.peers == {}
        for rec in agent.tracker.records.values():
            assert not rec.relayed and rec.created_by == did
        for entry in agent.auction.entries.values():
            assert set(entry.bids) == {did}
    assert {d: len(a.tracker.records) for d, a in w.drones.items()} == {"D1": 0, "D2": 1, "D3": 0, "D4": 0}


def test_duplicate_and_stale_messages_are_dropped():
    w = World(cfg(duration=1))
    d1 = w.drones["D1"]
    hb = {"pos": [500.0, 500.0, 60.0], "vel": [0.0, 0.0, 0.0], "battery_pct": 0.5, "mode": "PATROL",
          "current_task": None, "epoch": 0, "docked": False}  # fmt: skip
    first = Message("HEARTBEAT", "D2", 7, 1.0, hb)
    d1.inbox = [first]
    d1._read_inbox(1.1)
    assert d1.peers["D2"].t_received == 1.1
    d1.inbox = [Message("HEARTBEAT", "D2", 7, 1.0, {**hb, "battery_pct": 0.1})]  # same (sender, seq)
    d1._read_inbox(1.2)
    assert d1.peers["D2"].battery_pct == 0.5 and d1.peers["D2"].t_received == 1.1
    d1.inbox = [Message("HEARTBEAT", "D3", 1, 0.0, hb)]  # older than max_message_age (5 s)
    d1._read_inbox(6.0)
    assert "D3" not in d1.peers


def test_ground_truth_ids_never_reach_agents(world):
    for agent in world.drones.values():
        for tid in agent.tracker.records:
            assert not tid.startswith("GT-")
        for obj in walk(agent):
            if isinstance(obj, str):
                assert not obj.startswith("GT-")
