"""Auction / consensus without the world: a toy radio with random latency, reordering and loss."""

import heapq

import numpy as np
import pytest

from src.allocation.auction import Auction
from src.allocation.cost import Bid
from src.config import AllocationConfig

CFG = AllocationConfig()  # round_period 0.5, consensus_rounds 3, allocation_timeout 10


def own_bid(drone, value):
    return Bid(drone, "T", value is not None, value, "ok" if value is not None else "insufficient_energy", {})


def run(bids: dict[str, float | None], neighbours: dict[str, set[str]], seed=0, loss=0.0, latency=(0.05, 0.2),
        start_times=None, until=30.0):  # fmt: skip
    """Each agent: an Auction. Messages go only to neighbours, with loss and random latency."""
    rng = np.random.default_rng(seed)
    agents = {d: Auction(d, CFG) for d in bids}
    start_times = start_times or {d: 0.0 for d in bids}
    queue, order, commits, started = [], 0, {}, set()
    dt, t = 0.05, 0.0

    def send(sender, payload, kind):
        nonlocal order
        for r in sorted(neighbours[sender]):
            if rng.random() < loss:
                continue
            order += 1
            heapq.heappush(queue, (t + rng.uniform(*latency), order, r, sender, kind, payload))

    def payload(d):
        e = agents[d].entry("T")
        return {"winner_id": e.winner, "winning_bid": e.winning_bid, "own_bid": e.own.value,
                "feasible": e.own.feasible, "committed": e.committed}  # fmt: skip

    def handle(d, actions):
        for a in actions:
            if a.kind == "broadcast":
                send(d, payload(d), "BID")
            elif a.kind in ("commit", "resend_commit"):
                if a.kind == "commit":
                    commits[d] = a.winner
                send(d, payload(d), "CLAIM" if a.winner == d else "ACK")

    while t < until:
        t = round(t + dt, 9)
        for d in sorted(bids):
            if d not in started and t >= start_times[d]:
                started.add(d)
                handle(d, agents[d].start("T", 0, t, own_bid(d, bids[d])))
        while queue and queue[0][0] <= t:
            _, _, r, s, kind, p = heapq.heappop(queue)
            if r not in started:
                continue
            if kind == "BID":
                handle(r, agents[r].on_bid("T", 0, s, p, t))
            else:
                handle(r, agents[r].on_commit_msg("T", 0, s, p["winner_id"], p["winning_bid"], kind == "CLAIM", t))
        for d in sorted(started):
            handle(d, agents[d].tick(t, neighbours[d]))
    return commits, agents


def full(ids):
    return {d: set(ids) - {d} for d in ids}


def expected_winner(bids):
    feasible = [(v, d) for d, v in bids.items() if v is not None]
    return max(feasible, key=lambda x: (x[0], [-ord(c) for c in x[1]]))[1] if feasible else None


def test_all_agree_on_highest_bid():
    bids = {"D1": -50.0, "D2": -10.0, "D3": -30.0, "D4": None}
    commits, _ = run(bids, full(bids))
    assert commits == {d: "D2" for d in bids}


@pytest.mark.parametrize("seed", range(25))
def test_agreement_with_random_bids_latency_reordering_and_20pct_loss(seed):
    rng = np.random.default_rng(1000 + seed)
    ids = ["D1", "D2", "D3", "D4"]
    bids = {d: float(-rng.uniform(0, 100)) for d in ids}
    commits, _ = run(bids, full(ids), seed=seed, loss=0.2, latency=(0.05, 0.6))
    assert set(commits) == set(ids)
    assert set(commits.values()) == {max(bids, key=bids.get)}


def test_line_topology_agrees_through_relay():
    # D1 - D2 - D3 - D4: D4 can only learn about D1's winning bid through D2 and D3 re-broadcasting.
    bids = {"D1": -1.0, "D2": -20.0, "D3": -30.0, "D4": -40.0}
    nb = {"D1": {"D2"}, "D2": {"D1", "D3"}, "D3": {"D2", "D4"}, "D4": {"D3"}}
    commits, agents = run(bids, nb, loss=0.05)
    assert commits == {d: "D1" for d in bids}
    assert "D1" not in agents["D4"].entry("T").heard  # never heard D1 directly


def test_equal_bids_lower_id_wins():
    bids = {"D3": -7.0, "D1": -7.0, "D2": -9.0}
    commits, _ = run(bids, full(bids))
    assert set(commits.values()) == {"D1"}


def test_no_feasible_bidders_times_out():
    bids = {"D1": None, "D2": None}
    agents = {d: Auction(d, CFG) for d in bids}
    acts = []
    for d in bids:
        agents[d].start("T", 0, 0.0, own_bid(d, None))
    t = 0.0
    while t < CFG.allocation_timeout + 1:
        t = round(t + 0.5, 9)
        for d in bids:
            acts += [(d, a.kind) for a in agents[d].tick(t, set())]
    assert ("D1", "no_bidders") in acts and ("D2", "no_bidders") in acts
    assert all(agents[d].entry("T").status == "NO_BIDDERS" for d in bids)


def test_commit_waits_for_bids_from_all_alive_peers():
    a = Auction("D1", CFG)
    a.start("T", 0, 0.0, own_bid("D1", -1.0))
    kinds = [k.kind for t in (0.5, 1.0, 1.5, 2.0, 2.5) for k in a.tick(t, {"D2"})]
    assert "commit" not in kinds  # stable, but D2 (alive) has not bid yet
    a.on_bid("T", 0, "D2", {"winner_id": "D1", "winning_bid": -1.0, "own_bid": -5.0, "feasible": True}, 2.6)
    assert [k.kind for k in a.tick(3.0, {"D2"})] == ["commit"]


def test_late_agent_starting_after_commit_learns_winner():
    bids = {"D1": -5.0, "D2": -1.0, "D3": -9.0}
    commits, _ = run(bids, full(bids), start_times={"D1": 0.0, "D2": 0.0, "D3": 4.0})
    assert commits == {d: "D2" for d in bids}


def test_stale_epoch_bids_are_ignored():
    a = Auction("D1", CFG)
    a.start("T", 1, 0.0, own_bid("D1", -10.0))
    assert a.on_bid("T", 0, "D2", {"winner_id": "D2", "winning_bid": -1.0, "own_bid": -1.0, "feasible": True}, 0.1) == []
    e = a.entry("T")
    assert e.winner == "D1" and "D2" not in e.heard


def test_conflicting_claims_after_partition_resolve_to_better_bid():
    a, b = Auction("D1", CFG), Auction("D2", CFG)
    a.start("T", 0, 0.0, own_bid("D1", -3.0))
    b.start("T", 0, 0.0, own_bid("D2", -8.0))
    for t in (0.5, 1.0, 1.5, 2.0):  # partitioned: each commits itself
        a.tick(t, set())
        b.tick(t, set())
    assert a.entry("T").committed and b.entry("T").committed
    lost = b.on_commit_msg("T", 0, "D1", "D1", -3.0, True, 3.0)
    won = a.on_commit_msg("T", 0, "D2", "D2", -8.0, True, 3.0)
    assert [x.kind for x in lost] == ["conflict_lost"] and b.entry("T").winner == "D1"
    assert [x.kind for x in won] == ["conflict_won"] and a.entry("T").winner == "D1"


def test_merge_folds_duplicate_auction_tables():
    a = Auction("D1", CFG)
    a.start("T-D1-001", 0, 0.0, own_bid("D1", -10.0))
    a.start("T-D3-001", 0, 0.0, own_bid("D1", -10.0))
    a.on_bid("T-D3-001", 0, "D3", {"winner_id": "D3", "winning_bid": -2.0, "own_bid": -2.0, "feasible": True}, 0.1)
    a.merge("T-D1-001", "T-D3-001")
    assert list(a.entries) == ["T-D1-001"]
    e = a.entry("T-D1-001")
    assert e.winner == "D3" and "D3" in e.heard
