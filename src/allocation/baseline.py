"""Centralized optimum, for EVALUATION ONLY (PLAN.md section 8.7).

Runs after a scenario has finished, never during it. Given every drone's true
state at the moment allocation started, it evaluates the same cost function
with the same planner and returns the minimum-cost feasible drone, using the
same tie-break. It is used to report how often the decentralized result
matches the optimum.
"""

from __future__ import annotations

from src.allocation.cost import Bid, BidderState, CostConfig, better, evaluate
from src.config import Point2
from src.safety.planner import Planner


def optimal(bidders: list[BidderState], target_id: str, target_pos: Point2, planner: Planner,
            cfg: CostConfig) -> tuple[str | None, list[Bid]]:  # fmt: skip
    bids = [evaluate(b, target_id, target_pos, planner, cfg) for b in sorted(bidders, key=lambda b: b.id)]
    winner: str | None = None
    best: float | None = None
    for b in bids:
        if b.feasible and better(b.value, b.drone, best, winner):
            winner, best = b.drone, b.value
    return winner, bids
