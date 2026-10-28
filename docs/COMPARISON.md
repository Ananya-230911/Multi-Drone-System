# Comparison with other approaches

## A. Measured: our auction vs other ways of choosing the drone (same simulator, same cost function)

Run `python -m src.compare` (writes `output/compare/summary.md`, `methods.png`, `cost_per_scenario.png`).
Every allocation in 9 scenarios x 10 seeds (129 allocations) is scored with the same cost
(travel time + energy + remaining battery, lower is better).

| Method | Picks a drone that can finish (%) | Picks the best drone (%) | Mean extra cost vs best |
|---|---|---|---|
| **Ours (decentralized auction)** | 100 | 100 | 0.0 |
| Centralized optimum (sees everything) | 100 | 100 | 0.0 |
| Nearest drone | 84.5 | 69.8 | 0.61 |
| Most battery | 100 | 49.6 | 19.06 |
| Random drone | 90.3 | n/a | 34.74 |

Reading it: with no central controller and lossy radio, the drones reach the same choice as the all-knowing
optimum every time (note: where the radio is cut in two, `partition`, they cannot, which is excluded here and
reported in the Phase 5 evaluation). The popular shortcut "send the nearest drone" is wrong 30% of the time and in
15% of cases sends a drone that cannot finish (battery below the 20% reserve or no safe route).
These are our own simulations, not a field test.

## B. Literature: 3 real, published approaches (qualitative)

Different papers use different problems, robots and metrics, so **their numbers cannot be put on one graph with
ours**. The honest comparison is what each method is, and what ours adds.

| | **CBBA** (Choi, Brunet, How, IEEE T-RO 2009) | **MURDOCH** (Gerkey & Matarić, IEEE T-RA 2002) | **Gossip-based auction for drone swarms** (Drones, 2024) | **This project** |
|---|---|---|---|---|
| Idea | Auction + consensus on bids; agents bid on bundles of tasks | Sequence of one-round first-price auctions | Auction with gossip for rescheduling in heterogeneous drone swarms | Auction + max-consensus on a single task, CLAIM/ACK with epochs |
| Central controller | No | No (auctioneer per task) | No | No |
| Handles owner failure | Re-auction after conflicts | Re-auction on timeout | Rescheduling | Heartbeat timeout reopens the auction at epoch + 1 |
| Network split / heal | Consensus on local comms | Not the focus | Gossip | Duplicate owner detected after heal, weaker claim releases |
| Safety layer (geofence, routes, separation) | Not part of the algorithm | Not part of the algorithm | Not part of the algorithm | Hard filter: geofence, validated route, battery reserve |
| Quality guarantee | Published bound for single-assignment CBAA: within 50% of optimal (from a thesis summary of the work) | Empirical, real robots | Reports about 2% to 4% better benefit than CBBA in its own 16 UAV / 80 task simulation | Matches the centralized optimum in 129 of 129 measured allocations (our scenarios) |

What this project adds: the allocation sits inside a complete mission loop (detection, confirmation, lifecycle,
logging, simulated neutralization), with a safety layer that can veto a bid, and an AI crew that sets up scenarios.
What it does not claim: it handles one task at a time (CBBA handles many tasks per vehicle), and it is a simulation.

### Sources
- Choi, Brunet, How, "Consensus-Based Decentralized Auctions for Robust Task Allocation", IEEE T-RO 25(4), 2009: https://dspace.mit.edu/handle/1721.1/52330
- Gerkey & Matarić, MURDOCH auction-based coordination: https://robotics.usc.edu/~gerkey/research/murdoch.html
- Gerkey & Matarić, "A Formal Analysis and Taxonomy of Task Allocation in Multi-Robot Systems", IJRR 23(9), 2004: https://www.citedrive.com/en/discovery/a-formal-analysis-and-taxonomy-of-task-allocation-in-multi-robot-systems
- "A Gossip-Based Auction Algorithm for Decentralized Task Rescheduling in Heterogeneous Drone Swarms": https://avesis.ankara.edu.tr/yayin/d6f11675-dd64-43aa-98a5-e07d7d6f9e56/a-gossip-based-auction-algorithm-for-decentralized-task-rescheduling-in-heterogeneous-drone-swarms
- Drones (Dec 2024) swarm task allocation vs CBBA: https://doaj.org/article/c1e5184c9e6241e3ad7c071141d621b0

(Verify each row against the paper before the viva: the entries for the gossip-based paper come from search summaries, not a full read.)
