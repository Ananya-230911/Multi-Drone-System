# Before / after: Multi-Drone-System on the benchmark mission

Benchmark: `benchmark_comparison/harness/run_mds.py` and `common.py` (unchanged; only the repository path variable was
pointed at this repository), 1.9 km x 1.9 km, 8 targets, inspect-only, seeds 1-10 and 11-20, 420 runs per version.
**Before** = commit `0061d77`, **after** = commit `af5bf1e` (this work). No parameter was tuned on the seeds: the new
defaults (`max_queue` 4, `retry_period` 10 s) were chosen before the first run and never changed. The causes were diagnosed on runs
of seeds 1-10 (seed 3 of the 2-drone failure run led to two later fixes); seeds 11-20 were only run as part of the full sweeps. Both seed
ranges are reported separately below, and the numbers come from a complete re-run of all 420 runs on the final commit.

## Scorecard against the targets

| Measure | Target | Before -> after | Result |
|---|---|---|---|
| 4 drones, every 20 s: task completion, seeds 1-10 | >= 95 % | 61.2 % -> 96.2 % | met |
| 4 drones, every 20 s: missions fully completed, seeds 1-10 | >= 8 / 10 | 0/10 -> 7/10 (counting only targets outside the no-fly zone: 10/10) | NOT met |
| 4 drones, all targets at start: task completion, seeds 1-10 | >= 90 % | 45.0 % -> 96.2 % | met |
| 4 drones, every 20 s: task completion, seeds 11-20 | >= 95 % | 58.8 % -> 95.0 % | met |
| 4 drones, every 20 s: missions fully completed, seeds 11-20 | >= 8 / 10 | 0/10 -> 7/10 (counting only targets outside the no-fly zone: 10/10) | NOT met |
| 4 drones, all targets at start: task completion, seeds 11-20 | >= 90 % | 43.8 % -> 93.8 % | met |
| geofence violations, all 420 runs | 0 | 0 -> 0 | met |
| separation violations, 4 drones, all runs | no increase | 22 -> 2 | met |
| separation violations, all configurations, all runs | no increase | 36 -> 10 | met |
| separation violations, 2 drones only, all runs | no increase | 0 -> 4 | NOT met |
| completion at 30 % packet loss vs 0 %, 4 drones dynamic, seeds 1-10 | within 5 points | 96.2 % vs 96.2 % (difference 0.0) | met |
| completion at 30 % packet loss vs 0 %, 4 drones dynamic, seeds 11-20 | within 5 points | 95.0 % vs 95.0 % (difference 0.0) | met |
| double-assignment races (second win while the first task is open and neither queued nor given back), 240 replays | 0 | 81 runs -> 0 runs | met |

### Read this before the numbers

* **The harness world contains a no-fly zone it did not ask for.** `run_mds.py` gives the simulator no `restricted_zones`,
  so the simulator's default zone RZ-1 (800-1100 m x 600-1000 m, in the middle of the map) applies. Of the 160 target
  placements in seeds 1-20, 9 lie inside it or on its edge (seeds 2, 6, 7, 12, 14, 16, 17 and 19; seed 17 has two), and a
  drone may not enter a no-fly zone, so those targets cannot be inspected by design. That is why "missions fully
  completed" stops at 7 / 10 on seeds 1-10 and cannot reach 8 / 10 without changing the benchmark or the geofence. The
  harness is not changed here. Columns marked "feasible" in the tables count only the targets outside the zone: with
  that, every configuration completes 100 % of the feasible targets on all 20 seeds, including 30 % packet loss and one
  drone failing.
* "Task completion" and "missions fully completed" in the scorecard are the benchmark's own definitions (all 8 targets).
* **Separation violations** (true distance below 25 m) fell from 36 to 10 over all runs and from
  22 to 2 with 4 drones, but with 2 drones they went from
  0 to 4 (seed 10: two targets lie about 25 m apart, two drones inspect them at the same time and one passes at 24.4 m).
  In the failure runs of seeds 14 and 20 the violation is with D1, the drone that failed and stays frozen in the air (seed 14: D3 passes it at 24.0 m at t = 114 s).
* **Not done: item 6 (adaptive heartbeats).** Heartbeats are about two thirds of all messages. Making them adaptive
  needs the heartbeat period in the message (a format change) and risks the owner-timeout detection and the separation
  look-ahead, so it was left alone. Messages per completed target fell anyway (4 drones, every 20 s: 560 -> 214)
  because fewer retransmissions and re-auctions are needed, but they are still well above the 34-77 of the other two projects.
* **One remaining way to get stuck.** A drone sent to a target on the edge of a no-fly zone can hover at the nearest allowed
  point, where the sensor confidence stays below the inspection threshold, and neither complete nor time out
  (`1 of 240 replays` end with a drone still holding such a task; before the fix: `9`). It only occurs for targets that cannot be
  completed anyway.
* Double-assignment replay (240 runs: 4 fleet sizes x 20 seeds x dynamic / dynamic with failure / all targets at start): the benchmark's own
  definition (a second win while the first task is not yet logged, even if the first was cancelled) flags
  85 runs before and 236 after; but before, those wins overwrote the drone's task (321 events in 81 runs, once a cancelled first task is not counted), whereas after the fix all 870 second wins were queued or given back and none overwrote a task (0 events). Drones left holding a task that is never logged at the end of a run: 9 runs before, 1 after.

## Full tables



### Scalability, targets every 20 s and all at start (seeds 1-10)

| config | | runs | completion % | completion % of feasible targets | missions fully done (95% CI) | missions done, feasible targets only | dist / optimum (ok runs) | makespan @15 m/s (ok runs, s) | alloc latency s | msgs / done target | geofence viol. | separation viol. | duplicate insp. | runs not finished |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dynamic 2 drones | before | 10 | 31.2 | 32.3 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 425 | 0 | 0 | 0 | 10 |
| dynamic 2 drones | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 2.4 | 317 | 1.50 | 143 | 0 | 1 | 0 | 3 |
| dynamic 4 drones | before | 10 | 61.2 | 63.8 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 542 | 0 | 2 | 0 | 10 |
| dynamic 4 drones | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 3.8 | 243 | 1.50 | 207 | 0 | 0 | 0 | 3 |
| dynamic 6 drones | before | 10 | 83.8 | 87.1 | 1/10 (2-40%) | 2/10 | 5.9 | 220 | 1.50 | 407 | 0 | 0 | 0 | 9 |
| dynamic 6 drones | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 5.4 | 231 | 1.50 | 289 | 0 | 0 | 0 | 3 |
| dynamic 8 drones | before | 10 | 93.8 | 97.3 | 6/10 (31-83%) | 8/10 | 7.8 | 241 | 1.50 | 496 | 0 | 3 | 0 | 4 |
| dynamic 8 drones | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 7.4 | 231 | 1.50 | 383 | 0 | 0 | 0 | 3 |
| static 2 drones | before | 10 | 23.8 | 24.6 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 322 | 0 | 0 | 0 | 10 |
| static 2 drones | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 2.2 | 286 | 1.50 | 135 | 0 | 1 | 0 | 3 |
| static 4 drones | before | 10 | 45.0 | 46.8 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 505 | 0 | 0 | 0 | 10 |
| static 4 drones | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 3.8 | 243 | 1.50 | 209 | 0 | 0 | 0 | 3 |
| static 6 drones | before | 10 | 57.5 | 59.8 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 427 | 0 | 0 | 0 | 10 |
| static 6 drones | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 5.1 | 220 | 1.50 | 277 | 0 | 0 | 0 | 3 |
| static 8 drones | before | 10 | 67.5 | 70.2 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 710 | 0 | 0 | 0 | 10 |
| static 8 drones | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 5.0 | 160 | 1.50 | 328 | 0 | 0 | 0 | 3 |

### Scalability, targets every 20 s and all at start (seeds 11-20)

| config | | runs | completion % | completion % of feasible targets | missions fully done (95% CI) | missions done, feasible targets only | dist / optimum (ok runs) | makespan @15 m/s (ok runs, s) | alloc latency s | msgs / done target | geofence viol. | separation viol. | duplicate insp. | runs not finished |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dynamic 2 drones | before | 10 | 35.0 | 37.8 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 452 | 0 | 0 | 0 | 10 |
| dynamic 2 drones | after | 10 | 93.8 | 100.0 | 6/10 (31-83%) | 10/10 | 2.0 | 290 | 1.50 | 175 | 0 | 0 | 0 | 4 |
| dynamic 4 drones | before | 10 | 58.8 | 63.6 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 579 | 0 | 1 | 0 | 10 |
| dynamic 4 drones | after | 10 | 95.0 | 100.0 | 7/10 (40-89%) | 10/10 | 3.9 | 256 | 1.50 | 222 | 0 | 0 | 0 | 3 |
| dynamic 6 drones | before | 10 | 86.2 | 93.6 | 2/10 (6-51%) | 6/10 | 6.3 | 258 | 1.50 | 577 | 0 | 1 | 0 | 8 |
| dynamic 6 drones | after | 10 | 95.0 | 100.0 | 6/10 (31-83%) | 10/10 | 6.0 | 256 | 1.50 | 326 | 0 | 0 | 0 | 4 |
| dynamic 8 drones | before | 10 | 92.5 | 98.8 | 5/10 (24-76%) | 9/10 | 8.5 | 252 | 1.50 | 619 | 0 | 1 | 0 | 5 |
| dynamic 8 drones | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 8.1 | 256 | 1.50 | 414 | 0 | 0 | 0 | 3 |
| static 2 drones | before | 10 | 23.8 | 25.8 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 334 | 0 | 0 | 0 | 10 |
| static 2 drones | after | 10 | 93.8 | 100.0 | 6/10 (31-83%) | 10/10 | 1.9 | 266 | 1.50 | 143 | 0 | 0 | 0 | 4 |
| static 4 drones | before | 10 | 43.8 | 47.5 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 539 | 0 | 0 | 0 | 10 |
| static 4 drones | after | 10 | 93.8 | 100.0 | 6/10 (31-83%) | 10/10 | 3.7 | 241 | 1.50 | 233 | 0 | 0 | 0 | 4 |
| static 6 drones | before | 10 | 51.2 | 55.5 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 707 | 0 | 0 | 0 | 10 |
| static 6 drones | after | 10 | 95.0 | 100.0 | 6/10 (31-83%) | 10/10 | 5.0 | 212 | 1.50 | 289 | 0 | 0 | 0 | 4 |
| static 8 drones | before | 10 | 62.5 | 67.9 | 0/10 (0-28%) | 1/10 | - | - | 1.50 | 1055 | 0 | 0 | 0 | 10 |
| static 8 drones | after | 10 | 95.0 | 100.0 | 6/10 (31-83%) | 10/10 | 5.3 | 170 | 1.50 | 357 | 0 | 0 | 0 | 4 |

### Scalability, targets every 20 s and all at start (seeds 1-20)

| config | | runs | completion % | completion % of feasible targets | missions fully done (95% CI) | missions done, feasible targets only | dist / optimum (ok runs) | makespan @15 m/s (ok runs, s) | alloc latency s | msgs / done target | geofence viol. | separation viol. | duplicate insp. | runs not finished |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dynamic 2 drones | before | 20 | 33.1 | 35.1 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 438 | 0 | 0 | 0 | 20 |
| dynamic 2 drones | after | 20 | 95.0 | 100.0 | 13/20 (43-82%) | 20/20 | 2.2 | 304 | 1.50 | 159 | 0 | 1 | 0 | 7 |
| dynamic 4 drones | before | 20 | 60.0 | 63.7 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 560 | 0 | 3 | 0 | 20 |
| dynamic 4 drones | after | 20 | 95.6 | 100.0 | 14/20 (48-85%) | 20/20 | 3.9 | 249 | 1.50 | 214 | 0 | 0 | 0 | 6 |
| dynamic 6 drones | before | 20 | 85.0 | 90.4 | 3/20 (5-36%) | 8/20 | 6.2 | 245 | 1.50 | 492 | 0 | 1 | 0 | 17 |
| dynamic 6 drones | after | 20 | 95.6 | 100.0 | 13/20 (43-82%) | 20/20 | 5.7 | 243 | 1.50 | 308 | 0 | 0 | 0 | 7 |
| dynamic 8 drones | before | 20 | 93.1 | 98.0 | 11/20 (34-74%) | 17/20 | 8.1 | 246 | 1.50 | 557 | 0 | 4 | 0 | 9 |
| dynamic 8 drones | after | 20 | 96.2 | 100.0 | 14/20 (48-85%) | 20/20 | 7.8 | 243 | 1.50 | 398 | 0 | 0 | 0 | 6 |
| static 2 drones | before | 20 | 23.8 | 25.2 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 328 | 0 | 0 | 0 | 20 |
| static 2 drones | after | 20 | 95.0 | 100.0 | 13/20 (43-82%) | 20/20 | 2.1 | 277 | 1.50 | 139 | 0 | 1 | 0 | 7 |
| static 4 drones | before | 20 | 44.4 | 47.1 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 522 | 0 | 0 | 0 | 20 |
| static 4 drones | after | 20 | 95.0 | 100.0 | 13/20 (43-82%) | 20/20 | 3.8 | 242 | 1.50 | 221 | 0 | 0 | 0 | 7 |
| static 6 drones | before | 20 | 54.4 | 57.7 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 567 | 0 | 0 | 0 | 20 |
| static 6 drones | after | 20 | 95.6 | 100.0 | 13/20 (43-82%) | 20/20 | 5.1 | 216 | 1.50 | 283 | 0 | 0 | 0 | 7 |
| static 8 drones | before | 20 | 65.0 | 69.0 | 0/20 (0-16%) | 1/20 | - | - | 1.50 | 882 | 0 | 0 | 0 | 20 |
| static 8 drones | after | 20 | 95.6 | 100.0 | 13/20 (43-82%) | 20/20 | 5.2 | 164 | 1.50 | 343 | 0 | 0 | 0 | 7 |

### Packet loss, 4 drones (seeds 1-10)

| config | | runs | completion % | completion % of feasible targets | missions fully done (95% CI) | missions done, feasible targets only | dist / optimum (ok runs) | makespan @15 m/s (ok runs, s) | alloc latency s | msgs / done target | geofence viol. | separation viol. | duplicate insp. | runs not finished |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dynamic loss 0% | before | 10 | 61.2 | 63.8 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 542 | 0 | 2 | 0 | 10 |
| dynamic loss 0% | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 3.8 | 243 | 1.50 | 207 | 0 | 0 | 0 | 3 |
| dynamic loss 10% | before | 10 | 61.2 | 63.8 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 542 | 0 | 2 | 0 | 10 |
| dynamic loss 10% | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 3.8 | 243 | 1.50 | 207 | 0 | 0 | 0 | 3 |
| dynamic loss 20% | before | 10 | 61.2 | 63.8 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 544 | 0 | 2 | 3 | 10 |
| dynamic loss 20% | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 3.8 | 243 | 1.50 | 207 | 0 | 0 | 0 | 3 |
| dynamic loss 30% | before | 10 | 61.2 | 63.8 | 0/10 (0-28%) | 0/10 | - | - | 1.52 | 543 | 0 | 2 | 2 | 10 |
| dynamic loss 30% | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 3.7 | 236 | 1.50 | 205 | 0 | 0 | 0 | 3 |
| static loss 0% | before | 10 | 45.0 | 46.8 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 505 | 0 | 0 | 0 | 10 |
| static loss 0% | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 3.8 | 243 | 1.50 | 209 | 0 | 0 | 0 | 3 |
| static loss 10% | before | 10 | 45.0 | 46.8 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 504 | 0 | 0 | 0 | 10 |
| static loss 10% | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 3.8 | 243 | 1.50 | 210 | 0 | 0 | 0 | 3 |
| static loss 20% | before | 10 | 45.0 | 46.8 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 506 | 0 | 0 | 1 | 10 |
| static loss 20% | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 3.8 | 243 | 1.50 | 210 | 0 | 0 | 0 | 3 |
| static loss 30% | before | 10 | 42.5 | 43.9 | 0/10 (0-28%) | 0/10 | - | - | 1.51 | 743 | 0 | 0 | 2 | 10 |
| static loss 30% | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 3.8 | 243 | 1.51 | 209 | 0 | 0 | 0 | 3 |

### Packet loss, 4 drones (seeds 11-20)

| config | | runs | completion % | completion % of feasible targets | missions fully done (95% CI) | missions done, feasible targets only | dist / optimum (ok runs) | makespan @15 m/s (ok runs, s) | alloc latency s | msgs / done target | geofence viol. | separation viol. | duplicate insp. | runs not finished |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dynamic loss 0% | before | 10 | 58.8 | 63.6 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 579 | 0 | 1 | 0 | 10 |
| dynamic loss 0% | after | 10 | 95.0 | 100.0 | 7/10 (40-89%) | 10/10 | 3.9 | 256 | 1.50 | 222 | 0 | 0 | 0 | 3 |
| dynamic loss 10% | before | 10 | 58.8 | 63.6 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 581 | 0 | 1 | 0 | 10 |
| dynamic loss 10% | after | 10 | 95.0 | 100.0 | 7/10 (40-89%) | 10/10 | 3.9 | 256 | 1.50 | 222 | 0 | 0 | 0 | 3 |
| dynamic loss 20% | before | 10 | 57.5 | 62.4 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 661 | 0 | 1 | 1 | 10 |
| dynamic loss 20% | after | 10 | 95.0 | 100.0 | 7/10 (40-89%) | 10/10 | 3.9 | 256 | 1.50 | 222 | 0 | 0 | 0 | 3 |
| dynamic loss 30% | before | 10 | 57.5 | 62.4 | 0/10 (0-28%) | 0/10 | - | - | 1.52 | 661 | 0 | 2 | 4 | 10 |
| dynamic loss 30% | after | 10 | 95.0 | 100.0 | 7/10 (40-89%) | 10/10 | 3.9 | 256 | 1.51 | 222 | 0 | 0 | 0 | 3 |
| static loss 0% | before | 10 | 43.8 | 47.5 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 539 | 0 | 0 | 0 | 10 |
| static loss 0% | after | 10 | 93.8 | 100.0 | 6/10 (31-83%) | 10/10 | 3.7 | 241 | 1.50 | 233 | 0 | 0 | 0 | 4 |
| static loss 10% | before | 10 | 43.8 | 47.5 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 538 | 0 | 0 | 0 | 10 |
| static loss 10% | after | 10 | 93.8 | 100.0 | 6/10 (31-83%) | 10/10 | 3.7 | 241 | 1.50 | 233 | 0 | 0 | 0 | 4 |
| static loss 20% | before | 10 | 43.8 | 47.5 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 540 | 0 | 0 | 1 | 10 |
| static loss 20% | after | 10 | 93.8 | 100.0 | 6/10 (31-83%) | 10/10 | 3.7 | 241 | 1.50 | 233 | 0 | 0 | 0 | 4 |
| static loss 30% | before | 10 | 42.5 | 46.2 | 0/10 (0-28%) | 0/10 | - | - | 1.51 | 734 | 0 | 0 | 4 | 10 |
| static loss 30% | after | 10 | 93.8 | 100.0 | 6/10 (31-83%) | 10/10 | 3.8 | 247 | 1.51 | 235 | 0 | 0 | 0 | 4 |

### Packet loss, 4 drones (seeds 1-20)

| config | | runs | completion % | completion % of feasible targets | missions fully done (95% CI) | missions done, feasible targets only | dist / optimum (ok runs) | makespan @15 m/s (ok runs, s) | alloc latency s | msgs / done target | geofence viol. | separation viol. | duplicate insp. | runs not finished |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dynamic loss 0% | before | 20 | 60.0 | 63.7 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 560 | 0 | 3 | 0 | 20 |
| dynamic loss 0% | after | 20 | 95.6 | 100.0 | 14/20 (48-85%) | 20/20 | 3.9 | 249 | 1.50 | 214 | 0 | 0 | 0 | 6 |
| dynamic loss 10% | before | 20 | 60.0 | 63.7 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 561 | 0 | 3 | 0 | 20 |
| dynamic loss 10% | after | 20 | 95.6 | 100.0 | 14/20 (48-85%) | 20/20 | 3.9 | 249 | 1.50 | 214 | 0 | 0 | 0 | 6 |
| dynamic loss 20% | before | 20 | 59.4 | 63.1 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 602 | 0 | 3 | 4 | 20 |
| dynamic loss 20% | after | 20 | 95.6 | 100.0 | 14/20 (48-85%) | 20/20 | 3.9 | 249 | 1.50 | 214 | 0 | 0 | 0 | 6 |
| dynamic loss 30% | before | 20 | 59.4 | 63.1 | 0/20 (0-16%) | 0/20 | - | - | 1.52 | 602 | 0 | 4 | 6 | 20 |
| dynamic loss 30% | after | 20 | 95.6 | 100.0 | 14/20 (48-85%) | 20/20 | 3.8 | 246 | 1.50 | 213 | 0 | 0 | 0 | 6 |
| static loss 0% | before | 20 | 44.4 | 47.1 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 522 | 0 | 0 | 0 | 20 |
| static loss 0% | after | 20 | 95.0 | 100.0 | 13/20 (43-82%) | 20/20 | 3.8 | 242 | 1.50 | 221 | 0 | 0 | 0 | 7 |
| static loss 10% | before | 20 | 44.4 | 47.1 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 521 | 0 | 0 | 0 | 20 |
| static loss 10% | after | 20 | 95.0 | 100.0 | 13/20 (43-82%) | 20/20 | 3.8 | 242 | 1.50 | 221 | 0 | 0 | 0 | 7 |
| static loss 20% | before | 20 | 44.4 | 47.1 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 523 | 0 | 0 | 2 | 20 |
| static loss 20% | after | 20 | 95.0 | 100.0 | 13/20 (43-82%) | 20/20 | 3.8 | 242 | 1.50 | 221 | 0 | 0 | 0 | 7 |
| static loss 30% | before | 20 | 42.5 | 45.1 | 0/20 (0-16%) | 0/20 | - | - | 1.51 | 739 | 0 | 0 | 6 | 20 |
| static loss 30% | after | 20 | 95.0 | 100.0 | 13/20 (43-82%) | 20/20 | 3.8 | 245 | 1.51 | 222 | 0 | 0 | 0 | 7 |

### One drone fails at t = 40 s (dynamic) (seeds 1-10)

| config | | runs | completion % | completion % of feasible targets | missions fully done (95% CI) | missions done, feasible targets only | dist / optimum (ok runs) | makespan @15 m/s (ok runs, s) | alloc latency s | msgs / done target | geofence viol. | separation viol. | duplicate insp. | runs not finished |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2 drones, D1 fails | before | 10 | 17.5 | 18.2 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 422 | 0 | 0 | 0 | 10 |
| 2 drones, D1 fails | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 2.2 | 521 | 18.00 | 176 | 0 | 0 | 0 | 3 |
| 4 drones, D1 fails | before | 10 | 48.8 | 50.7 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 550 | 0 | 4 | 0 | 10 |
| 4 drones, D1 fails | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 3.6 | 291 | 1.50 | 188 | 0 | 0 | 0 | 3 |
| 6 drones, D1 fails | before | 10 | 77.5 | 80.7 | 2/10 (6-51%) | 3/10 | 4.9 | 231 | 1.50 | 394 | 0 | 0 | 0 | 8 |
| 6 drones, D1 fails | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 5.1 | 253 | 1.50 | 270 | 0 | 0 | 0 | 3 |
| 8 drones, D1 fails | before | 10 | 91.2 | 94.6 | 5/10 (24-76%) | 7/10 | 7.2 | 249 | 1.50 | 496 | 0 | 2 | 0 | 5 |
| 8 drones, D1 fails | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 6.9 | 243 | 1.50 | 362 | 0 | 0 | 0 | 3 |

### One drone fails at t = 40 s (dynamic) (seeds 11-20)

| config | | runs | completion % | completion % of feasible targets | missions fully done (95% CI) | missions done, feasible targets only | dist / optimum (ok runs) | makespan @15 m/s (ok runs, s) | alloc latency s | msgs / done target | geofence viol. | separation viol. | duplicate insp. | runs not finished |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2 drones, D1 fails | before | 10 | 20.0 | 21.5 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 457 | 0 | 0 | 0 | 10 |
| 2 drones, D1 fails | after | 10 | 93.8 | 100.0 | 6/10 (31-83%) | 10/10 | 2.1 | 529 | 19.71 | 186 | 0 | 2 | 0 | 4 |
| 4 drones, D1 fails | before | 10 | 47.5 | 51.6 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 566 | 0 | 2 | 0 | 10 |
| 4 drones, D1 fails | after | 10 | 95.0 | 100.0 | 7/10 (40-89%) | 10/10 | 3.2 | 267 | 1.50 | 192 | 0 | 2 | 0 | 3 |
| 6 drones, D1 fails | before | 10 | 75.0 | 81.2 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 601 | 0 | 4 | 0 | 10 |
| 6 drones, D1 fails | after | 10 | 95.0 | 100.0 | 6/10 (31-83%) | 10/10 | 4.8 | 238 | 1.50 | 279 | 0 | 2 | 0 | 4 |
| 8 drones, D1 fails | before | 10 | 92.5 | 98.8 | 5/10 (24-76%) | 9/10 | 7.7 | 252 | 1.50 | 559 | 0 | 3 | 0 | 5 |
| 8 drones, D1 fails | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 6.8 | 240 | 1.50 | 365 | 0 | 2 | 0 | 3 |

### One drone fails at t = 40 s (dynamic) (seeds 1-20)

| config | | runs | completion % | completion % of feasible targets | missions fully done (95% CI) | missions done, feasible targets only | dist / optimum (ok runs) | makespan @15 m/s (ok runs, s) | alloc latency s | msgs / done target | geofence viol. | separation viol. | duplicate insp. | runs not finished |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2 drones, D1 fails | before | 20 | 18.8 | 19.9 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 439 | 0 | 0 | 0 | 20 |
| 2 drones, D1 fails | after | 20 | 95.0 | 100.0 | 13/20 (43-82%) | 20/20 | 2.2 | 525 | 18.86 | 181 | 0 | 2 | 0 | 7 |
| 4 drones, D1 fails | before | 20 | 48.1 | 51.2 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 558 | 0 | 6 | 0 | 20 |
| 4 drones, D1 fails | after | 20 | 95.6 | 100.0 | 14/20 (48-85%) | 20/20 | 3.4 | 279 | 1.50 | 190 | 0 | 2 | 0 | 6 |
| 6 drones, D1 fails | before | 20 | 76.2 | 81.0 | 2/20 (3-30%) | 3/20 | 4.9 | 231 | 1.50 | 498 | 0 | 4 | 0 | 18 |
| 6 drones, D1 fails | after | 20 | 95.6 | 100.0 | 13/20 (43-82%) | 20/20 | 5.0 | 246 | 1.50 | 274 | 0 | 2 | 0 | 7 |
| 8 drones, D1 fails | before | 20 | 91.9 | 96.7 | 10/20 (30-70%) | 16/20 | 7.4 | 251 | 1.50 | 528 | 0 | 5 | 0 | 10 |
| 8 drones, D1 fails | after | 20 | 96.2 | 100.0 | 14/20 (48-85%) | 20/20 | 6.8 | 241 | 1.50 | 364 | 0 | 2 | 0 | 6 |

### Target arrival interval, 4 drones (dynamic) (seeds 1-10)

| config | | runs | completion % | completion % of feasible targets | missions fully done (95% CI) | missions done, feasible targets only | dist / optimum (ok runs) | makespan @15 m/s (ok runs, s) | alloc latency s | msgs / done target | geofence viol. | separation viol. | duplicate insp. | runs not finished |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| every 20 s | before | 10 | 61.2 | 63.8 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 542 | 0 | 2 | 0 | 10 |
| every 20 s | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 3.8 | 243 | 1.50 | 207 | 0 | 0 | 0 | 3 |
| every 40 s | before | 10 | 77.5 | 80.5 | 1/10 (2-40%) | 2/10 | 6.0 | 397 | 1.50 | 451 | 0 | 1 | 0 | 9 |
| every 40 s | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 5.6 | 343 | 1.50 | 265 | 0 | 0 | 0 | 3 |
| every 60 s | before | 10 | 91.2 | 94.6 | 5/10 (24-76%) | 7/10 | 8.4 | 500 | 1.50 | 422 | 0 | 1 | 0 | 5 |
| every 60 s | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 8.0 | 489 | 1.50 | 341 | 0 | 0 | 0 | 3 |
| every 90 s | before | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 11.5 | 697 | 1.50 | 475 | 0 | 1 | 0 | 3 |
| every 90 s | after | 10 | 96.2 | 100.0 | 7/10 (40-89%) | 10/10 | 11.3 | 685 | 1.50 | 447 | 0 | 0 | 0 | 3 |

### Target arrival interval, 4 drones (dynamic) (seeds 11-20)

| config | | runs | completion % | completion % of feasible targets | missions fully done (95% CI) | missions done, feasible targets only | dist / optimum (ok runs) | makespan @15 m/s (ok runs, s) | alloc latency s | msgs / done target | geofence viol. | separation viol. | duplicate insp. | runs not finished |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| every 20 s | before | 10 | 58.8 | 63.6 | 0/10 (0-28%) | 0/10 | - | - | 1.50 | 579 | 0 | 1 | 0 | 10 |
| every 20 s | after | 10 | 95.0 | 100.0 | 7/10 (40-89%) | 10/10 | 3.9 | 256 | 1.50 | 222 | 0 | 0 | 0 | 3 |
| every 40 s | before | 10 | 85.0 | 90.7 | 3/10 (11-60%) | 5/10 | 6.0 | 356 | 1.50 | 384 | 0 | 0 | 0 | 7 |
| every 40 s | after | 10 | 93.8 | 100.0 | 6/10 (31-83%) | 10/10 | 5.3 | 339 | 1.50 | 285 | 0 | 0 | 0 | 4 |
| every 60 s | before | 10 | 91.2 | 98.8 | 4/10 (17-69%) | 9/10 | 7.8 | 488 | 1.50 | 463 | 0 | 0 | 0 | 6 |
| every 60 s | after | 10 | 93.8 | 100.0 | 6/10 (31-83%) | 10/10 | 7.6 | 482 | 1.50 | 357 | 0 | 0 | 0 | 4 |
| every 90 s | before | 10 | 93.8 | 100.0 | 6/10 (31-83%) | 10/10 | 11.5 | 718 | 1.50 | 510 | 0 | 0 | 0 | 4 |
| every 90 s | after | 10 | 93.8 | 100.0 | 6/10 (31-83%) | 10/10 | 11.2 | 701 | 1.50 | 471 | 0 | 0 | 0 | 4 |

### Target arrival interval, 4 drones (dynamic) (seeds 1-20)

| config | | runs | completion % | completion % of feasible targets | missions fully done (95% CI) | missions done, feasible targets only | dist / optimum (ok runs) | makespan @15 m/s (ok runs, s) | alloc latency s | msgs / done target | geofence viol. | separation viol. | duplicate insp. | runs not finished |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| every 20 s | before | 20 | 60.0 | 63.7 | 0/20 (0-16%) | 0/20 | - | - | 1.50 | 560 | 0 | 3 | 0 | 20 |
| every 20 s | after | 20 | 95.6 | 100.0 | 14/20 (48-85%) | 20/20 | 3.9 | 249 | 1.50 | 214 | 0 | 0 | 0 | 6 |
| every 40 s | before | 20 | 81.2 | 85.6 | 4/20 (8-42%) | 7/20 | 6.0 | 366 | 1.50 | 417 | 0 | 1 | 0 | 16 |
| every 40 s | after | 20 | 95.0 | 100.0 | 13/20 (43-82%) | 20/20 | 5.5 | 341 | 1.50 | 275 | 0 | 0 | 0 | 7 |
| every 60 s | before | 20 | 91.2 | 96.7 | 9/20 (26-66%) | 16/20 | 8.1 | 495 | 1.50 | 443 | 0 | 1 | 0 | 11 |
| every 60 s | after | 20 | 95.0 | 100.0 | 13/20 (43-82%) | 20/20 | 7.8 | 486 | 1.50 | 349 | 0 | 0 | 0 | 7 |
| every 90 s | before | 20 | 95.0 | 100.0 | 13/20 (43-82%) | 20/20 | 11.5 | 707 | 1.50 | 492 | 0 | 1 | 0 | 7 |
| every 90 s | after | 20 | 95.0 | 100.0 | 13/20 (43-82%) | 20/20 | 11.2 | 692 | 1.50 | 459 | 0 | 0 | 0 | 7 |

### Recovery after the failure of D1 (re-commit of its task by another drone)

| drones | | runs | runs where the task was re-committed | mean time (s) |
|---|---|---|---|---|
| 2 | before | 20 | 0 / 20 | - |
| 2 | after | 20 | 20 / 20 | 41.6 |
| 4 | before | 20 | 10 / 20 | 8.7 |
| 4 | after | 20 | 10 / 20 | 8.7 |
| 6 | before | 20 | 10 / 20 | 8.7 |
| 6 | after | 20 | 10 / 20 | 8.7 |
| 8 | before | 20 | 9 / 20 | 8.7 |
| 8 | after | 20 | 9 / 20 | 8.7 |