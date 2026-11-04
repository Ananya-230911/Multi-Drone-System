# Multi-Drone Decentralized Target Inspection: full documentation

Everything here is a **simulation**. Nothing connects to a real vehicle; the "neutralization" is a simulated event
(sides and places are fictional or neutral).

## 1. The idea

A few drones patrol near an area. A target is reported at a location. The drones **talk to each other, compare their
own numbers (distance, altitude, battery, sensor radius, route) and agree among themselves which one goes**. The winner
flies there, confirms the target with its own sensor, locks on, **logs it**, and in "neutralize" mode the simulated
target is destroyed (explosion and crater on the map). If the winner fails, another drone takes over.

There is no central controller: every drone runs the same code and reaches the same decision from the messages it gets.

## 2. What was built, by phase

| Phase | What | Where |
|---|---|---|
| 1 | Simulation world, drones, battery model, sensor, config files, event log | `src/sim`, `src/drone`, `src/config.py`, `config/*.yaml` |
| 2 | Per-drone target tracking from noisy observations; target lifecycle (17-row state machine); `target_log.csv` | `src/tracking`, `src/drone/fsm.py`, `src/logging_` |
| 3 | Safety: geofence (boundary + no-fly zones), route validation, visibility-graph A* planner, separation monitor | `src/safety` |
| 4 | Decentralized allocation (auction + consensus), unreliable radio network, mission execution, baseline for checking | `src/allocation`, `src/comms`, `src/drone/mission.py` |
| Map | Browser map (Leaflet / OpenStreetMap), any-size area, click-to-place target, no-fly zone, drone table, explosion animation | `src/webmap` |
| AI crew | One sentence sets up the scenario (CrewAI + Groq) and an AI debrief explains the result | `src/crew` |
| 5 | Owner failure and network-split scenarios, duplicate-owner detection, batch evaluation with metrics and plots | `config/drone_failure.yaml`, `config/partition.yaml`, `src/evaluate.py` |
| Compare | Our auction vs nearest / most-battery / random / centralized optimum, plus literature comparison | `src/compare.py`, `docs/COMPARISON.md` |
| Benchmark fixes | Task queue, commit-time re-check, retry, no re-inspection, direct chaining, failure handover (benchmark completion 61 % to 96 %) | `src/allocation/cost.py`, `src/drone/mission.py`, `docs/BENCHMARK_BEFORE_AFTER.md` |
| Multi-target map | Several targets at once on the web map, command announcement and one bids table per target | `src/webmap/*` |

## 3. How the drones decide (the core)

1. A target is detected or reported (an "external cue" gives coordinates only).
2. Every drone computes its own **cost** for the task:
   `cost = 1.0 * travel_time + 0.5 * energy_needed / capacity + 0.3 * (1 - battery_after / capacity)`
3. A drone is **infeasible** if it would land below the 20% battery reserve, has no valid route, or the route breaks
   the geofence. Infeasible drones do not bid.
4. Drones exchange bids by radio (messages can be lost or delayed). The **lowest cost wins** (max-consensus on
   `-cost`); the winner sends CLAIM, the others ACK. Rounds are numbered (epochs).
5. If the owner times out or releases the task, the auction reopens and another drone wins (swap / takeover).
6. A separate **centralized optimum** is computed only for evaluation, to check the drones picked the same drone.

Safety rules always apply: geofence is a hard filter, routes are validated, drones keep separation.

## 4. The map page

Run: `python -m src.webmap` then open http://127.0.0.1:8000 (local only).

- Search any place (OpenStreetMap), or type `lat, lon`.
- Operating area: any rectangle 0.8 to 12 km per side (type width/height, or draw by 2 corners).
- Target: **click anywhere inside the blue rectangle**, or type exact coordinates.
- **Several targets at once:** after the first target press **+ Add another target** and click more spots (up to 8, at least 60 m
  apart; "Remove extra targets" clears them). See section 4b.
- Optional no-fly zone (2 corners).
- Drone table: battery %, altitude, sensor radius, and "fail at" time (to demonstrate takeover).
- Mode: neutralize (simulated) or inspect only.
- Playback: play/pause, speed, scrubber; panels show drone modes, target state, **bid table with the winner and why**,
  events; "Download log (CSV)".

How it works: the server runs the whole simulation, records a frame every 0.5 s, and the page replays it.
`POST /api/run` takes the scenario (JSON) and returns frames, events, bids, routes, summary.

## 4b. Several targets at once (web map)

What happens when you give more than one target:

1. **At 5 s the command goes out to every drone** at the same moment. The map draws a coloured ring at each new target and
   dashed lines from it to every drone for about 3 s, and the banner says "COMMAND SENT to all N drones".
2. **Each target gets its own bidding round.** The right panel shows one bids table per target ("BIDS - TARGET 1, 2, ...")
   with its own winner. A target given a delay (`at`) is announced, and bid for, only at that later time.
3. **The work is shared.** A drone that wins one target is usually not the best for the next; if a drone is already busy it
   keeps the next target in its queue (`allocation.max_queue`) and flies straight to it. Example run: 3 targets, 4 drones,
   3 different drones, all 3 neutralized, no geofence or separation violations.
4. **Each target is shown separately:** numbered marker, its own lock line and explosion, a per-target line in the panel,
   a banner counting "1 of 3 targets neutralized", and a per-target Result section.
5. **The run lasts until every target has ended** (logged, neutralized or cancelled), not until the first one does (a bug in
   the first version cut the recording 12 s after the first target; fixed).

API: `POST /api/run` accepts `targets: [{lat, lon, at?}]` instead of `target`; the answer then also has `targets` (with
`cue_t`), per-frame `tv` (state, owner, lock per target) and `nz` (neutralized flags), `summary.per_target`, and each bid
round carries `target_index`. A request with a single `target` is unchanged.

## 5. The AI crew

Type one sentence (e.g. "Send drones to Geeta Colony") and press **Ask the AI crew**. Built with the open-source
**CrewAI** Python library, running on your PC, with a **Groq** model as the brain (key in `.env`). It does not use the
CrewAI web dashboard.

| Agent | Job |
|---|---|
| Geo Scout | searches the map with a tool (`search_place`), picks the place; may only use coordinates the search returned |
| Mission Planner | area width/height, mode, whether to add a no-fly block |
| Fleet Engineer | 4-5 drones: start position, altitude, battery, sensor radius (all different) |
| Debrief Officer | after the run, explains in plain words why the winner won |

- **You** click the target; the crew does not choose it.
- The crew only **sets up and explains**. Winner selection stays with the drones' auction.
- Every AI answer is validated (CrewAI guardrail); a bad one is sent back for correction (2 retries). If it still fails,
  or there is no key / no CrewAI, built-in **offline rules** supply that part, and the page labels each step "AI" or "rules".
- Offline results are seeded from sentence + place: different places give different numbers, the same sentence repeats.
- Endpoints: `POST /api/crew`, `POST /api/debrief`, `GET /api/status` (never reveals the key).

## 5b. Phase 5: failures, partitions, evaluation

- **Owner failure** (`drone_failure.yaml`): the winner dies mid-flight; heartbeats stop; the owner timeout reopens the
  auction at epoch + 1 (about 8.6 s later); another drone completes and logs the target once.
- **Network split** (`partition.yaml`): the radio splits {D1,D2} | {D3,D4}; each side elects its own owner (two drones
  head for one target). After the heal, owners see each other's claim in heartbeats (`DUPLICATE_OWNER_SEEN`), the
  compare rule makes the weaker claim release, and the target is logged once.
- **Evaluation** (`python -m src.evaluate --seeds 10 --workers 4`): every scenario x N seeds, expectations checked,
  writes `output/eval/` (`metrics.csv`, `summary.md`, `timeline.png`, `messages.png`, `agreement.png`,
  `robustness.png`). Result: 120/120 runs met expectations; geofence and separation violations 0; false confirmations 0%;
  baseline agreement 100% where the drones can talk.
- **Two bugs found and fixed by it:** owners never noticed each other after a heal; records merged in opposite
  directions could loop forever (the smallest ID now always survives).

## 5c. Comparison

- **Measured** (`python -m src.compare`, 129 allocations): ours = centralized optimum (100% best pick, 0 extra cost);
  nearest drone picks the best only 69.8% and a drone that cannot finish 15.5% of the time; most-battery 49.6% best,
  extra cost 19.06; random extra cost 34.74. Plots in `output/compare/`.
- **Literature** (`docs/COMPARISON.md`): CBBA (Choi, Brunet, How 2009), MURDOCH (Gerkey & Matarić 2002), a gossip-based
  drone auction (2024). Qualitative table only, because their problems and metrics differ; check each row against the
  paper before the viva.

## 5d. Benchmark fixes (queue, retry, no double assignment)

After a benchmark against two similar projects (completion 61%, 0/10 missions finished with 4 drones and targets every
20 s) the allocation was extended; see the README section "Benchmark fixes" for the new `allocation.*` keys and their
defaults (`max_queue` 4, `retry_unassigned` true, `retry_period` 10, `retry_max_s` 600, `skip_completed_sites` true).
In short: drones hold a short queue of tasks and bid the marginal cost of inserting a target (battery reserve checked
over the whole queue), a second win is re-checked at commit time and queued or released, unassigned targets are
retried, finished drones fly straight to their next target, and a target that was already inspected is not inspected
again. The old behaviour is restored exactly with `max_queue: 1`, `retry_unassigned: false`,
`skip_completed_sites: false`. Before/after numbers: `docs/BENCHMARK_BEFORE_AFTER.md`.

Benchmark result (4 drones, 8 targets arriving every 20 s, 420 runs per version, 20 seeds):

| Measure | Before | After |
|---|---|---|
| Task completion | 61 % | 96 % |
| All targets all at start | 45 % | 96 % |
| Missions fully completed (seeds 1-10) | 0 / 10 | 7 / 10 (10 / 10 counting only targets outside the no-fly zone) |
| Completion at 30 % packet loss | 60 % | 96 % (same as 0 %) |
| 2 drones, 1 fails | 19 % | 95 % |
| Double-assignment overwrites (240 replays) | 81 runs | 0 |
| Geofence violations | 0 | 0 |
| Messages per completed target | 560 | 214 |

Also fixed on the way: a cue that merged into a drone's own detection was later cancelled and lost; a drone that gave a task
back only for lack of room was excluded from the re-auction and used up the reallocation budget (stranding tasks when it was
the last drone); a drone chaining to its next task refused bids for one tick.

Honest notes: the benchmark harness inherits the simulator's default no-fly zone, and 9 of 160 target placements lie inside
or on its edge, so 8 / 10 fully completed missions cannot be reached without changing the benchmark; with 2 drones separation
violations rose from 0 to 4 (two targets 25 m apart, or a failed drone frozen in the air); adaptive heartbeats (to cut the
message count further) were not done; a drone sent to a target on the edge of a no-fly zone can hover there without ever
completing or timing out (1 of 240 replays).

## 6. Setup

Plain simulation: Python 3.10+ (3.14 works with `pygame-ce`).
AI crew: **Python 3.10-3.13** (CrewAI does not support 3.14).

```powershell
py -3.13 -m venv .venv313
.venv313\Scripts\Activate.ps1
pip install -r requirements.txt -r requirements-crew.txt
copy .env.example .env        # open .env, paste your key after GROQ_API_KEY=
python -m src.webmap
```
`.env` is git-ignored; never commit or share the key. Other commands: `python -m src.main` (pygame window),
`python -m pytest` (all tests), `python -m src.evaluate`, `python -m src.compare`.

## 7. Project layout

```
config/            scenario YAML files (default, geofence_detour, low_battery, limited_comms, ...)
src/sim            world, scenario events, ground truth
src/drone          agent, FSM, battery, navigation, sensor, mission, coordination
src/tracking       observations, per-drone target tracker
src/safety         geofence, route validator, A* planner, monitor
src/allocation     cost, auction, baseline (evaluation only)
src/comms          messages, lossy network
src/logging_       event log (events.jsonl), target_log.csv
src/viz            pygame renderer and panels
src/webmap         geo conversion, scenario builder, runner, server, static page
src/crew           env, geocode, plan validators, offline planner, CrewAI agents, service
src/evaluate.py    batch runs, metrics, plots (Phase 5)
src/compare.py     our auction vs other selection methods
docs/COMPARISON.md measured and literature comparison
tests/unit, tests/sim
PLAN.md            the original plan / specification
```

## 8. Testing

About 454 tests: unit tests (cost, consensus, network, geofence, planner, webmap, crew) and simulation tests per phase
(including decentralization checks, i.e. drones decide without a central controller). The crew agents are tested with a
scripted fake model, so no key or network is needed. Browser behaviour was checked with automated Playwright runs.

## 9. Limits (be honest in the viva)

- Simulation only; simple point-mass drone dynamics and a flat-earth coordinate conversion (accurate to metres over km).
- The web map takes up to 8 targets at once (at least 60 m apart); a target inside a no-fly zone cannot be inspected. The radio model is a simple loss/delay model.
- Map tiles and place search need internet; the AI crew needs internet and a Groq key.
- The AI model proposes numbers; they are validated, but it is a planning aid, not part of the safety logic.
- If the network heals only after both owners already hover over the target, one separation violation is recorded (target still logged once).
- One of ten `limited_comms` seeds ends with a duplicate inspection; it is detected and marked, as required.
- Literature comparison is qualitative and based on search summaries.

## 10. Questions your professor may ask

- **Why decentralized?** No single point of failure; each drone decides from shared bids; the centralized solver is only a check.
- **How do they agree?** Bids are compared by max-consensus with claim/ack and epochs; ties and loss are handled by reopening.
- **What if the winner dies?** Timeout or release reopens the auction; the next best drone takes over (use "Fail at").
- **What does the AI do?** Sets up the scenario from one sentence and explains the result; it does not choose the winner.
- **How do you stop AI mistakes?** Tool-grounded coordinates, range checks, guardrail retries, offline fallback.
