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
- Optional no-fly zone (2 corners).
- Drone table: battery %, altitude, sensor radius, and "fail at" time (to demonstrate takeover).
- Mode: neutralize (simulated) or inspect only.
- Playback: play/pause, speed, scrubber; panels show drone modes, target state, **bid table with the winner and why**,
  events; "Download log (CSV)".

How it works: the server runs the whole simulation, records a frame every 0.5 s, and the page replays it.
`POST /api/run` takes the scenario (JSON) and returns frames, events, bids, routes, summary.

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

About 406 tests: unit tests (cost, consensus, network, geofence, planner, webmap, crew) and simulation tests per phase
(including decentralization checks, i.e. drones decide without a central controller). The crew agents are tested with a
scripted fake model, so no key or network is needed. Browser behaviour was checked with automated Playwright runs.

## 9. Limits (be honest in the viva)

- Simulation only; simple point-mass drone dynamics and a flat-earth coordinate conversion (accurate to metres over km).
- Single target at a time in the web map; radio model is a simple loss/delay model.
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
