# Multi-Drone Decentralized Target Inspection (simulation)

A fully simulated, decentralized team of 3–4 drone agents that patrol an area, detect a
simulated target, agree among themselves which drone inspects it, navigate inside geofence
limits, track the target and log the inspection. The full design is in [`PLAN.md`](PLAN.md).

**Simulation only.** Nothing in this project connects to or commands a real drone or flight
controller.

**Current status: Phase 4 of 5 (navigation, tracking, inspection / engagement, logging, recovery).**
See the phase reports below.

## Demo: four drones decide who takes the target

```bash
python -m src.main --config config/mission.yaml
```

Four drones operate near one target (`GT-1`). Nobody is in charge: each drone works out how
suitable it is, they exchange bids by radio, and they agree on one drone. That drone flies a
safe route, locks on, logs the target, and the simulated target is marked **NEUTRALIZED**.
Each drone shows a different parameter mattering:

| Drone | Situation | Result |
|---|---|---|
| D1 | closest, but low battery | cannot go + engage + return above the 20 % reserve, so `insufficient_energy` |
| D2 | flying high (110 m), right above the target | spots it first; must descend 50 m, still the cheapest, so **wins** |
| D3 | normal drone, mid distance, no-fly zone in between | 55 s away by the planned detour |
| D4 | farther away, small sensor (30 m) | must fly closer to the target; 60 s away |

Watch the BIDS table and the "WHY D2" lines in the panel, the red lock line and `LOCK n/10 s`
timer, and the green "MISSION COMPLETE" banner. To see the drones re-decide ("swap"), press `2`
then `F` while D2 is on its way: the others notice it is gone, reopen the task and send another
drone. Everything is simulated: "neutralized" is an event in the simulation, not a weapon model.

> **About "confidence":** every confidence value in this project is a *simulated score* used to
> exercise the decision logic (PLAN.md 7.3). It does not represent the detection accuracy of any
> real sensor or detector.

## Map version: search any place, click the target

```bash
python -m src.webmap
```

This opens a page in your browser with a real **OpenStreetMap** map (Leaflet), a **search box**
and the same simulation behind it:

1. **Search** a place (or type exact `lat, lon`). The map goes to the first match and drops a pin on
   it; every result lists its coordinates so you can pick the right city.
2. **Set the operating area (the geofence)**: type a width and a height (0.8 to 12 km each) and
   press the first button, which centres it where the **+** is, or press **Draw by 2 corners** and
   click two opposite corners. The area is the only place the drones may fly; they never leave it.
   The four drones appear inside it; drag them where you like.
3. **Place the target** by clicking inside the area (zoom in first for a more exact spot; the page
   shows the precision), or type its exact `lat, lon` and press **Set**.
4. Optionally **draw a no-fly zone** (two clicks).
5. Edit each drone's **battery, altitude and sensor radius**. "Fail at" breaks a drone at a given
   time to show the others taking over.
6. **Run simulation**, then play it back at 1x to 16x with the slider.

The drones are told where the target is (coordinates only; never counted as proof). They bid,
agree on one drone, which flies a safe route around the no-fly zone, confirms the target with
its own sensor, locks on (the target's marker pulses), logs it, and (in "neutralize" mode) the
simulated target is destroyed: a burst animation plays on the map and a scorched crater is left. The right-hand panel shows the bids and why the winner won, the events, and the
result; **Download log (CSV)** saves the mission log.

Notes: the map pictures and the search need an internet connection (they come from
OpenStreetMap, shown with its required credit); Leaflet itself is bundled, and the simulation runs
on your own computer only (the server listens on `127.0.0.1`). Everything is simulation. Use
`--port N` to change the port and `--no-browser` to skip opening the browser.

## Install

Python 3.11 or newer.

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

The dashboard uses `pygame-ce`, a maintained fork of pygame that is imported the same way
(`import pygame`). The original `pygame` package has no prebuilt wheels for Python 3.14 and fails
to install there. If you already have `pygame` installed, uninstall it first
(`pip uninstall pygame`), because the two packages conflict.

## Run

```bash
# Dashboard (pygame window)
python -m src.main --config config/default.yaml

# Headless (no window), e.g. for tests or batch runs
python -m src.main --config config/default.yaml --headless

# Options
#   --seed N        override sim.seed
#   --duration S    override sim.duration (seconds)
#   --output DIR    output directory (default: output/)
#   --run-id NAME   run directory name (default: <scenario>_seed<seed>_<timestamp>)
```

Each run writes `output/<run_id>/events.jsonl` (one JSON event per line),
`output/<run_id>/target_log.csv` (the mission log: one row per target) and
`output/<run_id>/summary.json` (final drone states, each drone's target records, geofence and
separation violations, false confirmations, message counts, each allocation's owner versus the
centralized baseline, and the SHA-256 of the event log).

### Scenarios

| File | What it shows (Phase 2) |
|---|---|
| `config/default.yaml` | 4 drones patrol; D2 passes 50 m from GT-1, detects it, holds, confirms it |
| `config/geofence_detour.yaml` | a long restricted wall; D1 confirms a target in the east, then its route home goes around the wall |
| `config/target_in_restricted.yaml` | target inside RZ-1; D1 sees it from the zone edge but may not overfly it, so the record is rejected |
| `config/false_observation.yaml` | no real target, only spurious observations; nothing may ever be confirmed |
| `config/duplicate_reports.yaml` | three drones reach GT-1 on the same tick and each creates its own record; MERGE messages bring every agent to `T-D1-001` |
| `config/external_cue.yaml` | no patrol sees GT-1; a cue with its coordinates arrives and is allocated while still unconfirmed |
| `config/low_battery.yaml` | the drone nearest a cue cannot afford the round trip (`insufficient_energy`), so another wins |
| `config/limited_comms.yaml` | 600 m radio range, drones in a line; D4 never hears D1, yet all agree through relaying |
| `config/tracking_loss.yaml` | GT-1 is hidden for 8 s during the inspection: track lost, spiral search, reacquired, timer paused |
| `config/mission.yaml` | the demo above: four drones, different parameters, one engages, target NEUTRALIZED |

### Dashboard controls

| Key / mouse | Action |
|---|---|
| `SPACE` | pause / resume |
| `←` / `→` | slower / faster (0.25× to 8×) |
| `R` | reset the scenario (same seed) |
| `1`–`4` | select a drone |
| `F` | fail the selected drone |
| Left click on map | place a **ground-truth** target (drones still have to detect it) |
| `Shift` + click | send an **external cue** (coordinates only; never counts as evidence) |
| `H` | hide / show all ground-truth targets |
| `G` | show / hide ground-truth markers |
| `M` | show / hide message flash lines |
| `TAB` | switch the panel feed between events and the message log |
| `T` | cycle the message-log filter (all, HEARTBEAT, BID, CLAIM, ..., dropped only) |
| `L` | show / hide the legend |
| `Esc` | quit |

The window scales to fit your screen (Windows display scaling is handled) and can be resized or maximised.

None of these touch a drone directly. Each one queues a scenario event that is applied on the
next tick and logged as `SCENARIO_EVENT` (PLAN.md 4.1 rule 6).

## Tests

```bash
python -m pytest            # all tests (about 1 minute; the simulation tests run full scenarios)
python -m pytest tests/unit # algorithm tests only, no world and no pygame (about 1 s)
```

## Layout

```
config/*.yaml               scenarios (see the table above)
src/config.py               YAML -> typed dataclasses, full schema validation
src/main.py                 entry point (dashboard or headless)
src/sim/world.py            sim clock, deterministic tick loop, owns ground truth and sensors
src/sim/ground_truth.py     true targets and visibility (agents never read this)
src/sim/scenario.py         scheduled and operator-injected events
src/drone/agent.py          drone agent (one per drone, identical code)
src/drone/state.py          private drone state
src/drone/fsm.py            drone behaviour state machine
src/drone/battery.py        energy model and reserve rule
src/drone/navigation.py     3D waypoint follower
src/drone/sensor.py         simulated sensor -> observations
src/tracking/observation.py observation types and validation
src/tracking/target_state.py target lifecycle FSM: transition table rows 1-17, guards, authority
src/tracking/tracker.py     per-agent records: association, merge, confirmation, freshness
src/safety/geofence.py      boundary, zones, altitude limits, margin, edge rule
src/safety/route_validator.py point / segment / route validation with reason codes
src/safety/planner.py       visibility graph + A*, 3D profile, inspection points
src/safety/monitor.py       runtime checks: next step, separation, battery reserve, landing
src/comms/messages.py       message types and per-drone outbox
src/comms/network.py        simulated radio: range, latency, loss, partitions
src/drone/coordination.py   inbox handling, peer table, sharing, auction actions
src/allocation/cost.py      bid cost and feasibility with breakdown
src/allocation/auction.py   decentralized auction + consensus
src/allocation/baseline.py  centralized optimum (evaluation only)
src/drone/mission.py        the owner's mission: route, lock, inspect, log, reacquire, release
src/logging_/target_log.py  per-agent logbooks merged into target_log.csv
src/logging_/event_log.py   JSON Lines event log (canonical, hashable)
src/viz/                    snapshot, map renderer, side panel, legend
src/webmap/                 browser map: Leaflet page, scenario builder, runner, local server
tests/unit/                 algorithm tests (no world, no pygame)
tests/sim/                  headless scenario tests per phase
```

`src/evaluate.py` (batch runs, metrics and plots) arrives in Phase 5.

---

## Phase 1 report

### What was built
- **Config** (`src/config.py`): every section of the PLAN.md section 12 schema is parsed into
  frozen dataclasses and validated (types, ranges, unknown keys, duplicate IDs, drone references
  in scenario events and partitions, cruise/standoff altitude inside the altitude limits).
  Sections used only by later phases are validated now and reported as `info:` lines at startup.
  Each drone entry may override any `drone_defaults` key (nested `sensor` keys merge).
- **World** (`src/sim/world.py`): integer tick counter (`t = tick × dt`, so the clock does not
  drift), tick order scenario → drones in sorted ID order → record. `make_rng(seed, component)`
  gives each component its own NumPy generator derived from `sim.seed`. Phase 1 has no
  randomness, so this is set up for the sensor and network in later phases.
- **Drone agent** (`src/drone/`): private state; battery with level, climb, descent and hover
  costs; a waypoint follower with simultaneous horizontal (≤ `max_speed`) and vertical
  (≤ `climb_rate`) motion; drone FSM with `PATROL`, `RETURNING`, `FAILED`. Every mode in
  section 6.2 is declared in the enum, but only these three have transitions so far.
  The agent queues its events in an outbox that the world drains, so it holds no reference to
  the world, the log or other drones (checked by a test).
- **Battery reserve protection** (6.4): each tick in `PATROL` the drone computes the energy of
  its route home and switches to `RETURNING` if `battery − E_home` would fall below the reserve.
- **Event log** (`src/logging_/event_log.py`): canonical JSON Lines (sorted keys, floats rounded
  to 6 digits), so the same seed gives a byte-identical file and SHA-256.
- **Dashboard** (`src/viz/`): map with boundary, grid, patrol routes, homes, trails and drones
  (triangle plus a mode label, cross for FAILED, ring for RETURNING), top bar, drone panel with
  battery bars and reserve marker, event feed, legend, and the controls above. It draws only
  from the read-only `Snapshot`.

### Acceptance check (default.yaml)
| Criterion | Result |
|---|---|
| 4 drones patrolling | Yes. All four fly their loops at 15 m/s and 60 m |
| Battery decreases | Yes. After 400 s: D1 61%, D2 46%, D4 71% |
| Climbs cost more than level flight | Yes. `k_c` = 5 × `k_h` per meter; a test flies the same square level and with altitude changes and checks the second uses more energy |
| Failing a drone freezes it | Yes. Position, altitude and battery stay constant and it emits no more events (tested through the config and through the operator key path) |
| Same seed → identical event logs | Yes. Byte-identical `events.jsonl` across runs and through the CLI |

D3 starts at 30%. At t = 31.3 s its reserve check trips (`SAFETY_RETURN`) and it flies home.
It reaches home at t = 184.5 s with 20.02%, just above the 20% reserve, and stays parked.

**Tests:** 59 passed (`tests/unit/test_battery.py`, `test_navigation.py`, `test_config.py`,
`tests/sim/test_phase1_patrol.py`).

### Phase 1 simplifications (to be replaced in later phases)
1. **Route home is the direct line** (climb to cruise altitude over home, then descend). There is
   no planner or geofence yet, so `RZ-1` and the boundary are **not enforced** in Phase 1, and
   patrol routes are not validated against the geofence. *(Resolved in Phase 2: routes are
   planned and validated, and patrol routes are checked at startup.)*
2. **Reserve look-ahead.** Checked literally, `battery − E_home < reserve` trips one tick late,
   after the reserve has already been crossed (D3 came home at 19.99%). The check now adds an
   upper bound on how much one more tick can change `battery − E_home`, so it trips before the
   reserve is crossed. It comes from the drone's own parameters, so there is no new magic number.
3. **After returning home** a drone resumes patrol only if it can reach its nearest patrol
   waypoint and get back home above the reserve. Otherwise it stays parked on the ground in
   `RETURNING` with no energy use (shown as "RETURNING (home)"). There is no recharging. A
   `set_battery` event that makes resuming feasible sends it back to patrol.
4. **Hover energy** is charged only while airborne with no waypoint to fly to. Pure vertical
   motion is charged through `k_c` / `k_d`.
5. **`LANDED` is not implemented yet.** If a `set_battery` event drops a drone to 0, the battery
   clamps at zero and the drone still flies home. *(Resolved in Phase 2: the monitor lands the
   drone.)*
6. **Two event types beyond the section 13.1 list:** `WAYPOINT_REACHED` and `HOME_REACHED`, so
   the Phase 1 log has enough content for the determinism hash to be meaningful.
7. Scenario event types for later phases (`external_cue`, `place_target`, `hide_target`,
   `show_target`) are validated, logged as `SCENARIO_EVENT` with `applied: false`, and ignored.
   *(Phase 2 applies all of them except `external_cue`, which needs Phase 3.)*
8. Pressing `R` resets the run. Only the run that is active when the window closes is written to
   `output/`.

---

## Phase 2 report

### What was built
- **Ground truth and sensor** (`sim/ground_truth.py`, `drone/sensor.py`): targets with
  `appear_at`, hidden intervals and operator hide/show. The world owns one sensor per drone, each
  with its own seeded random generator. It applies 7.2 (`r_eff = min(radius, alt·tan(half_fov))`,
  altitude band, `rate_hz`, `p_detect`, position noise, Poisson spurious observations) and 7.3
  (confidence falloff, categories). A drone only gets a `sensor_view` callable that returns
  `AgentObservation`s, which have no `is_spurious` field and no ground-truth IDs.
- **Observation validation** (`tracking/observation.py`): future or too-old timestamp, outside the
  operating area, confidence out of range, unknown observer, duplicate `obs_id`. Each rejection
  is logged as `OBSERVATION_REJECTED` with its reason.
- **Target lifecycle** (`tracking/target_state.py`): the full transition table, rows 1–17, with
  guards and who may cause each transition (owner-only rows are enforced). Terminal states are
  sticky, a reopen increments the epoch and excludes the previous owner, and the remote-adoption
  rule is implemented now (it is used once messages arrive in Phase 3). Illegal transitions raise
  an exception.
- **Tracker** (`tracking/tracker.py`), one per agent: IDs `T-<creator>-<seq>`, association within
  `association.radius`, the estimate as an exponential average, duplicate merge keeping the
  smallest ID with alias redirect, no merging into terminal records, the
  `ignore_after_complete` window, sliding-window confirmation (count, mean confidence, distinct
  observers), timeout → `REJECTED` → `CANCELLED`, and `FRESH`/`STALE`/`LOST` freshness.
- **Safety layer** (`safety/`):
  - *Geofence* (shapely): the boundary shrunk and zones grown by the margin, a point on an edge
    counts as outside, altitude limits, and config checks for invalid or self-intersecting
    polygons and for zones that don't overlap the boundary.
  - *Route validator*: segment and route checks with the section 9.2 reason codes.
  - *Planner*: visibility graph + A\*. The fixed part of the graph is built once, so a plan takes
    about 0.1 ms. It adds the cruise-altitude profile, vertical takeoff and landing at home, and
    `inspection_point` (above the target, else along the approach, else points on a circle,
    else `None`).
  - *Monitor*: checks every tick, before moving, for the next position, the segment about to be
    flown, the battery reserve (now using the planned route home) and emergency landing.
- **Startup validation**: homes, start positions and full patrol loops are checked before
  tick 0. An invalid one stops the run with a clear message, for example
  `drone D1: patrol route is invalid (CROSSES_ZONE:RZ-1, segment 1: ...)`.
- **Drone FSM** gains `CONFIRM_HOLD`, `SAFETY_HOLD` (replanning up to `replan_limit` times, then
  return home) and `LANDED`.
- **Dashboard**: boundary and dashed margin, hatched zones with IDs and dashed buffers, sensor
  footprints, belief targets (marker shape and colour per state, plus a confidence ring), the
  ground-truth cross, safety warning rings, the planned route with waypoints, a targets panel
  showing each drone's own view, a fuller event feed, a geofence-violation counter, the new
  controls and an extended legend.

### Acceptance check
| Criterion | Result |
|---|---|
| A drone flying near GT-1 creates a record and confirms it; nothing further happens | `default`: D2 creates `T-D2-001` at t = 83.1 s, holds, and it is CONFIRMED at t = 88.2 s (7 observations, mean confidence 0.65). The estimate is 1.3 m from the truth. The only transitions are rows 1 and 2, and D2 returns to patrol. |
| Spurious observations never confirm | `false_observation` over seeds 1, 2, 3 and 42: 389 spurious observations and 222 records with seed 42, all REJECTED → CANCELLED, 0 confirmed |
| Planner routes around the zone | `geofence_detour`: D1's route home is `(1700,844) → (1116,284) → (884,284) → (150,1000) → down`, around the wall's southern end |
| No commanded position ever outside the geofence | Asserted every tick in all 5 scenarios: 0 violations. The world also measures true positions independently. |

Other checks: in `target_in_restricted` the hold point falls back to the zone edge
(1116, 786), where confidence is only about 0.3, so the record is rejected and the zone is never
entered. In `duplicate_reports`, D1, D2 and D3 each hold exactly one local record
(`T-D1-001`, `T-D2-001`, `T-D3-001`), all confirmed. Merging them across agents needs messages
(Phase 3).

**Tests:** 229 passed. New unit tests: `test_geofence`, `test_route_validator`, `test_planner`,
`test_sensor`, `test_observation`, `test_target_state` (every legal row, illegal samples, sticky
terminals, adoption), `test_tracker`. New simulation tests: `test_phase2_detection_safety`. It
covers the per-tick geofence assertion; that every planned route validates; HOLD on an invalid
next step, simulated by handing a drone an unvalidated route through RZ-1; the replan limit →
return home; landing on a depleted battery; startup config errors; hidden targets; placing a
target; and determinism with sensor randomness.

### Phase 2 decisions to review
1. **`CONFIRM_HOLD` flies to the detection, then holds.** Confidence falls to 0 at the edge of
   the sensor footprint, and D2 first sees GT-1 50 m away with a 60 m radius, so its confidence
   there is about 0.07. Holding at the spot where it first saw the target would never confirm.
   So the hold point is `inspection_point(estimate)`: directly above the estimated position if
   that is allowed, else the closest valid point within sensor range. The hold is still capped
   at `hold_max`, and the drone holds at most once per record.
2. **Home corridor.** Homes are at altitude 0, below `min_alt` = 15. A point exactly above a
   home, and a purely vertical segment there (takeoff and landing), are exempt from the altitude
   floor. Nowhere else is.
3. **Margins use mitre joins**, so a grown rectangle stays a rectangle. This is slightly more
   conservative than a rounded buffer, and it keeps the planner graph small.
4. **New config key `navigation.planner_clearance`** (1.0 m): planner nodes sit this far
   outside the margin, because a node exactly on a buffered edge is "outside" by the edge rule.
5. **Route checks at run time.** The geofence is static in Phases 1–5, so routes are validated
   in full when they are assigned. Each tick the monitor then checks the next position and the
   segment about to be flown, instead of re-validating the whole remaining route.
6. **Energy estimates skip the final re-validation.** The route home used for the reserve check
   every tick is an estimate and is never flown. Its segments are already checked while planning,
   so the final `validate_route` pass is skipped there (`revalidate=False`). Every route a drone
   actually flies is fully re-validated.
7. **Row 2 guard.** Confirmation is only ever computed from sensor observations; an external cue
   never counts as evidence. So "source ≠ external_cue, or the evidence is sensor observations"
   always holds once `confirmation = CONFIRMED`.
8. **Records after cancellation.** A rejected record is terminal, so the next pass over the same
   spot creates a new record (`target_in_restricted` shows `T-D1-001`, then `T-D1-002`). The
   ignore window in 7.4 applies only after COMPLETED, as the plan specifies.
9. `merge_remote` raises `NotImplementedError` until Phase 3 adds messages.

---

## Phase 3 report

### What was built
- **Simulated radio** (`comms/network.py`): every message is a broadcast. Delivery depends on
  range (the smaller of the network's and the sender's range), per-recipient packet loss and
  uniform latency, so messages can arrive out of order. Scheduled partitions are supported, and
  FAILED drones neither send nor receive. Every send, delivery and drop is logged with a reason.
- **Messages** (`comms/messages.py`): the ten types in section 10.1, each identified by
  `(sender, seq)`. Receivers drop duplicates and messages older than `max_message_age`.
- **Heartbeats and peer table**: each drone broadcasts its position, velocity, battery, mode and
  task every second. A peer counts as alive if it was heard within `miss_limit` periods.
- **Sharing** (`drone/coordination.py`): `TARGET_DETECTED` when a record is created, `OBSERVATION`
  (at most `obs_share_hz` per target), `TARGET_STATE` with the confirming evidence, and `MERGE`.
  A drone re-broadcasts what it learns once, so information travels several hops when the radio
  range is short.
- **Peer reports in the tracker**: a peer's record is copied under the peer's ID. The copy is
  CONFIRMED only if the shared evidence meets the 7.5 criteria when this agent checks it itself.
  A duplicate merges into the smallest ID, and a peer's MERGE renames or merges local records.
- **Cost** (`allocation/cost.py`): exactly as in 8.2, with every term in the breakdown. Drones that
  cannot compete send `feasible=false` with a reason: `mode_RETURNING`, `parked_at_home`, `busy`,
  `excluded_this_epoch`, `no_inspection_point`, `no_route`, `no_route_home` or
  `insufficient_energy`. The geofence enters only through feasibility and route length.
- **Auction and consensus** (`allocation/auction.py`): the max-consensus rules of 8.3:
  - compare rule: higher epoch, then higher bid, then lower ID;
  - any change is re-broadcast immediately (this is the relay);
  - commit after `consensus_rounds` stable rounds and a BID, CLAIM or ACK from every alive peer;
  - after `allocation_timeout`, commit the current winner, or cancel with `no_feasible_bidders`;
  - the winner sends CLAIM and the others ACK; a committed agent repeats its CLAIM or ACK to a
    peer that is still bidding;
  - conflicting CLAIMs resolve by the compare rule, and the loser sends `RELEASE(conflict_lost)`;
  - auction tables of merged duplicate records are combined.
- **Lifecycle**: every agent applies row 5 (CONFIRMED → ALLOCATED), or row 4 for a cue (still
  DETECTED and PENDING), when it commits.
- **External cues**: every live drone receives the coordinates as record `T-EXT-<n>`. The cue is
  never evidence, and its confirmation timeout waits until a drone is on site (Phase 4).
- **Separation**: the monitor predicts each peer's position from its heartbeat over
  `separation_horizon`. A drone without a task yields to one with a task; otherwise the higher ID
  yields. Yielding means holding position until the path is clear (`SEPARATION_YIELD`). The
  world separately measures true separation, for evaluation only.
- **Baseline** (`allocation/baseline.py`): after the run, the same cost and planner are applied
  to the true drone states at the moment allocation started; `World.baseline_results()` reports
  agreement and match. It never runs during the simulation.
- **Dashboard**:
  - a target panel showing the owner's (or lowest ID's) view with an "agents agreeing: n/m" count,
    highlighted in red when the drones disagree;
  - a bid table (t_go, E_need, battery after, cost, winner, or the reason a drone can't bid);
  - a message log with a type filter and send/delivery/drop counters;
  - fading message lines between drones, a dashed radio-range circle for the selected drone, and
    owner labels;
  - a separation-violation counter.

### Acceptance check
| Criterion | Result |
|---|---|
| `default`: every live agent commits the same owner, matching the baseline | All four commit **D2** within 2 s of the first bid (88.2 → 90.2 s, 25 auction messages). The baseline also picks D2. D3 reports `mode_RETURNING`. |
| `low_battery`: the nearest drone reports `feasible=false` and another wins | D1, the nearest, reports `insufficient_energy`; all four commit **D2**, matching the baseline |
| `limited_comms`: agreement through relay | All four commit **D1**. D4 received **0** messages from D1 and learned the target from D3. |
| `duplicate_reports`: all agents converge on one target ID | D1, D2 and D3 create `T-D1-001`, `T-D2-001` and `T-D3-001` on the same tick. After the MERGEs every agent holds only `T-D1-001`, and all agree on owner D2. |
| `external_cue`: the cue is allocated while `confirmation=PENDING` | All four hold `T-EXT-001` as ALLOCATED and PENDING, via row 4. It is never CONFIRMED, and owner D3 matches the baseline. |
| The owner is announced but does not fly to the target | No drone enters ENROUTE or INSPECTING; the owner keeps patrolling with `current_task` set |

Across all scenarios there are 0 geofence violations and 0 true separation violations, and every
allocation matches the baseline.

**Tests:** 316 passed.
- New unit tests: `test_cost`, `test_consensus` (random bids with random latency, reordering and
  20 % loss over 25 seeds; line topology relay; stale epochs; conflicting claims; merge),
  `test_network`, `test_separation`, and new peer-message tests in `test_tracker`.
- New simulation tests: `test_phase3_allocation` and `test_decentralization`. The latter checks
  that no agent holds a reference to another agent, the world, ground truth, the network, the
  sensors or the scenario; that peer tables and peer bids come only from delivered messages;
  that with the radio off an agent knows nothing about the others; and that duplicate and stale
  messages are dropped.
- Phase 2 tests that check purely local tracking now run with the radio off (1 m range). With the
  radio off, a drone that hears no peers allocates the target to itself.

### Phase 3 decisions to review
1. **Records from peers keep their original `source`** (`sensor` or `external_cue`) and get a
   separate `relayed=True` flag. The plan says "source=relayed", but replacing the source would
   lose the fact that a record is an external cue, and the cue's allocation path (row 4) depends
   on it.
2. **Confirmation evidence is shared.** OBSERVATION sharing is limited to 1 Hz, so peers might
   never collect 3 observations within the 5 s window. So TARGET_STATE (and each BID) carries the
   observations that met the criteria, and each receiver re-checks the 7.5 criteria on that
   evidence itself. It never adopts "CONFIRMED" on trust.
3. **Inspection point for bidding**: `inspection_standoff` (15.8 m at a 60 m sensor radius) is
   the distance at which the noise-free confidence model gives `inspection.min_confidence`. A
   point at the very edge of `r_eff` would have a confidence of about 0, so the inspection could
   never succeed. A larger `r_eff` still gives a shorter route, as 8.2 intends.
4. **Committed agents repeat CLAIM/ACK** to peers that are still bidding (at most once per round),
   so a peer that started late or lost messages can still collect a reply from every alive peer
   without waiting for the 10 s timeout.
5. **Separation counts in 3D** (`min_separation` 25 m + buffer 10 m). Grounded peers (parked or
   landed) are ignored. A FAILED drone stops sending heartbeats and drops out of the prediction
   after `miss_limit` periods. A drone hovering in its place could then be passed too closely;
   the world's true-separation measurement would report that. `duplicate_reports` now stacks its
   drones 36 m apart vertically, so they can all pass over GT-1 together without yielding.
6. **Thin feasibility margins are allowed.** In `default`, a Shift+click cue near D3 gives D3
   `batt_after` = 20.06 Wh, just above the 20 Wh reserve, so D3 wins. Bids are computed once per
   epoch while drones keep patrolling (8.2). If the owner later reaches its reserve, Phase 4's
   `RELEASE(battery_reserve)` and reallocation cover it.
7. **Speed:** the dashboard snapshot is built only when something reads it. A 400 s default run
   takes about 3 s headless here.

---

## Phase 4 report

### What was built
- **The owner's mission** (`drone/mission.py`). After the commit, the winner:
  - computes the inspection (engagement) point, using the same method as its bid;
  - flies a validated route there (`ENROUTE`, row 7), every step still checked by the monitor;
  - locks on when it is in the inspection region with a fresh track (`INSPECTING`, row 9; row 8
    if it is already in position);
  - counts lock time only while all the 7.10 conditions hold: CONFIRMED, inside `r_eff`,
    geofence-valid, FRESH track, confidence ≥ `inspection.min_confidence`;
  - after `inspection.duration`, writes its logbook entry, broadcasts `TARGET_LOGGED`, goes
    LOGGED → COMPLETED (rows 14–15), and flies back to the nearest patrol waypoint
    (`RETURNING` → `PATROL`).
- **Tracking loss and reacquisition**: when the track goes LOST, the owner goes TRACKING_LOST →
  REACQUIRING (rows 10–11) and flies a square spiral (`reacquire.leg_step`). Every leg is
  planned and validated, and legs outside the geofence are skipped. Once the target is seen
  again it goes back to TRACKING (row 12) and the owner returns to an inspection point of the
  new estimate. After `reacquire.timeout` it cancels (row 13) or releases for reallocation,
  depending on `timeout_action`. `timer_on_loss: pause` keeps the seconds already counted.
- **Release and reallocation (the "swap")**: the owner releases the task when:
  - its battery reserve trips (`RELEASE(battery_reserve)`, then it flies home);
  - it cannot find an inspection point or a route (`route_unreachable`);
  - a reacquisition times out with `reallocate`.

  Any agent also reopens the task if the owner has not been heard from for too long: heartbeat
  times are passed between drones, so a drone out of direct radio range is not wrongly declared
  dead. A reopen is row 16: epoch + 1, and the old owner is excluded for the new epoch. The
  others bid again, and after `max_reallocations` the task is cancelled.
- **Agreement on progress**: every owner transition is broadcast (`TARGET_STATE`) and relayed
  once. Peers adopt it by the 7.8 rule: a terminal state if the epoch is ≥ theirs, a higher
  epoch, or same-epoch progress from the owner itself. Owner updates carry a version number, so
  messages that arrive out of order never move a view backwards.
- **External cues**: the owner verifies on site. Its own sensor must meet the confirmation
  criteria before anything is logged. If nothing is seen within `confirmation.timeout` of
  arriving, the cue is CANCELLED with `cue_not_confirmed` (row 17) and never logged.
- **Target log** (`logging_/target_log.py`): each agent keeps its own logbook. At the end the
  logbooks and every agent's final views are merged by target ID, with aliases resolved, into
  `target_log.csv` with the section 13.2 columns. `messages_used` counts every message about the
  target, and duplicate logged inspections are marked `duplicate_inspection`.
- **Engage missions** (`mission.neutralize_on_complete`, a new config section): when the owner
  logs the target, the world marks the matching ground-truth target NEUTRALIZED
  (`TARGET_NEUTRALIZED`), and it produces no more observations.
- **Dashboard**:
  - an owner lock line (dashed while flying, solid red with a ring when locked) and a
    `LOCK n/10 s` timer;
  - two "WHY <winner>" lines under the bids;
  - a larger target crosshair, a red ✕ "NEUTRALIZED" marker and a green "MISSION COMPLETE" or
    "TARGET LOGGED" banner;
  - mission modes and target states in the legend;
  - the EVENTS panel now keeps the important events (it used to be emptied by radio traffic).

### Acceptance check
| Criterion | Result |
|---|---|
| `default`: owner flies a valid route, inspects for `inspection.duration`, logs, returns to patrol; one COMPLETED row | D2 commits at 89.7 s, locks at 90.6 s, logs at 100.6 s after 10.0 s of lock (mean confidence 0.79), and is back on patrol at 103.4 s. `target_log.csv` has one COMPLETED row; the estimate is within 1 m of GT-1. |
| `tracking_loss`: track lost and reacquired, timer paused | Lost at 95.7 s with 3 s counted, spiral search, reacquired at 101.1 s, timer resumed from 3 s, logged at 110.5 s |
| `target_in_restricted`: inspected from outside the zone or cancelled with `no_feasible_bidders` | The cue deep inside RZ-1 is CANCELLED with `no_feasible_bidders` (every bid: `no_inspection_point`). The edge target is never confirmed, so it is never inspected. |
| A cue at a location with no target ends CANCELLED and is never logged | `external_cue`: the second cue is CANCELLED with `cue_not_confirmed`, and there is no TARGET_LOGGED for it. The first cue is verified on site and logged. |
| (demo) four drones, parameters decide, one engages | `mission`: D1 `insufficient_energy`, D2 wins (matches the baseline), GT-1 is NEUTRALIZED at 50.6 s by D2 with an estimate error of 0.6 m |
| (swap) owner lost mid-mission | D2 failed: the others reopen after about 7 s (epoch 1, D2 excluded) and D4 completes the mission. D2 battery-reserve: D2 releases at once and D4 is chosen 1.6 s later. |

Across all scenarios there are 0 geofence and 0 true separation violations. No target is ever
logged without confirmation and a full inspection; a test checks this over 8 scenarios.

**Tests:** 345 passed. New: `tests/sim/test_phase4_inspection.py` (every criterion above;
reacquire timeout with cancel and with reallocate; `max_reallocations`; the logging invariant
over 8 scenarios) and `tests/unit/test_mission.py` (spiral, the new rows, the inspection guard,
logbook merge and duplicate marking). Earlier tests that assumed "the owner does not fly yet"
now check the same properties together with the completed mission.

### Phase 4 decisions to review
1. **Engagement point at the confidence midpoint.** The inspection point is now where the
   noise-free confidence is halfway between `min_confidence` (0.6) and `base` (0.95): 7.6 m at a
   60 m sensor radius. At the threshold itself, about half the noisy observations fall below 0.6
   and the lock timer barely advances.
2. **Three small additions to the lifecycle table**, each commented in `target_state.py`:
   - row 10 also from ENROUTE: the owner arrives but the track is already LOST;
   - row 17 also from ENROUTE: a cue where there is nothing to see;
   - row 6 also from the active states, when a reopen is refused because `max_reallocations` is
     reached (8.4 says the task is then cancelled, but the table had no row for it).
3. **`default.yaml`: `association.ignore_after_complete: 400`** instead of the plan's 60. GT-1
   stays where it is in that scenario, so after 60 s D2 re-detects and re-inspects it on every
   lap (3 COMPLETED rows), but the plan's acceptance check asks for one row. The schema default
   stays 60, and other scenarios use it.
4. **Owner-failure detection uses passed-on heartbeat times** (`seen` in HEARTBEAT): the limit is
   `miss_limit × period + max_message_age`. Without this, a drone that never heard the owner
   directly (short radio range) would reopen the task.
5. **New config section `mission`** (`neutralize_on_complete`, default false). The original
   inspection behaviour is unchanged unless a scenario turns it on, as `mission.yaml` does.
6. After a reacquisition the owner flies back to an inspection point of the new estimate: the
   spiral can leave it up to 60 m away, where confidence is too low for the timer.
