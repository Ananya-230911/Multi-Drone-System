# Multi-Drone Decentralized Target Inspection (simulation)

A fully simulated, decentralized team of 3–4 drone agents that patrol an area, detect a
simulated target, agree among themselves which drone inspects it, navigate inside geofence
limits, track the target and log the inspection. The full design is in [`PLAN.md`](PLAN.md).

**Simulation only.** Nothing in this project connects to or commands a real drone or flight
controller.

**Current status: Phase 2 of 5 (target lifecycle, sensor simulation, geofencing, route safety).**
See the phase reports below.

> **About "confidence":** every confidence value in this project is a *simulated score* used to
> exercise the decision logic (PLAN.md 7.3). It does not represent the detection accuracy of any
> real sensor or detector.

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

Each run writes `output/<run_id>/events.jsonl` (one JSON event per line) and
`output/<run_id>/summary.json` (final drone states, each drone's target records, geofence
violations, false confirmations, and the SHA-256 of the event log).

### Scenarios

| File | What it shows (Phase 2) |
|---|---|
| `config/default.yaml` | 4 drones patrol; D2 passes 50 m from GT-1, detects it, holds, confirms it |
| `config/geofence_detour.yaml` | a long restricted wall; D1 confirms a target in the east, then its route home goes around the wall |
| `config/target_in_restricted.yaml` | target inside RZ-1; D1 sees it from the zone edge but may not overfly it, so the record is rejected |
| `config/false_observation.yaml` | no real target, only spurious observations; nothing may ever be confirmed |
| `config/duplicate_reports.yaml` | three drones fly over GT-1 within seconds; each keeps one local record (merged across agents in Phase 3) |

### Dashboard controls

| Key / mouse | Action |
|---|---|
| `SPACE` | pause / resume |
| `←` / `→` | slower / faster (0.25× to 8×) |
| `R` | reset the scenario (same seed) |
| `1`–`4` | select a drone |
| `F` | fail the selected drone |
| Left click on map | place a **ground-truth** target (drones still have to detect it) |
| `Shift` + click | external cue (logged now, applied from Phase 3) |
| `H` | hide / show all ground-truth targets |
| `G` | show / hide ground-truth markers |
| `L` | show / hide the legend |
| `Esc` | quit |

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
src/safety/monitor.py       runtime checks: next step, battery reserve, landing
src/logging_/event_log.py   JSON Lines event log (canonical, hashable)
src/viz/                    snapshot, map renderer, side panel, legend
tests/unit/                 algorithm tests (no world, no pygame)
tests/sim/                  headless scenario tests per phase
```

Empty packages (`allocation/`, `comms/`) are placeholders for Phase 3.

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
