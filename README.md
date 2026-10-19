# Multi-Drone Decentralized Target Inspection (simulation)

A fully simulated, decentralized team of 3–4 drone agents that patrol an area, detect a
simulated target, agree among themselves which drone inspects it, navigate inside geofence
limits, track the target and log the inspection. The full design is in [`PLAN.md`](PLAN.md).

**Simulation only.** Nothing in this project connects to or commands a real drone or flight
controller.

**Current status: Phase 1 of 5 (core simulation foundation).** See the phase report below.

## Install

Python 3.11 or newer.

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

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
`output/<run_id>/summary.json` (final drone states and the SHA-256 of the event log).

### Dashboard controls (Phase 1)

| Key | Action |
|---|---|
| `SPACE` | pause / resume |
| `←` / `→` | slower / faster (0.25× to 8×) |
| `R` | reset the scenario (same seed) |
| `1`–`4` | select a drone |
| `F` | fail the selected drone |
| `L` | show / hide the legend |
| `Esc` | quit |

Pressing `F` doesn't touch the drone directly. It queues a `fail_drone` scenario event, which
is applied on the next tick and logged as `SCENARIO_EVENT` (PLAN.md 4.1 rule 6).

## Tests

```bash
python -m pytest            # all tests (about 3 s)
python -m pytest tests/unit # algorithm tests only, no world and no pygame
```

## Layout

```
config/default.yaml       scenario: 4 drones, 1 ground-truth target (target used from Phase 2)
src/config.py             YAML -> typed dataclasses, full schema validation
src/main.py               entry point (dashboard or headless)
src/sim/world.py          sim clock and deterministic tick loop
src/sim/scenario.py       scheduled and operator-injected events
src/drone/agent.py        drone agent (one per drone, identical code)
src/drone/state.py        private drone state
src/drone/fsm.py          drone behaviour state machine
src/drone/battery.py      energy model and reserve rule
src/drone/navigation.py   3D waypoint follower
src/logging_/event_log.py JSON Lines event log (canonical, hashable)
src/viz/                  snapshot, map renderer, side panel, legend
tests/unit/               battery, navigation, config
tests/sim/                headless Phase 1 scenario tests
```

Empty packages (`tracking/`, `safety/`, `allocation/`, `comms/`) are placeholders for later phases.

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
   patrol routes are not validated against the geofence. Phase 2 adds `safety/` and replaces
   this route with a validated, planned one. The energy estimate already uses the route's
   waypoints, not a straight line, so it will switch to planned routes without changes.
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
   clamps at zero and the drone still flies home. Phase 2's monitor adds the critical-level
   landing.
6. **Two event types beyond the section 13.1 list:** `WAYPOINT_REACHED` and `HOME_REACHED`, so
   the Phase 1 log has enough content for the determinism hash to be meaningful.
7. Scenario event types for later phases (`external_cue`, `place_target`, `hide_target`,
   `show_target`) are validated, logged as `SCENARIO_EVENT` with `applied: false`, and ignored.
8. Pressing `R` resets the run. Only the run that is active when the window closes is written to
   `output/`.
