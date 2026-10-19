# Multi-Drone Decentralized Target Inspection — Master Implementation Plan

> **Revision 2.** Supersedes revision 1. See section 18 for the change log and design decisions.
>
> Hand this file to Claude Code as `PLAN.md` in an empty project folder and say:
> "Read PLAN.md and build Phase 1 only. Stop at the Phase 1 review point."

---

## 1. Project objective

Build a **fully simulated**, decentralized multi-drone system with 3–4 autonomous drone agents. The drones:

1. patrol an operating area,
2. detect and confirm a simulated target,
3. each evaluate how suitable they are for the task,
4. exchange messages and agree on which drone owns the task (no leader, no human in the loop),
5. navigate safely inside geofence limits,
6. maintain a simulated track of the target (stable ID, latest observation, freshness, confidence),
7. log the inspection result only after the inspection conditions are satisfied.

In this plan, "target locking" means keeping a stable target ID and an active, fresh track on it. It does not mean anything beyond observation and logging.

**Difference from the earlier UAS-SYSTEM project:** that project was one drone navigating to a point. This project is about decentralized multi-agent decision-making, target-state management, tracking, and geofence-aware navigation.

**Simulation only.** No code in this project connects to, or sends commands to, real drones or flight controllers.

### 1.1 Scope of this plan (Phases 1–5)
- 3–4 drones, **one ground-truth target at a time**, stationary target position.
- 2D map with altitude as a separate value (2.5D).
- Fully decentralized allocation. No central coordinator ever chooses the winner.
- Geofence and safety layer enforced as hard constraints.
- Visible dashboard including the message log between drones.

### 1.2 Out of scope for Phases 1–5
- Multiple simultaneous targets, moving targets, real computer-vision detection, Gazebo/AirSim/ROS 2, real hardware. These are listed in section 17 (Future work).

---

## 2. Technology stack

| Purpose | Choice | Why |
|---|---|---|
| Language | Python 3.11+ | Readable, fast to iterate, easy to demo |
| Math | `numpy` | Vectors and distances |
| Geometry | `shapely` 2.x | Robust polygon containment, segment intersection and buffering for the geofence. Hand-written polygon code is a common source of edge-case bugs |
| Visualization | `pygame` | Real-time 2D dashboard; runs on any laptop |
| Config | `pyyaml` | Every scenario is a YAML file |
| Testing | `pytest` | Unit and simulation tests |
| Plots | `matplotlib` | Phase 5 evaluation plots |
| Output | JSON Lines + CSV | Event log and target log for the report |

**Change from revision 1:** `shapely` added (justification above). Everything else is unchanged.

---

## 3. Folder structure

```
multi_drone_inspection/
├── PLAN.md
├── README.md
├── requirements.txt
├── config/
│   ├── default.yaml                 # P1+  4 drones, 1 target, sensor detection
│   ├── geofence_detour.yaml         # P2+  restricted zone between drones and target
│   ├── target_in_restricted.yaml    # P2+  target inside a zone; inspect from the edge or cancel
│   ├── false_observation.yaml       # P2+  spurious observations must not confirm
│   ├── duplicate_reports.yaml       # P2+  several drones detect the same target at once
│   ├── external_cue.yaml            # P3+  coordinates arrive from scenario, not a sensor
│   ├── low_battery.yaml             # P3+  nearest drone cannot afford the round trip
│   ├── limited_comms.yaml           # P3+  short radio range, multi-hop relay needed
│   ├── partition.yaml               # P5   network split, then heal
│   ├── tracking_loss.yaml           # P4+  target hidden for an interval, then visible
│   └── drone_failure.yaml           # P5   owner fails mid-task, reallocation
├── src/
│   ├── main.py                      # python -m src.main --config config/default.yaml
│   ├── evaluate.py                  # P5  batch runs + metrics + plots
│   ├── config.py                    # load YAML -> typed dataclasses, validate
│   ├── sim/
│   │   ├── world.py                 # sim clock, tick loop, owns ground truth
│   │   ├── ground_truth.py          # true target(s) and visibility schedule (agents never read this)
│   │   └── scenario.py              # scheduled events: cues, failures, partitions
│   ├── drone/
│   │   ├── agent.py                 # Drone agent; orchestrates the modules below
│   │   ├── state.py                 # private state (position, alt, battery, mode)
│   │   ├── fsm.py                   # drone behaviour state machine (section 6.2)
│   │   ├── battery.py               # energy model
│   │   ├── sensor.py                # simulated sensor -> Observation objects
│   │   └── navigation.py            # waypoint follower (horizontal + vertical)
│   ├── tracking/
│   │   ├── observation.py           # Observation dataclass + validation
│   │   ├── target_state.py          # target lifecycle FSM, transition table, guards
│   │   └── tracker.py               # target records, association/dedup, confirmation, freshness
│   ├── safety/
│   │   ├── geofence.py              # boundary, restricted zones, altitude limits, margins
│   │   ├── route_validator.py       # validate points, segments, full routes
│   │   ├── planner.py               # geofence-aware path planning (visibility graph + A*)
│   │   └── monitor.py               # runtime safety: next-step check, separation, battery reserve
│   ├── allocation/
│   │   ├── cost.py                  # cost and feasibility, with full breakdown
│   │   ├── auction.py               # decentralized auction + consensus + epochs
│   │   └── baseline.py              # centralized optimum for EVALUATION ONLY
│   ├── comms/
│   │   ├── messages.py              # message dataclasses
│   │   └── network.py               # simulated radio: range, latency, loss, partitions
│   ├── logging_/
│   │   ├── event_log.py             # every event -> JSON Lines
│   │   └── target_log.py            # per-agent logbooks -> merged CSV
│   └── viz/
│       ├── snapshot.py              # read-only snapshot built each frame
│       ├── renderer.py              # map layer
│       ├── panel.py                 # side panel
│       └── legend.py
├── tests/
│   ├── unit/                        # pure algorithm tests, no world, no pygame
│   │   ├── test_battery.py
│   │   ├── test_geofence.py
│   │   ├── test_route_validator.py
│   │   ├── test_planner.py
│   │   ├── test_sensor.py
│   │   ├── test_observation.py
│   │   ├── test_target_state.py
│   │   ├── test_tracker.py
│   │   ├── test_cost.py
│   │   ├── test_consensus.py
│   │   └── test_network.py
│   └── sim/                         # full headless simulation runs
│       ├── test_phase1_patrol.py
│       ├── test_phase2_detection_safety.py
│       ├── test_phase3_allocation.py
│       ├── test_phase4_inspection.py
│       ├── test_phase5_robustness.py
│       └── test_decentralization.py
└── output/                          # generated, gitignored
```

The `P1+`, `P2+` tags show the first phase that uses each scenario. Sections 14 and 15 list which tests belong to which phase.

---

## 4. Architecture

```
┌──────────────────────────── World (sim clock, dt = 0.1 s) ─────────────────────────────┐
│                                                                                         │
│  GroundTruth (true target, visibility)        Scenario (scheduled events)               │
│        │ only via sensor model                       │ only via defined injection points│
│        ▼                                             ▼                                  │
│  ┌──────────── Drone agent (one per drone, identical code) ─────────────┐               │
│  │  state (private)   battery   sensor   tracker   target_state FSM    │               │
│  │  allocation (cost + auction)   safety (geofence, validator, planner, │               │
│  │  monitor)   navigation   drone FSM   logbook                         │               │
│  └───────────────────────────────┬──────────────────────────────────────┘               │
│                                  │ outbox / inbox only                                  │
│              ┌───────────────────▼────────────────────┐                                 │
│              │ CommNetwork (range, latency, loss,     │                                 │
│              │ partitions, dedup)                     │                                 │
│              └────────────────────────────────────────┘                                 │
│                                                                                         │
│  EventLog (append-only)                      Visualizer (reads Snapshot, writes nothing)│
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

### 4.1 Information boundaries (decentralization rules)
1. A drone reads **only** its own private state, its own inbox, its own sensor output, and the static shared configuration (map, geofence, its own parameters, the list of drone IDs).
2. A drone never holds a reference to another drone object, to `GroundTruth`, or to `World`.
3. All information about other drones and about targets reaches a drone as **messages** or **its own observations**.
4. Ground-truth target IDs (`GT-1`) are never visible to agents. Agents create their own belief IDs (section 7.4).
5. The visualizer, event log and evaluator may read everything, but only through a read-only `Snapshot` built after the tick. They never call agent methods or write agent state.
6. Operator/scenario injections (place a ground-truth target, send an external cue, fail a drone, cut a link) go through `Scenario` and are logged as `SCENARIO_EVENT`. They are test inputs, not coordination.

### 4.2 Tick order (deterministic)
1. `scenario.apply(t)` — due scenario events.
2. `network.deliver(t)` — move due messages into inboxes. Agents are processed in sorted ID order.
3. For each drone in ID order, `drone.step(t, dt, sensor_view)`:
   1. read inbox → update tracker, auction tables, peer table (heartbeats)
   2. sense → produce observations → tracker
   3. update target lifecycle (local view)
   4. allocation step (bid / compare / commit)
   5. drone FSM decides the mode and goal
   6. planner/validator produce or check the route
   7. safety monitor checks the next step; may override with SAFETY_HOLD or RETURN
   8. navigation moves the drone; battery drains
   9. outbound messages → network
4. `world.record()` → event log, Snapshot.
5. Visualizer draws the Snapshot (skipped when headless).

`sensor_view` is a function supplied by the world that applies the sensor model to ground truth for that drone's pose. It returns `Observation` objects only, never ground-truth objects.

---

## 5. Key interfaces

These signatures are the contracts between modules. Exact types are dataclasses in the listed files.

```python
# tracking/observation.py
@dataclass(frozen=True)
class Observation:
    obs_id: str              # f"{observer}-{seq}"
    observer: str            # drone ID, or "EXT" for an external cue
    source: Literal["sensor", "external_cue", "relayed"]
    t: float                 # time of observation (sim seconds)
    pos: tuple[float, float] # estimated target position (noisy)
    alt: float | None        # estimated target altitude (ground = 0.0)
    confidence: float        # SIMULATED score in [0, 1]; see 7.3
    is_spurious: bool        # ground truth flag; hidden from agents, used by tests/eval only

def validate(obs: Observation, now: float, geofence: Geofence, cfg) -> tuple[bool, str]: ...
```

> `is_spurious` is stripped before an observation is handed to an agent; agents receive an `AgentObservation` without it.

```python
# tracking/target_state.py
class TargetState(Enum): UNKNOWN, DETECTED, CONFIRMED, ALLOCATED, ENROUTE, TRACKING,
                         TRACKING_LOST, REACQUIRING, LOGGED, COMPLETED, CANCELLED
class Confirmation(Enum): PENDING, CONFIRMED, REJECTED

def can_transition(rec: TargetRecord, to: TargetState, ctx) -> tuple[bool, str]: ...
def apply_transition(rec, to, ctx, reason) -> TransitionEvent: ...   # raises on illegal

# tracking/tracker.py
class Tracker:
    def ingest(self, obs: AgentObservation, now) -> TrackUpdate
    def record(self, target_id) -> TargetRecord
    def freshness(self, target_id, now) -> Literal["FRESH", "STALE", "LOST"]
    def merge_remote(self, msg) -> None          # OBSERVATION / TARGET_STATE / MERGE messages
    def check_confirmation(self, target_id, now) -> Confirmation

# safety/geofence.py
class Geofence:
    def contains_point(self, x, y, alt) -> bool
    def violation(self, x, y, alt) -> str | None # reason code or None

# safety/route_validator.py
def validate_segment(a: Waypoint, b: Waypoint, fence) -> ValidationResult
def validate_route(route: list[Waypoint], fence) -> ValidationResult

# safety/planner.py
def plan(start: Waypoint, goal_region: InspectionRegion | Waypoint, fence, cfg) -> Route | None
def inspection_point(target_pos, sensor, fence, cfg) -> Waypoint | None

# safety/monitor.py
def check_step(state, next_pos, peers, fence, battery, cfg) -> SafetyVerdict
# verdicts: OK, HOLD, REPLAN, RETURN, LAND  (+ reason)

# allocation/cost.py
def evaluate(drone_view, target_record, fence, cfg) -> Bid   # Bid has feasible flag + breakdown
# allocation/auction.py
class Auction:
    def on_target_ready(self, rec, now) -> list[Message]
    def on_message(self, msg, now) -> list[Message]
    def tick(self, now) -> AuctionStatus     # BIDDING / COMMITTED(owner) / NO_BIDDERS / TIMEOUT

# comms/network.py
class CommNetwork:
    def send(self, msg: Message, now) -> None
    def deliver(self, now) -> None

# drone/agent.py
class Drone:
    def step(self, now, dt, sensor_view) -> None
```

---

## 6. Drone agent

### 6.1 Private state
```
id, pos (x, y), alt, vel, heading
battery_wh, capacity_wh, reserve_pct
mode (drone FSM state), current_task (target_id or None), task_epoch
home (x, y, alt), patrol_route (list of waypoints)
peer_table: last heartbeat per peer (pos, alt, battery_pct, mode, task, t_received)
params: max_speed, climb_rate, sensor, comm_range
```

### 6.2 Drone behaviour state machine
This describes what the **drone** is doing. It is separate from the **target lifecycle** (section 7), which describes the task.

| Mode | Meaning |
|---|---|
| `PATROL` | Following its patrol route. Auctions run in the background; patrolling continues while bidding |
| `CONFIRM_HOLD` | Detected a possible target and holds position (if safe) to gather confirming observations, for at most `confirmation.hold_max` s |
| `ENROUTE` | Owner navigating to the inspection point |
| `INSPECTING` | Owner at the inspection point, observing the target |
| `REACQUIRING` | Owner searching for a lost target |
| `RETURNING` | Flying home or back to patrol along a validated route |
| `SAFETY_HOLD` | Stopped/hovering because the next step or route became invalid; replanning |
| `LANDED` | Simulated emergency landing: no valid route home and battery critical |
| `FAILED` | Injected failure; sends nothing, does nothing |

| From | To | Trigger |
|---|---|---|
| PATROL | CONFIRM_HOLD | Own sensor produced a valid observation of an unconfirmed record, `detector_holds=true`, and hold position is geofence-valid |
| CONFIRM_HOLD | PATROL | Record CONFIRMED or CANCELLED, or `hold_max` elapsed |
| PATROL / CONFIRM_HOLD | ENROUTE | This drone committed as owner and a validated route exists |
| ENROUTE | INSPECTING | Arrived at inspection point (within `arrival_tolerance` and `alt_tolerance`) |
| INSPECTING | REACQUIRING | Track becomes LOST |
| REACQUIRING | INSPECTING | Fresh valid observation inside inspection region |
| INSPECTING | RETURNING | Target LOGGED → COMPLETED |
| REACQUIRING | RETURNING | Reacquisition timed out |
| any active | RETURNING | Battery reserve protection triggers (6.4), task released |
| any moving | SAFETY_HOLD | Monitor verdict HOLD or REPLAN |
| SAFETY_HOLD | previous mode | Replan succeeded and new route validated |
| SAFETY_HOLD | RETURNING | `replan_limit` attempts failed; task released |
| RETURNING | PATROL | Reached home or the nearest patrol waypoint |
| any | LANDED | No valid route home and battery below critical level |
| any | FAILED | Scenario failure event |

### 6.3 Battery model
```
k_h          Wh per meter, level flight
k_c          Wh per meter climbed
k_d          Wh per meter descended
hover_power  Wh per second while holding / inspecting
reserve      reserve_pct × capacity_wh   (never planned below this)
critical     critical_pct × capacity_wh  (triggers LANDED if no route home)
```
Energy estimates always use **planned route length**, not straight-line distance.

### 6.4 Battery reserve protection (continuous)
Every tick the monitor computes `E_home` = energy to fly the validated route from the current position to home.
If `battery − E_home < reserve`: the drone switches to `RETURNING`. If it owns a task it broadcasts `RELEASE(reason="battery_reserve")` first.

### 6.5 Navigation
- Kinematic point mass: horizontal speed ≤ `max_speed`, vertical speed ≤ `climb_rate`, horizontal and vertical motion simultaneous.
- Follows a list of 3D waypoints produced by the planner. Never moves toward a goal without a validated route.
- Altitude profile: climb/descend to `cruise_alt` at the start of the route, cruise, then descend/climb to `inspection.standoff_alt` on the final leg. All altitudes clamped inside geofence altitude limits.
- Patrol routes come from config and are validated at startup. An invalid patrol route is a configuration error and the run stops with a clear message.

---

## 7. Target detection, lifecycle and tracking

### 7.1 Three different ways a target becomes known
| Source | What it is | Record created as | Confirmed? |
|---|---|---|---|
| **Scenario ground truth** | A true target placed in the world by config or a mouse click | No record. Agents know nothing until a sensor sees it | n/a |
| **Sensor detection** | A drone's own simulated sensor produced a valid observation | `DETECTED`, `source=sensor`, `confirmation=PENDING` | Not until 7.5 criteria are met |
| **Reported by another agent** | A message (`TARGET_DETECTED`, `OBSERVATION`) from a peer | Copy of the peer's record, `source=relayed` | Only if the peer's evidence meets the criteria |
| **External cue** | Coordinates injected by the scenario ("possible target here") | `DETECTED`, `source=external_cue`, `confirmation=PENDING` | Only after the owner observes it on site |

Receiving coordinates never sets `CONFIRMED` and never permits logging. Only sensor observations count as evidence.

### 7.2 Sensor simulation (`drone/sensor.py`)
Per-drone, configurable:
- `radius` — maximum horizontal detection range (m).
- `half_fov_deg` — downward-looking sensor; ground footprint radius = `alt × tan(half_fov)`.
- **Effective radius** `r_eff = min(radius, alt × tan(half_fov))`.
- `alt_band [min, max]` — sensor only works inside this altitude band.
- `rate_hz` — observations per second when the target is in view.
- `p_detect` — probability a sample produces an observation when in view.
- `position_noise_std` — Gaussian noise on reported position (m).
- `false_obs_rate` — spurious observations per second (testing; default 0).

An observation is generated when **all** hold: the target is visible (not in a hidden interval), drone altitude is inside `alt_band`, horizontal distance ≤ `r_eff`, and the `p_detect` draw succeeds. Spurious observations appear at a random point inside the current footprint.

### 7.3 Simulated confidence
```
confidence = clamp(base × (1 − d / r_eff) ^ falloff_exp + N(0, noise_std), 0, 1)
spurious observations: confidence drawn from U(spurious_conf_range)
category: LOW < low_threshold ≤ MEDIUM < medium_threshold ≤ HIGH
```
This number is a **simulation parameter** used to exercise the decision logic. It does not represent real-world detection accuracy, and the README must say so.

### 7.4 Stable target IDs, association and duplicate prevention (`tracking/tracker.py`)
- **ID format:** `T-<creator>-<seq>` (e.g. `T-D2-001`); external cues use `T-EXT-<seq>`.
- **Association:** a new observation within `association.radius` of an existing non-terminal record's estimated position is attached to that record. Otherwise a new record is created.
- **Duplicates from concurrent detection:** two drones can create `T-D1-001` and `T-D3-001` for the same target before they hear each other. When any agent holds two non-terminal records within `association.radius`, it merges them:
  - **surviving ID = the lexicographically smallest ID.** Every agent applies the same rule, so all converge on the same ID.
  - the other ID is kept as an **alias**; messages using the alias are redirected.
  - observations are combined; lifecycle state = the more advanced of the two; if both have allocation tables, they are compared with the normal bid rule (8.3).
  - the merging agent broadcasts `MERGE {survivor, alias}`.
- Records in a terminal state are never merged into; a new observation near a COMPLETED target creates no new record for `association.ignore_after_complete` seconds (avoids re-inspecting the same target immediately).

### 7.5 Confirmation criteria
A record moves to `confirmation=CONFIRMED` when, inside a sliding window of `confirmation.window` seconds:
- at least `min_observations` valid sensor observations are associated with it,
- their mean confidence ≥ `min_mean_confidence`,
- they come from at least `min_distinct_observers` drones (default 1).

If the window since detection passes `confirmation.timeout` without meeting the criteria, `confirmation=REJECTED`.
Peer observations count when received as `OBSERVATION` messages; each observation is counted once (dedup by `obs_id`).

### 7.6 Observation validation (`tracking/observation.py`)
An observation is rejected (logged as `OBSERVATION_REJECTED` with reason) if: timestamp is in the future or older than `max_observation_age`; position is outside the operating boundary; confidence outside [0, 1]; observer is unknown; or it is a duplicate `obs_id`.

### 7.7 Track maintenance
For each record the tracker keeps: stable ID and aliases, latest observation and time, estimated position (exponential average of recent valid observations, weight `track.smoothing`), last confidence and category, freshness, and the transition history.

Freshness (`now − t_last_obs`):
- `FRESH` ≤ `track.stale_after`
- `STALE` ≤ `track.lost_after`
- `LOST` > `track.lost_after`

Only the owner's own sensor observations, or observations from peers within `max_observation_age`, refresh the track.

### 7.8 Target lifecycle state machine (`tracking/target_state.py`)

Each agent holds its **own local view** of each target record. Views converge through messages; they are not shared memory.

States: `UNKNOWN, DETECTED, CONFIRMED, ALLOCATED, ENROUTE, TRACKING, TRACKING_LOST, REACQUIRING, LOGGED, COMPLETED, CANCELLED`.

Two attributes travel with the state:
- `confirmation ∈ {PENDING, CONFIRMED, REJECTED}`
- `epoch` — allocation round number, starts at 0, increases on every reallocation.

`UNKNOWN` is the implicit state before a record exists. `COMPLETED` and `CANCELLED` are **terminal and sticky**.

#### Transition table

| # | From | To | Trigger | Guard | Who may cause it |
|---|---|---|---|---|---|
| 1 | UNKNOWN | DETECTED | First valid sensor observation, peer report, or external cue | — | any agent |
| 2 | DETECTED | CONFIRMED | Confirmation criteria met | `source ≠ external_cue` or evidence is sensor observations | any agent holding the evidence |
| 3 | DETECTED | CANCELLED | `confirmation=REJECTED` | `source ≠ external_cue` | any agent |
| 4 | DETECTED | ALLOCATED | Consensus committed | `source = external_cue` only (verification task, 7.9) | each agent locally |
| 5 | CONFIRMED | ALLOCATED | Consensus committed | ≥1 feasible bid | each agent locally |
| 6 | CONFIRMED / DETECTED(cue) | CANCELLED | `allocation_timeout` with no feasible bidders, or `max_reallocations` exceeded | — | any agent |
| 7 | ALLOCATED | ENROUTE | Owner has a validated route and starts moving | route valid | owner only |
| 8 | ALLOCATED | TRACKING | Owner already at a valid inspection point with a FRESH track | inspection point valid | owner only |
| 9 | ENROUTE | TRACKING | Owner inside inspection region with a FRESH valid observation | — | owner only |
| 10 | TRACKING | TRACKING_LOST | Track freshness becomes LOST | — | owner only |
| 11 | TRACKING_LOST | REACQUIRING | Owner starts reacquisition | battery and route allow | owner only |
| 12 | REACQUIRING | TRACKING | Fresh valid observation in inspection region | — | owner only |
| 13 | REACQUIRING | CANCELLED | `reacquire_timeout` and `reacquire_timeout_action=cancel` | — | owner only |
| 14 | TRACKING | LOGGED | Inspection conditions met (7.10) | `confirmation=CONFIRMED` | owner only |
| 15 | LOGGED | COMPLETED | Logbook entry written and `TARGET_LOGGED` broadcast | — | owner only |
| 16 | ALLOCATED, ENROUTE, TRACKING, TRACKING_LOST, REACQUIRING | CONFIRMED (cue: DETECTED) **reopen** | Owner `RELEASE`, owner heartbeat timeout, route unreachable, or `reacquire_timeout_action=reallocate` | reallocations < `max_reallocations`; `epoch += 1`; failed owner excluded for that epoch | owner (release) or any agent (timeout) |
| 17 | TRACKING, REACQUIRING (cue) | CANCELLED | Owner's on-site observations give `confirmation=REJECTED` | `source = external_cue` | owner only |

Rules:
- Any transition not in the table is **illegal**. `apply_transition` raises; the agent logs `ILLEGAL_TRANSITION` and keeps its state.
- **Adopting remote state:** an agent adopts a `TARGET_STATE` from a peer only if (a) the message epoch is higher than local, or (b) the epoch is equal and the sender is the owner in the local winner table and the transition is legal from the local state. Terminal states are adopted whenever the epoch is ≥ local.
- **States not visited** are normal: a cue target skips `CONFIRMED`; an owner already in position skips `ENROUTE`; a smooth inspection never visits `TRACKING_LOST`.

#### 7.9 External-cue (verification) path
A cue cannot be confirmed by a drone that is not there, so it is allocated while still `DETECTED` (row 4) as a **verify-and-inspect** task. The owner flies to the inspection point, and its own observations run the normal confirmation criteria. `CONFIRMED` evidence is required before `LOGGED` (row 14 guard). If the owner's observations produce `REJECTED`, the task is `CANCELLED` with reason `cue_not_confirmed` (row 17).

#### 7.10 Inspection conditions (guard for TRACKING → LOGGED)
All must hold:
1. `confirmation = CONFIRMED`;
2. the owner is inside the inspection region (horizontal distance ≤ its `r_eff` at current altitude) and geofence-valid;
3. accumulated time with a `FRESH` track and confidence ≥ `inspection.min_confidence` ≥ `inspection.duration`;
4. `inspection.timer_on_loss` defines what happens if the track goes stale: `pause` (default) keeps accumulated time, `reset` restarts it.

### 7.11 Reacquisition
On `TRACKING_LOST` the owner (if battery and geofence allow) moves to the last estimated target position's inspection point, then flies a square-spiral search with leg step `reacquire.leg_step`, clipped to the geofence (every leg validated). It stops on a fresh observation (row 12) or after `reacquire.timeout` (row 13 or 16 depending on `reacquire_timeout_action`).

---

## 8. Decentralized task allocation

### 8.1 Eligibility
A drone bids only when **all** hold:
- mode is `PATROL` or `CONFIRM_HOLD` (not owning another task, not RETURNING/SAFETY_HOLD/LANDED/FAILED);
- not excluded for this epoch;
- a geofence-valid inspection point exists and the planner finds a valid route to it **and** a valid route from it to home;
- energy is feasible (8.2).

Ineligible drones send `BID` messages with `feasible=false` and a reason, so the UI can explain why they did not compete. They never become a winner.

### 8.2 Cost function (`allocation/cost.py`)
```
L_go    = planned route length to inspection point (horizontal, m)
dz_go   = altitude changes along that route (climb and descent separately)
t_go    = L_go / max_speed + Σclimb / climb_rate + Σdescent / climb_rate
E_go    = k_h·L_go + k_c·Σclimb + k_d·Σdescent
E_insp  = hover_power · inspection.duration · inspection.energy_margin
E_home  = route energy from inspection point to home
E_need  = E_go + E_insp + E_home
batt_after = battery − E_need

feasible = route_go valid AND route_home valid AND batt_after ≥ reserve AND eligible

cost = w_time · t_go
     + w_energy · (E_need / capacity)
     + w_battery · (1 − batt_after / capacity)
bid  = −cost   (only if feasible)
```
- The sensor radius enters through the inspection point: the drone only needs to reach a point inside its `r_eff`, so a drone with a larger effective sensor radius has a shorter route.
- **Geofence is not a cost term.** It enters only through route feasibility (8.1) and route length. A drone with no valid route cannot bid, whatever its other numbers.
- Tie-break: equal bids (within `1e-9`) → lower drone ID wins. Every agent applies the same rule.
- Every term is included in the `BID` message as `breakdown` and shown in the UI and event log.
- A bid is computed once per epoch at the moment the drone first hears of the task. Drones keep patrolling while bidding, so bids are slightly stale; this is accepted and documented.

### 8.3 Auction and consensus (`allocation/auction.py`)
Single-task max-consensus auction (the single-task case of CBBA).

Each agent keeps a winner table per target and epoch: `(winner_id, winning_bid, epoch, updated_at)`.

1. **Start:** when the local record becomes allocatable (CONFIRMED, or DETECTED with `source=external_cue`), the agent computes its bid. If feasible and better than the table entry, it writes itself in.
2. **Broadcast:** it sends `BID {target_id, epoch, winner_id, winning_bid, own_bid, feasible, breakdown}` every `round_period` while bidding.
3. **Compare rule** (applied to every incoming entry): higher epoch wins; same epoch → higher bid wins; equal bid → lower winner ID wins. Changes trigger an immediate re-broadcast (this is the multi-hop relay).
4. **Commit:** an agent commits when its table has been unchanged for `consensus_rounds` consecutive rounds **and** it has received a `BID` for this epoch from every peer it currently believes alive (heartbeat within `miss_limit`). If `allocation_timeout` passes first, it commits to its current table entry if any feasible winner exists; otherwise row 6 (CANCELLED, `no_feasible_bidders`).
5. **Ownership:** the committed winner sends `CLAIM {target_id, epoch}` and moves to ENROUTE. Others send `ACK` and stay on patrol.
6. **After commit** for an epoch, new bids for that epoch are ignored, except conflicting CLAIMs (8.5).

**Ownership vs observation:** the drone that detected the target is not automatically the owner. It is just another bidder.

### 8.4 Restarting allocation (reopen)
A reopen (transition 16) increments `epoch` and excludes the previous owner for the new epoch. Triggers:
| Trigger | Detected by |
|---|---|
| Owner heartbeat missing for `heartbeat.miss_limit` periods | any agent |
| Owner `RELEASE(battery_reserve)` | owner |
| Owner `RELEASE(route_unreachable)` after `replan_limit` failures | owner |
| Owner `RELEASE(reacquire_timeout)` when `reacquire_timeout_action=reallocate` | owner |

If two agents reopen at the same time they both compute `epoch+1`; the compare rule handles it. Reopens are capped at `max_reallocations`; then the task is CANCELLED (`max_reallocations`).

### 8.5 Conflicts, stale messages and partitions
- **Message identity:** every message has `(sender, seq)`; receivers drop duplicates.
- **Stale messages:** dropped if `now − t_sent > max_message_age` or `epoch < local epoch`.
- **Conflicting claims:** if an owner receives a `CLAIM` for the same target and epoch from another drone, both apply the compare rule. The loser sends `RELEASE(conflict_lost)`, returns to patrol, and logs `CLAIM_CONFLICT`.
- **Network partition:** each side may commit its own owner. That is allowed while partitioned. When links return, heartbeats and table re-broadcasts reach both sides; owners find the conflict and resolve it as above. If the losing side had already logged the target, the merged target log keeps one primary entry and marks the other `duplicate_inspection`.
- **Owner silent but alive (partition, not failure):** the other side may reopen and assign a new owner with a higher epoch. When the partition heals, the old owner sees the higher epoch and releases.

### 8.6 What is and is not guaranteed
- **Guaranteed (by construction, tested):** a drone with no valid route or insufficient energy never wins; a geofence violation is never commanded; a target is never logged without `confirmation=CONFIRMED` and the inspection conditions; all agents use identical deterministic tie-breaking.
- **Expected, not guaranteed:** all connected agents agree on the same owner once messages are delivered within the configured timeouts and the network stays connected long enough. Under long partitions, heavy loss or very high latency, two owners can exist temporarily and work may be duplicated. The design detects and resolves this after recovery; it does not prevent it.
- **Not claimed:** consensus under arbitrary network failure, Byzantine (lying) agents, or real-time guarantees.

### 8.7 Centralized baseline (evaluation only)
`allocation/baseline.py` computes the minimum-cost feasible drone from true states using the same cost and planner. It runs after a scenario finishes, never during. It is used to report how often the decentralized result matches the optimum.

---

## 9. Geofence and safety layer

The safety layer is an independent hard constraint. Allocation never bypasses it: a winning drone still needs a validated route, and every tick still passes the runtime monitor.

### 9.1 Geofence model (`safety/geofence.py`)
- **Operating boundary:** one polygon. Flight is allowed only strictly inside it.
- **Restricted zones:** list of polygons with IDs. In Phases 1–5 they are **full-height keep-out zones** (no overflight).
- **Altitude limits:** global `[min_alt, max_alt]`.
- **Margin:** boundary is shrunk and zones are grown by `geofence.margin` meters (shapely `buffer`). All checks use the buffered shapes.
- **Edge rule:** a point exactly on a buffered boundary or zone edge counts as **outside the allowed area** (conservative).
- Config validation: polygons must be valid and non-self-intersecting; zones must overlap the boundary; `home` positions and patrol routes must be valid; altitude limits must contain `cruise_alt`, `standoff_alt` and an altitude inside every sensor's `alt_band`. Any failure stops the run before tick 0 with a clear message.

### 9.2 Route validation (`safety/route_validator.py`)
A route is a list of 3D waypoints. It is valid only if every segment is:
- inside the buffered boundary (no edge crossing),
- not intersecting any buffered zone (shapely segment–polygon intersection, touching counts as intersecting),
- within altitude limits at both ends (linear altitude between waypoints keeps it within limits).

`ValidationResult` returns `ok`, the first failing segment index, and a reason code: `OUT_OF_BOUNDARY`, `CROSSES_ZONE:<id>`, `ALT_BELOW_MIN`, `ALT_ABOVE_MAX`, `START_INVALID`, `GOAL_INVALID`.

### 9.3 Path planning (`safety/planner.py`)
- Horizontal path: visibility graph over vertices of the buffered zones (plus start and goal), edges kept only if `validate_segment` passes, then A* by length. Deterministic ordering of nodes.
- Vertical profile: section 6.5.
- **Inspection point:** if the straight-line approach to the target ends at a valid point inside `r_eff` at `standoff_alt`, use the closest such point. Otherwise sample `inspection.candidate_count` points on the circle of radius `0.9 × r_eff` around the target, keep valid ones, and pick the one with the shortest route. If none are valid (for example the target is deep inside a restricted zone), return `None`: that drone cannot bid.
- Output routes are always re-checked by `validate_route` before use.

### 9.4 Runtime safety monitor (`safety/monitor.py`)
Each tick, before the drone moves:
| Check | Verdict on failure |
|---|---|
| Next position inside geofence and altitude limits | `HOLD` (do not move this tick), then `REPLAN` |
| Remaining route still valid (geofence may change in later versions; also catches planner bugs) | `REPLAN` |
| Predicted separation from any peer < `min_separation + separation_buffer` within `separation_horizon` s (peer positions from heartbeats, extrapolated) | Lower-priority drone yields: drone **without** a task yields to the one with a task; otherwise higher ID yields. Yield = `HOLD` until clear |
| Battery reserve (6.4) | `RETURN` |
| No valid route home and battery < critical | `LAND` |

Replanning: up to `navigation.replan_limit` attempts. If all fail, the owner releases the task (`route_unreachable`) and returns. Every verdict other than `OK` is logged as a `SAFETY_*` event and shown in the dashboard.

Separation uses only heartbeat information (decentralized), so it can be wrong under packet loss or latency. The world separately measures **true** separation from ground truth for evaluation and reports any violation; it never feeds that back to the agents.

---

## 10. Communication

### 10.1 Message types (`comms/messages.py`)
Every message carries `sender, seq, t_sent`, and where relevant `target_id, epoch`.

| Type | Payload | Sent when |
|---|---|---|
| `HEARTBEAT` | pos, alt, vel, battery_pct, mode, current_task, epoch | every `heartbeat.period` |
| `TARGET_DETECTED` | target_id, est_pos, alt, source, first observation | record created |
| `OBSERVATION` | target_id, observation (without ground-truth flags) | each valid sensor observation, rate-limited to `obs_share_hz` |
| `MERGE` | survivor_id, alias_id | duplicate records merged |
| `TARGET_STATE` | target_id, epoch, from, to, reason | owner or authorized agent transitions |
| `BID` | target_id, epoch, winner_id, winning_bid, own_bid, feasible, reason, breakdown | during bidding |
| `CLAIM` | target_id, epoch | winner commits |
| `ACK` | target_id, epoch, winner_id | non-winner commits |
| `RELEASE` | target_id, epoch, reason | owner gives up the task |
| `TARGET_LOGGED` | target_id, aliases, inspection result | target logged |

### 10.2 Simulated network (`comms/network.py`)
- `comm_range` — a message reaches only drones within range at send time.
- `latency` — uniform `[min, max]` seconds per recipient, so messages can arrive **reordered**.
- `packet_loss` — independent drop probability per recipient.
- `partitions` — scheduled groups that cannot talk to each other between `start` and `end`.
- FAILED drones neither send nor receive.
- Every send, delivery and drop is logged.

---

## 11. Visualization (`viz/`)

The dashboard reads a `Snapshot` each frame. It has no reference to agents and cannot change their state. Inputs from the keyboard and mouse are turned into `Scenario` events (section 4.1 rule 6).

### 11.1 Layout (≈ 1440 × 860)
```
┌────────────────────────── MAP (≈ 920 px) ─────────────────────────┬────────── PANEL (≈ 520 px) ─────────┐
│ operating boundary (solid), margin (dashed), restricted zones     │ DRONES                              │
│ (hatched, with IDs)                                               │ D1 PATROL      82%  60 m            │
│ drones: triangle + ID + altitude; trail; sensor footprint r_eff   │ D2 ENROUTE T-D1-001  64%  60 m      │
│   (filled faint), comm range (dashed); colour = drone mode        │ D3 PATROL  (infeasible: battery)    │
│ planned route of owner (polyline with waypoints)                  │ TARGET T-D1-001  [TRACKING]         │
│ belief target: marker shape = lifecycle state, label = ID,        │  confirmation CONFIRMED  conf 0.81  │
│   confidence ring; last-known position when LOST                  │  last obs 0.4 s ago (FRESH)         │
│ ground-truth target: faint cross (toggle G)                       │  owner D2  epoch 0  insp 6.2/10 s   │
│ message flashes: line between sender and receiver (fades)         │  agents agreeing: 4/4               │
│ safety warnings: red ring on drone + text                         │ BIDS (epoch 0)  t_go  E  batt  cost │
│ top bar: sim time, speed, PAUSED, scenario name                   │ TRANSITIONS / TASK HISTORY          │
│ legend (toggle L)                                                 │ SAFETY + ALLOCATION EVENTS          │
│                                                                   │ MESSAGE LOG (scroll, filter by type)│
└───────────────────────────────────────────────────────────────────┴─────────────────────────────────────┘
```

### 11.2 Whose view is shown
The target panel shows the **owner's** local view (or the lowest-ID agent's view when there is no owner) and an "agents agreeing" count: how many live agents hold the same `(target_id, state, epoch, owner)`. Disagreement is highlighted. This makes convergence visible instead of hiding it.

### 11.3 Visual encoding
- Drone mode and target lifecycle state each use a distinct colour **and** a distinct shape or label, so states are readable without relying on colour alone.
- Legend lists every drone mode, every target state, zone styles, sensor/comm circles, route, ground-truth marker and safety warning.

### 11.4 Controls
| Key / mouse | Action |
|---|---|
| `SPACE` | pause / resume |
| `←` / `→` | slower / faster (0.25× to 8×) |
| `R` | reset scenario |
| Left click on map | place a **ground-truth** target (drones must still detect it) |
| `Shift` + click | send an **external cue** at that point |
| `1`–`4` | select drone |
| `F` | fail selected drone |
| `H` | hide / show target (forces tracking loss) |
| `G`, `L`, `M` | toggle ground truth, legend, message-flash lines |

---

## 12. Configuration

One YAML schema covers all phases. The loader validates every section present; sections used by later phases are accepted and ignored (with an info message) until implemented. All numbers below are defaults.

```yaml
sim:
  dt: 0.1
  duration: 400
  seed: 42
  headless: false
  map_size: [2000, 2000]          # meters, for drawing

geofence:
  boundary: [[50, 50], [1950, 50], [1950, 1950], [50, 1950]]
  margin: 15
  altitude_limits: [15, 120]
  restricted_zones:
    - id: RZ-1
      polygon: [[800, 600], [1100, 600], [1100, 1000], [800, 1000]]

network:
  comm_range: 1500
  latency: [0.05, 0.2]
  packet_loss: 0.05
  max_message_age: 5.0
  partitions: []                  # e.g. [{start: 60, end: 120, groups: [[D1, D2], [D3, D4]]}]

heartbeat:
  period: 1.0
  miss_limit: 3

allocation:
  weights: {time: 1.0, energy: 0.5, battery: 0.3}
  round_period: 0.5
  consensus_rounds: 3
  allocation_timeout: 10.0
  max_reallocations: 2

confirmation:
  window: 5.0
  timeout: 15.0
  min_observations: 3
  min_mean_confidence: 0.6
  min_distinct_observers: 1
  detector_holds: true
  hold_max: 8.0

association:
  radius: 30
  ignore_after_complete: 60

observation:
  max_observation_age: 3.0
  obs_share_hz: 1.0

confidence:
  base: 0.95
  falloff_exp: 1.5
  noise_std: 0.05
  spurious_conf_range: [0.2, 0.6]
  low_threshold: 0.4
  medium_threshold: 0.7

track:
  smoothing: 0.3
  stale_after: 1.5
  lost_after: 3.0

reacquire:
  timeout: 20.0
  leg_step: 40
  timeout_action: cancel          # cancel | reallocate

inspection:
  duration: 10.0
  min_confidence: 0.6
  timer_on_loss: pause            # pause | reset
  standoff_alt: 60
  energy_margin: 1.2
  candidate_count: 16

navigation:
  cruise_alt: 60
  arrival_tolerance: 5.0
  alt_tolerance: 2.0
  replan_limit: 3

safety:
  min_separation: 25
  separation_buffer: 10
  separation_horizon: 3.0

drone_defaults:
  max_speed: 15                   # m/s
  climb_rate: 3                   # m/s
  comm_range: 1500
  capacity_wh: 100
  reserve_pct: 0.20
  critical_pct: 0.08
  k_h: 0.004
  k_c: 0.02
  k_d: 0.002
  hover_power: 0.05
  sensor:
    radius: 80
    half_fov_deg: 45              # at 60 m altitude: r_eff = min(80, 60) = 60 m
    alt_band: [20, 120]
    rate_hz: 2
    p_detect: 0.9
    position_noise_std: 3.0
    false_obs_rate: 0.0

drones:
  - id: D1
    pos: [200, 300]
    alt: 60
    battery_pct: 0.85
    home: [100, 100, 0]
    patrol: [[200, 300, 60], [600, 300, 60], [600, 500, 60], [200, 500, 60]]
  - id: D2
    pos: [1500, 400]
    alt: 60
    battery_pct: 0.70
    home: [100, 100, 0]
    patrol: [[1500, 400, 60], [1800, 400, 60], [1800, 900, 60], [1350, 900, 60]]   # passes 50 m from GT-1
  - id: D3
    pos: [900, 1600]
    alt: 60
    battery_pct: 0.30
    home: [100, 100, 0]
    patrol: [[900, 1600, 60], [1200, 1600, 60], [1200, 1800, 60], [900, 1800, 60]]
  - id: D4
    pos: [1700, 1700]
    alt: 60
    battery_pct: 0.95
    home: [100, 100, 0]
    patrol: [[1700, 1700, 60], [1850, 1700, 60], [1850, 1850, 60], [1700, 1850, 60]]

targets:                          # ground truth; agents never read this directly
  - id: GT-1
    pos: [1300, 900]
    alt: 0
    appear_at: 10
    hidden_intervals: []          # e.g. [[150, 158]] for tracking_loss.yaml

scenario_events: []
  # - {t: 20, type: external_cue, pos: [1300, 900]}
  # - {t: 90, type: fail_drone, drone: D2}
  # - {t: 30, type: set_battery, drone: D1, battery_pct: 0.15}
```

Note on `default.yaml`: D2's patrol passes 50 m from `GT-1`, inside its 60 m effective radius, so D2 detects the target during patrol. D2 is then one bidder among four; it is not automatically the owner. `external_cue.yaml` covers the case where no patrol sees the target.

---

## 13. Logging and outputs

### 13.1 Event log — `output/<run_id>/events.jsonl`
One JSON object per line: `{t, type, actor, target_id?, epoch?, data}`. Event types:

`SCENARIO_EVENT`, `MSG_SENT`, `MSG_DELIVERED`, `MSG_DROPPED`, `OBSERVATION`, `OBSERVATION_REJECTED`, `TARGET_CREATED`, `TARGET_MERGED`, `TARGET_TRANSITION`, `ILLEGAL_TRANSITION`, `CONFIRMATION_RESULT`, `BID_COMPUTED` (with full breakdown), `ALLOCATION_COMMITTED`, `ALLOCATION_FAILED`, `CLAIM_CONFLICT`, `REALLOCATION`, `ROUTE_PLANNED`, `ROUTE_REJECTED` (with reason), `SAFETY_HOLD`, `SAFETY_REPLAN`, `SAFETY_RETURN`, `SAFETY_LAND`, `SEPARATION_YIELD`, `TRACK_FRESHNESS`, `TRACK_LOST`, `REACQUIRE_START`, `REACQUIRE_SUCCESS`, `REACQUIRE_TIMEOUT`, `INSPECTION_PROGRESS`, `TARGET_LOGGED`, `TASK_COMPLETED`, `TASK_CANCELLED`, `DRONE_MODE`, `DRONE_FAILED`.

### 13.2 Target log — `output/<run_id>/target_log.csv`
Each agent writes to its own logbook. At the end of the run the logbooks are merged by target ID (aliases resolved). Columns:

`target_id, aliases, source, detected_at, detected_by, confirmed_at, allocated_at, owner, epoch, inspection_start, logged_at, inspection_duration, mean_confidence, est_x, est_y, est_alt, final_state, cancel_reason, reallocations, messages_used, duplicate_inspection`

A row reaches `final_state=COMPLETED` only through the TRACKING → LOGGED → COMPLETED path. Cancelled tasks are logged with `final_state=CANCELLED` and a reason, never as inspected.

### 13.3 Run summary — `output/<run_id>/summary.json`
Times (detect, confirm, allocate, first track, log), messages used, owner vs baseline owner and match flag, reallocation count, track-loss count, reacquisition outcome, geofence violations (must be 0), true-separation violations, false-confirmation flag (a record built only from spurious observations reached CONFIRMED), and position error of the logged estimate vs ground truth.

---

## 14. Phases, deliverables and review points

Each phase ends with a **stop for review**. Do not start the next phase until it is approved.

### Phase 1 — Core simulation foundation
**Deliverables:** project skeleton and `requirements.txt`; `config.py` with full schema validation (later sections accepted but unused); `sim/world.py` tick loop; `drone/state.py`, `battery.py`, `navigation.py` (waypoint follower), drone FSM with `PATROL`, `RETURNING`, `FAILED` only; pygame map, top bar, drone panel, controls (pause, speed, reset, select, fail); `event_log.py`; README with install and run steps.
**Acceptance:** `python -m src.main --config config/default.yaml` shows 4 drones patrolling; battery decreases; climbs cost more than level flight; failing a drone freezes it; same seed gives identical event logs.
**Tests:** `unit/test_battery.py`, `sim/test_phase1_patrol.py` (headless, deterministic event-log hash).
**Stop for review.**

### Phase 2 — Target lifecycle, sensor simulation, geofencing, route safety
**Deliverables:** `sim/ground_truth.py`; `drone/sensor.py`; `tracking/observation.py`, `target_state.py` (full transition table), `tracker.py` (association, merge, confirmation, freshness) running **locally per agent** (no messages yet, so no peer reports); `safety/geofence.py`, `route_validator.py`, `planner.py`, `monitor.py` (geofence + altitude + battery reserve; separation added in Phase 3 when heartbeats exist); patrol routes validated at startup; `CONFIRM_HOLD` mode; map shows boundary, margins, zones, sensor footprints, belief targets with state markers, legend.
**Acceptance:** a drone that flies over `GT-1` creates a record, reaches CONFIRMED with the default criteria, and nothing further happens (allocation not built yet); spurious observations never confirm in `false_observation.yaml`; planner routes around RZ-1 in `geofence_detour.yaml`; no commanded position is ever outside the geofence (asserted every tick in tests).
**Tests:** `unit/test_sensor.py`, `test_observation.py`, `test_target_state.py`, `test_tracker.py` (single-agent association/merge), `test_geofence.py`, `test_route_validator.py`, `test_planner.py`; `sim/test_phase2_detection_safety.py`.
**Stop for review.**

### Phase 3 — Communication, decentralized auction, consensus, task ownership
**Deliverables:** `comms/messages.py`, `network.py` (range, latency, reordering, loss); heartbeats and peer table; separation check in the monitor; `TARGET_DETECTED`, `OBSERVATION`, `MERGE` sharing; `allocation/cost.py`, `auction.py`, `baseline.py`; lifecycle reaches ALLOCATED on every live agent; `CLAIM`/`ACK`; external-cue allocation from DETECTED; dashboard bid table with breakdowns, message log, "agents agreeing" count.
**Stop condition for this phase:** the owner is chosen and announced; it does **not** yet fly to the target (ENROUTE is Phase 4).
**Acceptance:** in `default.yaml` every live agent commits the same owner and it matches the baseline; in `low_battery.yaml` the nearest drone reports `feasible=false` and another wins; in `limited_comms.yaml` agreement is reached through relay; in `duplicate_reports.yaml` all agents converge on one target ID; in `external_cue.yaml` the cue is allocated while still `confirmation=PENDING`.
**Tests:** `unit/test_cost.py`, `test_consensus.py`, `test_network.py`; `sim/test_phase3_allocation.py`; `sim/test_decentralization.py`.
**Stop for review.**

### Phase 4 — Navigation, tracking, inspection, logging and recovery
**Deliverables:** owner ENROUTE with validated route and runtime monitor; inspection point selection; INSPECTING with inspection timer; track freshness, TRACKING_LOST, REACQUIRING with spiral search; LOGGED → COMPLETED; `target_log.py` with per-agent logbooks and merge; RELEASE on battery reserve or unreachable route and reopen with epoch increment; RETURNING to patrol; cue verification and `cue_not_confirmed` cancellation; dashboard route, inspection progress, transition history.
**Acceptance:** in `default.yaml` the owner flies a valid route, inspects for `inspection.duration`, logs, returns to patrol, and `target_log.csv` has one COMPLETED row; in `tracking_loss.yaml` the track is lost and reacquired, and the inspection timer pauses; in `target_in_restricted.yaml` the target is inspected from outside the zone or cancelled with `no_feasible_bidders`; a cue at a location with no ground-truth target ends CANCELLED and is never logged.
**Tests:** `sim/test_phase4_inspection.py` plus new unit cases for inspection guards and reacquisition.
**Stop for review.**

### Phase 5 — Failure scenarios, robustness, evaluation, report plots
**Deliverables:** `drone_failure.yaml` (owner fails mid-task → heartbeat timeout → reallocation), `partition.yaml` (split and heal, duplicate-owner detection and resolution); `src/evaluate.py` running each scenario over `N` seeds headless; metrics and plots in `output/eval/`.
**Metrics:** time to detect / confirm / allocate / first track / log; messages per allocation; agreement with baseline (%); reallocation time after owner failure; track-loss and reacquisition success rate; false-confirmation rate; geofence violations (expected 0); true-separation violations; duplicate inspections under partition.
**Acceptance:** `python -m src.evaluate` produces a metrics table and plots; every scenario's assertions pass across all seeds; any duplicate inspection is detected and marked.
**Tests:** `sim/test_phase5_robustness.py`.
**Stop for review.**

---

## 15. Testing plan

Unit tests (`tests/unit/`) use no world and no pygame and run in seconds. Simulation tests (`tests/sim/`) run full headless scenarios. All randomness comes from `sim.seed` (one `numpy.random.Generator` per component, seeded from it), so every test is reproducible.

| Area | Tests | Phase |
|---|---|---|
| Target-state transitions | every legal row of 7.8 succeeds; a sample of illegal ones raise; terminal states are sticky; adoption rules for remote state | 2 (remote adoption: 3) |
| Stable identity / duplicates | association radius; merge keeps smallest ID; alias redirect; no merge into terminal records; multi-agent convergence on one ID | 2 / 3 |
| Detection checks | inside/outside `r_eff`; altitude band edges; hidden interval produces no observations; `p_detect` statistics | 2 |
| Observation validation | future/old timestamp, out-of-boundary, bad confidence, duplicate `obs_id` | 2 |
| Confirmation | criteria met; window expiry → REJECTED; spurious-only never confirms | 2 |
| Freshness and loss | FRESH → STALE → LOST at configured times | 2 / 4 |
| Reacquisition | recovery returns to TRACKING; timeout → cancel or reallocate per config | 4 |
| Cost and feasibility | closer drone better bid; larger `r_eff` shortens route; insufficient energy → infeasible; no valid route → infeasible; deterministic tie-break | 3 |
| Consensus | random bids with random latency, reordering and 20% loss → all agents agree; line topology (relay); stale-epoch messages ignored | 3 |
| Partitions | split → two owners possible; heal → one owner, loser releases, duplicate marked | 5 |
| Geofence | point inside/outside; exactly on edge = outside; margin applied; concave boundary; altitude limits | 2 |
| Route validation | segment crossing a zone corner; segment touching an edge; segment leaving and re-entering a concave boundary; altitude out of range | 2 |
| Planner | detour around zone; no route → `None`; target inside zone → edge inspection point or `None` | 2 |
| Runtime safety | every-tick assertion: no commanded position outside geofence in any scenario; HOLD on invalid next step; replan limit → release | 2 / 4 |
| Battery reserve | owner releases and returns before going below reserve; never plans below reserve | 2 / 4 |
| Separation | conflicting paths → yield rule applied; true separation measured | 3 |
| Failure and reallocation | owner fails → reopen with epoch+1 → new owner completes | 5 |
| Correct logging | no COMPLETED row without CONFIRMED + inspection conditions; cue at empty location never logged; cancelled tasks logged as cancelled | 4 |
| End-to-end | every scenario YAML headless with expected final states | each phase |
| Decentralization | agents receive a restricted context; a test asserts `Drone` holds no references to other `Drone`, `World` or `GroundTruth` objects (introspection of attributes) and that cross-drone data arrives only through the inbox | 3 |

---

## 16. Instructions for Claude Code

1. Build **one phase at a time**. At the end of each phase: run all tests, run the phase's scenarios, write a short phase report in `README.md`, then **stop and wait for approval**.
2. Keep every tunable value in YAML. No magic numbers in logic.
3. Use dataclasses and type hints. Keep modules small and single-purpose, matching section 3.
4. A drone may never read another drone's object, `World` or `GroundTruth`. Cross-drone information only goes through `CommNetwork`.
5. Never let allocation bypass the safety layer. Every movement goes through a validated route and the runtime monitor.
6. Never log a target as inspected unless the 7.10 conditions hold.
7. Seed all randomness from `sim.seed`.
8. Everything is simulation. Do not add code that connects to or commands real vehicles.

---

## 17. Future work (after Phase 5, not part of this plan)
- Multiple simultaneous targets (full CBBA with task bundles).
- Moving targets with a proper estimator (α-β or Kalman filter) in `tracker.py`.
- Altitude-bounded restricted zones (overflight allowed above a ceiling); geofence updates at runtime.
- Heterogeneous drones (different speeds, sensors, endurance).
- Plug a computer-vision detector into `sensor.py` through the same `Observation` interface.
- Port agent logic to ROS 2 nodes with Gazebo or AirSim; `tracking/`, `allocation/`, `safety/` and `comms/messages.py` are designed to move unchanged.
- Optional plain-language explanation of each drone's decision, generated from its bid breakdown, for presentation only.

---

## 18. Revision notes

### 18.1 Contradictions and problems fixed from revision 1
1. **Phase 1 scope conflict.** Revision 1 said Phase 1 included allocation, but its phase list said Phase 1 had none. Section 1 now describes the whole plan's scope, and section 14 uses your five phases.
2. **Coordinates treated as detection.** Revision 1's "injected" mode effectively let coordinates start allocation as if the target were seen. Now coordinates create an unconfirmed record and logging requires sensor confirmation.
3. **`AUCTION` as a drone state.** Revision 1 stopped drones in an AUCTION state. Now bidding runs in the background while drones patrol, which avoids freezing the whole team during every auction.
4. **Straight-line energy estimates.** Revision 1 used straight-line distance, which is wrong once restricted zones exist. Now cost uses planned route length.
5. **`busy` cost weight.** Revision 1 penalised busy drones with a weight. A drone that already owns a task is now simply ineligible, which is clearer and cannot be outweighed.
6. **Separation using other drones' positions.** Revision 1 implied direct knowledge. Now separation uses heartbeat data only, and true separation is measured by the world for evaluation.
7. **"Split brain" fix overstated.** Revision 1 implied conflicts were always resolved. Section 8.6 now states what is guaranteed and what is not.

### 18.2 Significant design decisions
1. **Lifecycle state plus a confirmation attribute.** External cues cannot be confirmed before someone flies there, so a cue is allocated from DETECTED as a verify-and-inspect task, and LOGGED requires `confirmation=CONFIRMED`. Without this split, the cue path would contradict "confirm before allocate".
2. **TRACKING_LOST and REACQUIRING are separate.** TRACKING_LOST is the observed fact (track went stale); REACQUIRING is the owner actively searching. If the owner cannot search (battery, route), the task goes from TRACKING_LOST straight to reopen.
3. **Drone FSM and target lifecycle FSM are separate machines.** One describes what each drone is doing; the other describes the task. The tables in 6.2 and 7.8 are kept consistent.
4. **Authority per transition.** After allocation only the owner moves the task forward; any agent can reopen on owner timeout. This removes ambiguity about who decides in a leaderless system.
5. **Epochs** number each allocation round so stale bids and claims from earlier rounds are ignored after a reallocation.
6. **Duplicate target IDs** are resolved by keeping the smallest ID, a rule every agent applies identically without coordination.
7. **Geofence is a hard filter, never a cost term**, with a conservative edge rule (on the edge = outside) and a safety margin.
8. **Restricted zones are full-height** in Phases 1–5. Altitude-bounded zones are future work, which keeps route validation simple and testable.
9. **Visibility-graph A\*** for planning (exact around polygons, deterministic) instead of a grid planner.
10. **`shapely` added** for robust geometry.
11. **Per-agent logbooks merged at the end**, so logging stays decentralized and duplicate inspections after a partition are detectable.
12. **Detecting drone may hold position briefly to confirm** (`CONFIRM_HOLD`). This is observation, not ownership; it bids like everyone else.
