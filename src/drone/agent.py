"""Drone agent (PLAN.md section 6).

One instance per drone, identical code for all. A drone holds only its own
state, its own parameters and static shared configuration (map, geofence,
drone IDs). It never holds a reference to another `Drone`, to `World` or to
ground truth (section 4.1). Its sensor output arrives through the
`sensor_view` callable the world passes into `step`, as observations only.

Events the drone produces are queued in `self.events`; the world drains them
into the event log after each step.

Tick (section 4.2):
  1. read inbox -> peer table, tracker, auction tables  (coordination.py)
  2. sense -> validate -> tracker; share observations
  3. tracker updates confirmation / lifecycle / freshness (local view)
  4. allocation step: bid / compare / commit             (coordination.py)
  5. drone FSM: battery checks, CONFIRM_HOLD entry/exit, SAFETY_HOLD replanning
  6-7. routes come from the planner (validated); the monitor checks every step,
       including predicted separation from peers (heartbeats only)
  8. navigation moves the drone; battery drains
  9. heartbeat; outbound messages wait in `self.outbox` for the world to send

Modes so far: PATROL, CONFIRM_HOLD, RETURNING, SAFETY_HOLD, LANDED, FAILED.
Phase 3 stops at ownership: the owner is chosen and announced but does not fly
to the target yet (ENROUTE is Phase 4).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Literal

from src.config import (
    AllocationConfig,
    AssociationConfig,
    ConfidenceConfig,
    ConfirmationConfig,
    DroneConfig,
    HeartbeatConfig,
    InspectionConfig,
    NavigationConfig,
    ObservationConfig,
    Point3,
    SafetyConfig,
    TrackConfig,
)
from src.drone.battery import Battery, EnergyModel, route_cost
from src.drone.coordination import CoordinationMixin
from src.drone.fsm import DroneMode, check_transition
from src.drone.navigation import Navigator, StepResult
from src.drone.sensor import confidence_category, effective_radius
from src.drone.state import DroneState
from src.safety import monitor
from src.safety.geofence import Geofence
from src.safety.planner import Planner, inspection_point
from src.safety.route_validator import route_length
from src.tracking.observation import AgentObservation, validate
from src.tracking.target_state import Confirmation, TargetState
from src.tracking.tracker import Tracker

# sensor_view(x, y, alt, now) -> observations of the world from this pose (no ground truth objects)
SensorView = Callable[[float, float, float, float], list[AgentObservation]] | None
Then = Literal["patrol", "hover", "home"]


@dataclass(frozen=True)
class SharedContext:
    """Static configuration every agent may read (section 4.1 rule 1)."""

    drone_ids: tuple[str, ...]
    fence: Geofence
    navigation: NavigationConfig
    confirmation: ConfirmationConfig
    association: AssociationConfig
    observation: ObservationConfig
    confidence: ConfidenceConfig
    track: TrackConfig
    inspection: InspectionConfig
    allocation: AllocationConfig
    heartbeat: HeartbeatConfig
    safety: SafetyConfig
    max_message_age: float


@dataclass(frozen=True)
class TargetView:
    id: str
    aliases: tuple[str, ...]
    state: str
    confirmation: str
    est_pos: tuple[float, float]
    last_confidence: float
    category: str
    freshness: str
    n_obs: int
    obs_ids: tuple[str, ...]
    created_at: float
    confirmed_at: float | None
    epoch: int
    owner: str | None


@dataclass(frozen=True)
class DroneView:
    """Read-only copy of a drone's state for the snapshot. Not given to other agents."""

    id: str
    x: float
    y: float
    alt: float
    heading: float
    speed: float
    vz: float
    mode: str
    battery_pct: float
    battery_wh: float
    reserve_pct: float
    home: Point3
    docked: bool
    current_task: str | None
    route: tuple[Point3, ...]
    patrol_route: tuple[Point3, ...]
    r_eff: float = 0.0
    targets: tuple[TargetView, ...] = ()
    hold_target: str | None = None
    safety_reason: str | None = None
    alive_peers: tuple[str, ...] = ()
    auctions: tuple["AuctionView", ...] = ()
    yielding_to: str | None = None


@dataclass(frozen=True)
class BidView:
    drone: str
    feasible: bool
    value: float | None
    reason: str
    t_go: float | None
    e_need: float | None
    batt_after: float | None
    cost: float | None


@dataclass(frozen=True)
class AuctionView:
    target_id: str
    epoch: int
    status: str
    winner: str | None
    winning_bid: float | None
    bids: tuple[BidView, ...]
    heard: tuple[str, ...]


class Drone(CoordinationMixin):
    def __init__(self, cfg: DroneConfig, shared: SharedContext):
        p = cfg.params
        self.id = cfg.id
        self.shared = shared
        self.fence = shared.fence
        self.planner = Planner(shared.fence, shared.navigation.cruise_alt, shared.navigation.planner_clearance)
        self.state = DroneState(
            id=cfg.id, x=cfg.pos[0], y=cfg.pos[1], alt=cfg.alt, home=cfg.home, patrol_route=list(cfg.patrol), params=p
        )
        self.battery = Battery.from_params(p, cfg.battery_pct)
        self.energy = EnergyModel.from_params(p)
        self.tracker = Tracker(cfg.id, shared.association, shared.confirmation, shared.track)
        self._known_observers = frozenset(shared.drone_ids) | {"EXT"}
        # Movement plan: a patrol loop (validated at startup) or a planned transit route.
        self.navigator = Navigator(self.state.patrol_route, loop=True, start_index=0)
        self.on_patrol_loop = True
        self._then: Then = "patrol"
        self._then_index = 0
        self.docked = False  # parked at home on the ground; no energy use
        # CONFIRM_HOLD bookkeeping
        self.hold_target: str | None = None
        self._held: set[str] = set()
        self._resume_index = 0
        # SAFETY_HOLD bookkeeping
        self._safety_prev: DroneMode | None = None
        self._safety_goal: tuple[Point3, Then, int] | None = None
        self._replan_attempts = 0
        self.safety_reason: str | None = None
        self.events: list[dict[str, Any]] = []
        self._init_coordination()  # inbox, outbox, peer table, auction

    # ------------------------------------------------------------------ tick

    def step(self, now: float, dt: float, sensor_view: SensorView = None) -> None:
        if self.state.mode is DroneMode.FAILED:
            return  # sends nothing, does nothing
        self._read_inbox(now)
        observed = self._sense(now, sensor_view)
        self._on_tracker_events(now, self.tracker.update(now))
        self._allocation_step(now)
        if self.state.mode is not DroneMode.LANDED:
            self._decide(now, dt, observed)
        if self.state.mode is DroneMode.LANDED:
            self._descend_to_ground(dt)
        else:
            self._move(now, dt)
        self._heartbeat(now)

    # ------------------------------------------------------------------ 2-3. sense and track

    def _sense(self, now: float, sensor_view: SensorView) -> list[str]:
        """Feed own observations to the tracker; return IDs of records observed this tick."""
        if sensor_view is None:
            return []
        s, observed = self.state, []
        for obs in sensor_view(s.x, s.y, s.alt, now):
            ok, reason = validate(obs, now, self.fence, self.shared.observation, self._known_observers, self.tracker.seen_obs)
            if not ok:
                self._emit(now, "OBSERVATION_REJECTED", {"obs_id": obs.obs_id, "reason": reason, "pos": list(obs.pos)})
                continue
            upd = self.tracker.ingest(obs, now)
            if upd.kind == "ignored":
                continue
            if upd.transition is not None:
                self._emit(now, "TARGET_CREATED", {"source": obs.source, "pos": list(obs.pos)}, target_id=upd.target_id, epoch=0)
                self._emit(now, "TARGET_TRANSITION",
                           {"from": "UNKNOWN", "to": "DETECTED", "row": 1, "reason": upd.transition.reason, "by": self.id},
                           target_id=upd.target_id, epoch=0)  # fmt: skip
            self._handle_merges(now, upd.merges)
            tid = self.tracker.resolve(upd.target_id)
            if upd.transition is not None:
                self._announce_target(now, tid)
            self._emit(now, "OBSERVATION", {
                "obs_id": obs.obs_id, "pos": list(obs.pos), "confidence": obs.confidence,
                "category": confidence_category(obs.confidence, self.shared.confidence),
            }, target_id=tid)  # fmt: skip
            self._share_observation(now, tid, obs)
            observed.append(tid)
        return observed

    # ------------------------------------------------------------------ 5. decide

    def _decide(self, now: float, dt: float, observed: list[str]) -> None:
        s = self.state
        if self.docked:
            return
        e_home = self.energy_to_home()
        verdict = monitor.check_energy(
            self.battery, e_home, self._lookahead_wh(dt), check_reserve=s.mode in (DroneMode.PATROL, DroneMode.CONFIRM_HOLD)
        )
        if verdict.kind == "LAND":
            self._emit(now, "SAFETY_LAND", {"reason": verdict.reason, "battery_wh": self.battery.charge_wh})
            self._set_mode(now, DroneMode.LANDED, verdict.reason)
            return
        if verdict.kind == "RETURN":
            self._emit(now, "SAFETY_RETURN", {
                "reason": verdict.reason, "battery_wh": self.battery.charge_wh,
                "e_home_wh": e_home, "reserve_wh": self.battery.reserve_wh,
            })  # fmt: skip
            self.hold_target = None
            self._set_mode(now, DroneMode.RETURNING, verdict.reason)
            self._go_home(now)
            return
        if s.mode is DroneMode.SAFETY_HOLD:
            self._try_replan(now)
        elif s.mode is DroneMode.CONFIRM_HOLD:
            self._check_hold_exit(now)
        elif s.mode is DroneMode.PATROL and self.shared.confirmation.detector_holds:
            self._maybe_start_hold(now, observed)

    # CONFIRM_HOLD ----------------------------------------------------------

    def _maybe_start_hold(self, now: float, observed: list[str]) -> None:
        for tid in dict.fromkeys(observed):
            rec = self.tracker.record(tid)
            if rec is None or rec.id in self._held:
                continue
            if rec.state is not TargetState.DETECTED or rec.confirmation is not Confirmation.PENDING:
                continue
            self._held.add(rec.id)
            alt = self._hold_altitude()
            r_eff = effective_radius(self.state.params.sensor, alt)
            # Above the estimate if allowed, else the closest valid point still inside the sensor radius.
            hold = inspection_point(rec.est_pos, r_eff, self.planner, alt, self.shared.inspection.candidate_count,
                                    from_pos=self.state.pos)  # fmt: skip
            if hold is None:
                self._emit(now, "ROUTE_REJECTED", {"kind": "confirm_hold", "goal": list(rec.est_pos), "reason": "NO_VALID_HOLD_POINT"})
                continue
            route = self._plan(now, hold, "confirm_hold")
            if route is None:
                continue
            self._resume_index = self.navigator.index if self.on_patrol_loop else self._then_index
            self.hold_target = rec.id
            self._set_mode(now, DroneMode.CONFIRM_HOLD, f"detected:{rec.id}")
            self._set_route(route, "hover")
            return

    def _hold_altitude(self) -> float:
        """Current altitude, clamped into the geofence limits and the sensor's working band."""
        lo, hi = self.state.params.sensor.alt_band
        return min(max(self.state.alt, self.fence.min_alt, lo), self.fence.max_alt, hi)

    def _check_hold_exit(self, now: float) -> None:
        rec = self.tracker.record(self.hold_target) if self.hold_target else None
        reason = None
        if rec is None or rec.terminal:
            reason = "record_cancelled"
        elif rec.confirmation is Confirmation.CONFIRMED:
            reason = "record_confirmed"
        elif rec.confirmation is Confirmation.REJECTED:
            reason = "record_rejected"
        elif now - self.state.mode_since >= self.shared.confirmation.hold_max:
            reason = "hold_max"
        if reason:
            self.hold_target = None
            self._set_mode(now, DroneMode.PATROL, reason)
            self._go_patrol(now, self._resume_index)

    # SAFETY_HOLD -----------------------------------------------------------

    def _enter_safety_hold(self, now: float, verdict: monitor.SafetyVerdict) -> None:
        s = self.state
        goal = self.navigator.waypoints[-1] if not self.on_patrol_loop else self.navigator.target
        then, idx = (self._then, self._then_index) if not self.on_patrol_loop else ("patrol", self.navigator.index)
        self._safety_prev = s.mode
        self._safety_goal = (goal, then, idx)
        self._replan_attempts = 0
        self.safety_reason = verdict.reason
        etype = "SAFETY_HOLD" if verdict.kind == "HOLD" else "SAFETY_REPLAN"
        self._emit(now, etype, {"reason": verdict.reason, "pos": [s.x, s.y], "alt": s.alt, "goal": list(goal)})
        self._set_mode(now, DroneMode.SAFETY_HOLD, verdict.reason)

    def _try_replan(self, now: float) -> None:
        assert self._safety_goal is not None and self._safety_prev is not None
        goal, then, idx = self._safety_goal
        self._replan_attempts += 1
        route = self._plan(now, goal, "replan")
        if route is not None:
            self._emit(now, "SAFETY_REPLAN", {"result": "ok", "attempt": self._replan_attempts})
            self.safety_reason = None
            self._set_mode(now, self._safety_prev, "replan_succeeded")
            self._then_index = idx
            self._set_route(route, then)
            return
        if self._replan_attempts >= self.shared.navigation.replan_limit:
            home = self._plan(now, self.state.home, "home")
            if home is not None:
                self.safety_reason = None
                self.hold_target = None
                self._set_mode(now, DroneMode.RETURNING, "replan_limit")
                self._set_route(home, "home")
            # else: keep hovering; check_energy lands the drone once the battery is critical

    # ------------------------------------------------------------------ routes

    def _plan(self, now: float, goal: Point3, kind: str) -> list[Point3] | None:
        res = self.planner.plan(self.state.pos3, goal)
        if res.route is None:
            self._emit(now, "ROUTE_REJECTED", {"kind": kind, "goal": list(goal), "reason": res.reason})
            return None
        self._emit(now, "ROUTE_PLANNED", {"kind": kind, "waypoints": [list(w) for w in res.route],
                                          "length": route_length(res.route)})  # fmt: skip
        return res.route

    def _set_route(self, route: list[Point3], then: Then) -> None:
        self.navigator = Navigator(route[1:] if len(route) > 1 else route, loop=False)
        self.on_patrol_loop = False
        self._then = then

    def _go_patrol(self, now: float, index: int) -> None:
        wp = self.state.patrol_route[index]
        route = self._plan(now, wp, "to_patrol")
        if route is None:
            self._enter_safety_hold(now, monitor.SafetyVerdict("REPLAN", "NO_ROUTE_TO_PATROL"))
            self._safety_goal = (wp, "patrol", index)
            return
        self._then_index = index
        self._set_route(route, "patrol")

    def _go_home(self, now: float) -> None:
        route = self._plan(now, self.state.home, "home")
        if route is None:
            self._enter_safety_hold(now, monitor.SafetyVerdict("REPLAN", "NO_ROUTE_HOME"))
            self._safety_prev = DroneMode.RETURNING
            self._safety_goal = (self.state.home, "home", 0)
            return
        self._set_route(route, "home")

    def energy_to_home(self) -> float | None:
        """Energy of the planned route home (None if there is no valid route)."""
        s = self.state
        if math.dist(s.pos3, s.home) <= 1e-9:
            return 0.0
        return self._planned_energy(s.pos3, s.home)

    def _planned_energy(self, start: Point3, goal: Point3) -> float | None:
        """Energy along the planned route (an estimate only: this route is not flown)."""
        res = self.planner.plan(start, goal, revalidate=False)
        return None if res.route is None else route_cost(res.route[0], res.route[1:], self.energy).energy_wh

    def _lookahead_wh(self, dt: float) -> float:
        """Upper bound on how much one more tick can worsen `battery - E_home`.

        One tick of flight spends at most k_h*v*dt + k_c*c*dt + k_d*c*dt (+ hover) and can
        lengthen the way home by as much again, so checking against this margin means the
        reserve check trips before the reserve is crossed, not one tick after.
        """
        p = self.state.params
        h, v = p.max_speed * dt, p.climb_rate * dt
        return self.energy.move_energy(2 * h, v, v) + self.energy.hover_energy(dt)

    # ------------------------------------------------------------------ 7-8. monitor and move

    def _move(self, now: float, dt: float) -> None:
        s = self.state
        if self.docked:
            s.vx = s.vy = s.vz = 0.0
            return
        if s.mode is DroneMode.SAFETY_HOLD:
            self._hover(dt)
            return
        saved = (self.navigator.index, self.navigator.finished)
        res = self.navigator.step(s.x, s.y, s.alt, dt, s.params.max_speed, s.params.climb_rate)
        verdict = monitor.check_step(s.pos3, (res.x, res.y, res.alt), self.fence)
        if not verdict.ok:
            self.navigator.index, self.navigator.finished = saved  # do not move this tick
            self._hover(dt)
            self._enter_safety_hold(now, verdict)
            return
        vel = ((res.x - s.x) / dt, (res.y - s.y) / dt, (res.alt - s.alt) / dt)
        sep = monitor.check_separation(self.id, s.pos3, vel, s.current_task is not None, self.peer_tracks(now), self.shared.safety)
        if not sep.ok:
            # Yield = HOLD until clear. Not a SAFETY_HOLD: nothing is wrong with the route.
            self.navigator.index, self.navigator.finished = saved
            self._hover(dt)
            other = sep.reason.split(":", 1)[1]
            if self._yielding_to != other:
                self._yielding_to = other
                self._emit(now, "SEPARATION_YIELD", {"to": other, "pos": [s.x, s.y], "alt": s.alt})
            return
        self._yielding_to = None
        self._apply_motion(res, dt)
        self._on_progress(now, res)

    def _hover(self, dt: float) -> None:
        s = self.state
        s.vx = s.vy = s.vz = 0.0
        if s.alt > 0.0:
            self.battery.drain(self.energy.hover_energy(dt))

    def _descend_to_ground(self, dt: float) -> None:
        """Simulated emergency landing: straight down at climb_rate. Exempt from the altitude floor."""
        s = self.state
        s.vx = s.vy = 0.0
        step = min(s.alt, s.params.climb_rate * dt)
        s.vz = -step / dt
        s.alt -= step
        self.battery.drain(self.energy.move_energy(0.0, 0.0, step))

    def _apply_motion(self, res: StepResult, dt: float) -> None:
        s = self.state
        dx, dy, dz = res.x - s.x, res.y - s.y, res.alt - s.alt
        if math.hypot(dx, dy) > 1e-9:
            s.heading = math.atan2(dy, dx)
        s.vx, s.vy, s.vz = dx / dt, dy / dt, dz / dt
        s.x, s.y, s.alt = res.x, res.y, res.alt
        energy = self.energy.move_energy(res.horizontal, res.climb, res.descent)
        if s.alt > 0.0:
            energy += self.energy.hover_energy(res.idle_time)
        self.battery.drain(energy)

    def _on_progress(self, now: float, res: StepResult) -> None:
        s = self.state
        if self.on_patrol_loop:
            for idx in res.reached:
                s.patrol_index = (idx + 1) % len(s.patrol_route)
                self._emit(now, "WAYPOINT_REACHED", {"index": idx, "pos": [s.x, s.y], "alt": s.alt})
            return
        if not res.finished:
            return
        if self._then == "patrol":
            idx = self._then_index
            self._emit(now, "WAYPOINT_REACHED", {"index": idx, "pos": [s.x, s.y], "alt": s.alt})
            s.patrol_index = (idx + 1) % len(s.patrol_route)
            self.navigator = Navigator(s.patrol_route, loop=True, start_index=s.patrol_index)
            self.on_patrol_loop = True
        elif self._then == "home" and s.mode is DroneMode.RETURNING:
            self._arrived_home(now)
        # "hover": stay at the end of the route (CONFIRM_HOLD); idle time is charged as hover

    def _arrived_home(self, now: float) -> None:
        can_resume = self._resume_patrol_feasible()
        self._emit(now, "HOME_REACHED",
                   {"battery_pct": self.battery.pct, "resume_patrol": can_resume, "battery_wh": self.battery.charge_wh})
        if can_resume:
            self._resume_from_home(now, "battery_sufficient")
        else:
            self.docked = True

    def _resume_patrol_feasible(self) -> bool:
        """Enough energy to reach the nearest patrol waypoint and come back home above reserve."""
        wp = self.state.patrol_route[self._nearest_patrol_index()]
        e_go = self._planned_energy(self.state.pos3, wp)
        e_back = self._planned_energy(wp, self.state.home)
        return e_go is not None and e_back is not None and not self.battery.reserve_violated(e_go + e_back)

    def _resume_from_home(self, now: float, reason: str) -> None:
        self.docked = False
        self._set_mode(now, DroneMode.PATROL, reason)
        self._go_patrol(now, self._nearest_patrol_index())

    def _nearest_patrol_index(self) -> int:
        s = self.state
        dists = [math.hypot(x - s.x, y - s.y) for x, y, _ in s.patrol_route]
        return min(range(len(dists)), key=lambda i: (dists[i], i))

    # ------------------------------------------------------------------ injections (scenario only)

    def inject_failure(self, now: float, reason: str = "scenario") -> None:
        s = self.state
        if s.mode is DroneMode.FAILED:
            return
        self._set_mode(now, DroneMode.FAILED, reason)
        s.vx = s.vy = s.vz = 0.0
        self._emit(now, "DRONE_FAILED", {"pos": [s.x, s.y], "alt": s.alt, "battery_pct": self.battery.pct})

    def inject_battery(self, now: float, pct: float) -> None:
        self.battery.set_pct(pct)
        if self.docked and self._resume_patrol_feasible():
            self._resume_from_home(now, "battery_sufficient")

    # ------------------------------------------------------------------ read-only export

    def export_view(self) -> DroneView:
        s = self.state
        cc = self.shared.confidence
        targets = tuple(
            TargetView(
                id=r.id, aliases=tuple(sorted(r.aliases)), state=r.state.value, confirmation=r.confirmation.value,
                est_pos=r.est_pos, last_confidence=r.last_confidence,
                category=confidence_category(r.last_confidence, cc), freshness=r.freshness,
                n_obs=len(r.observations), obs_ids=tuple(o[3] for o in r.observations),
                created_at=r.created_at, confirmed_at=r.confirmed_at, epoch=r.epoch, owner=r.owner,
            )
            for _, r in sorted(self.tracker.records.items())
        )  # fmt: skip
        flying = s.mode not in (DroneMode.FAILED, DroneMode.LANDED, DroneMode.SAFETY_HOLD) and not self.docked
        return DroneView(
            id=s.id, x=s.x, y=s.y, alt=s.alt, heading=s.heading, speed=math.hypot(s.vx, s.vy), vz=s.vz,
            mode=s.mode.value, battery_pct=self.battery.pct, battery_wh=self.battery.charge_wh,
            reserve_pct=s.params.reserve_pct, home=s.home, docked=self.docked, current_task=s.current_task,
            route=tuple(self.navigator.remaining()) if flying and not self.on_patrol_loop else (),
            patrol_route=tuple(s.patrol_route), r_eff=effective_radius(s.params.sensor, s.alt),
            targets=targets, hold_target=self.hold_target, safety_reason=self.safety_reason,
            alive_peers=tuple(sorted(self.alive_peers(self._last_heartbeat or 0.0))),
            auctions=tuple(self._auction_view(tid) for tid in sorted(self.auction.entries)),
            yielding_to=self._yielding_to,
        )  # fmt: skip

    def _auction_view(self, tid: str) -> AuctionView:
        e = self.auction.entries[tid]
        bids = tuple(
            BidView(b.drone, b.feasible, b.value, b.reason, b.breakdown.get("t_go"), b.breakdown.get("E_need"),
                    b.breakdown.get("batt_after"), b.breakdown.get("cost"))
            for _, b in sorted(e.bids.items())
        )  # fmt: skip
        return AuctionView(tid, e.epoch, e.status, e.winner, e.winning_bid, bids, tuple(sorted(e.heard)))

    # ------------------------------------------------------------------ helpers

    def _set_mode(self, now: float, to: DroneMode, reason: str) -> None:
        s = self.state
        check_transition(s.mode, to)
        frm = s.mode
        s.mode = to
        s.mode_since = now
        self._emit(now, "DRONE_MODE", {"from": frm.value, "to": to.value, "reason": reason})

    def _emit(self, now: float, etype: str, data: dict[str, Any], target_id: str | None = None, epoch: int | None = None) -> None:
        ev: dict[str, Any] = {"t": now, "type": etype, "actor": self.id, "data": data}
        if target_id is not None:
            ev["target_id"] = target_id
        if epoch is not None:
            ev["epoch"] = epoch
        self.events.append(ev)
