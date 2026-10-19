"""Drone agent (PLAN.md section 6).

One instance per drone, identical code for all. A drone holds only its own
state, its own parameters and static shared configuration. It never holds a
reference to another `Drone`, to `World` or to ground truth (section 4.1).

Events the drone produces are queued in `self.events`; the world drains them
into the event log after each step, so the agent does not need a reference to
any shared logging object either.

Phase 1 behaviour: PATROL along the configured loop, RETURNING home when the
battery reserve check trips, FAILED on an injected failure.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable

from src.config import DroneConfig, NavigationConfig, Point3
from src.drone.battery import Battery, EnergyModel, route_cost
from src.drone.fsm import DroneMode, check_transition
from src.drone.navigation import Navigator, StepResult
from src.drone.state import DroneState

SensorView = Callable[..., list] | None


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


class Drone:
    def __init__(self, cfg: DroneConfig, nav_cfg: NavigationConfig, drone_ids: list[str]):
        p = cfg.params
        self.id = cfg.id
        self._nav_cfg = nav_cfg
        self._drone_ids = tuple(drone_ids)  # static shared config: IDs only
        self.state = DroneState(
            id=cfg.id,
            x=cfg.pos[0],
            y=cfg.pos[1],
            alt=cfg.alt,
            home=cfg.home,
            patrol_route=list(cfg.patrol),
            params=p,
        )
        self.battery = Battery.from_params(p, cfg.battery_pct)
        self.energy = EnergyModel.from_params(p)
        self.navigator = Navigator(self.state.patrol_route, loop=True, start_index=0)
        self.docked = False  # parked at home on the ground; no energy use
        self.inbox: list[Any] = []  # filled by CommNetwork from Phase 3
        self.events: list[dict[str, Any]] = []

    # ------------------------------------------------------------------ tick

    def step(self, now: float, dt: float, sensor_view: SensorView = None) -> None:
        s = self.state
        if s.mode is DroneMode.FAILED:
            return
        # 5. drone FSM decides mode and goal
        if s.mode is DroneMode.PATROL:
            self._check_battery_reserve(now, dt)
        # 8. navigation moves the drone; battery drains
        if not self.docked:
            res = self.navigator.step(s.x, s.y, s.alt, dt, s.params.max_speed, s.params.climb_rate)
            self._apply_motion(res, dt)
            self._on_waypoints(now, res)
        else:
            s.vx = s.vy = s.vz = 0.0

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
            self._resume_patrol(now, "battery_sufficient")

    # ------------------------------------------------------------------ read-only export

    def export_view(self) -> DroneView:
        s = self.state
        return DroneView(
            id=s.id,
            x=s.x,
            y=s.y,
            alt=s.alt,
            heading=s.heading,
            speed=math.hypot(s.vx, s.vy),
            vz=s.vz,
            mode=s.mode.value,
            battery_pct=self.battery.pct,
            battery_wh=self.battery.charge_wh,
            reserve_pct=s.params.reserve_pct,
            home=s.home,
            docked=self.docked,
            current_task=s.current_task,
            route=tuple(self.navigator.remaining()) if s.mode is not DroneMode.FAILED and not self.docked else (),
            patrol_route=tuple(s.patrol_route),
        )

    # ------------------------------------------------------------------ internals

    def _return_route(self) -> list[Point3]:
        """Route home: cruise altitude over home, then descend to the home altitude.

        Phase 1 has no planner, so this is the direct route. From Phase 2 it is
        replaced by a geofence-validated route from safety/planner.py.
        """
        hx, hy, hz = self.state.home
        return [(hx, hy, self._nav_cfg.cruise_alt), (hx, hy, hz)]

    def energy_to_home(self) -> float:
        return route_cost(self.state.pos3, self._return_route(), self.energy).energy_wh

    def _lookahead_wh(self, dt: float) -> float:
        """Upper bound on how much one more tick can worsen `battery - E_home`.

        One tick of flight spends at most k_h*v*dt + k_c*c*dt + k_d*c*dt (+ hover) and can
        lengthen the way home by as much again, so checking against this margin means the
        reserve check trips before the reserve is crossed, not one tick after.
        """
        p = self.state.params
        h, v = p.max_speed * dt, p.climb_rate * dt
        return self.energy.move_energy(2 * h, v, v) + self.energy.hover_energy(dt)

    def _check_battery_reserve(self, now: float, dt: float) -> None:
        e_home = self.energy_to_home()
        if self.battery.reserve_violated(e_home + self._lookahead_wh(dt)):
            self._emit(
                now,
                "SAFETY_RETURN",
                {
                    "reason": "battery_reserve",
                    "battery_wh": self.battery.charge_wh,
                    "e_home_wh": e_home,
                    "reserve_wh": self.battery.reserve_wh,
                },
            )
            self._set_mode(now, DroneMode.RETURNING, "battery_reserve")
            self.navigator = Navigator(self._return_route(), loop=False)

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

    def _on_waypoints(self, now: float, res: StepResult) -> None:
        s = self.state
        if s.mode is DroneMode.PATROL:
            for idx in res.reached:
                s.patrol_index = (idx + 1) % len(s.patrol_route)
                self._emit(now, "WAYPOINT_REACHED", {"index": idx, "pos": [s.x, s.y], "alt": s.alt})
        elif s.mode is DroneMode.RETURNING and res.finished:
            self._arrived_home(now)

    def _arrived_home(self, now: float) -> None:
        can_resume = self._resume_patrol_feasible()
        self._emit(
            now,
            "HOME_REACHED",
            {"battery_pct": self.battery.pct, "resume_patrol": can_resume, "battery_wh": self.battery.charge_wh},
        )
        if can_resume:
            self._resume_patrol(now, "battery_sufficient")
        else:
            self.docked = True

    def _resume_patrol_feasible(self) -> bool:
        """Enough energy to reach the nearest patrol waypoint and come back home above reserve."""
        wp = self.state.patrol_route[self._nearest_patrol_index()]
        e_resume = route_cost(self.state.pos3, [wp], self.energy).energy_wh
        e_back = route_cost(wp, self._return_route(), self.energy).energy_wh
        return not self.battery.reserve_violated(e_resume + e_back)

    def _resume_patrol(self, now: float, reason: str) -> None:
        idx = self._nearest_patrol_index()
        self.docked = False
        self._set_mode(now, DroneMode.PATROL, reason)
        self.navigator = Navigator(self.state.patrol_route, loop=True, start_index=idx)

    def _nearest_patrol_index(self) -> int:
        s = self.state
        dists = [math.hypot(x - s.x, y - s.y) for x, y, _ in s.patrol_route]
        return min(range(len(dists)), key=lambda i: (dists[i], i))

    def _set_mode(self, now: float, to: DroneMode, reason: str) -> None:
        s = self.state
        check_transition(s.mode, to)
        frm = s.mode
        s.mode = to
        s.mode_since = now
        self._emit(now, "DRONE_MODE", {"from": frm.value, "to": to.value, "reason": reason})

    def _emit(self, now: float, etype: str, data: dict[str, Any]) -> None:
        self.events.append({"t": now, "type": etype, "actor": self.id, "data": data})
