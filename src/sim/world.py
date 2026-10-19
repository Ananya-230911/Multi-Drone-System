"""Simulation clock and deterministic tick loop (PLAN.md section 4.2).

Phase 1 tick:
  1. scenario.apply(t)
  2. (network.deliver -- Phase 3)
  3. each drone, in sorted ID order: drone.step(t, dt, sensor_view)
  4. record: drain agent events into the event log, build the Snapshot
"""

from __future__ import annotations

import zlib

import numpy as np

from src.config import Config
from src.drone.agent import Drone
from src.logging_.event_log import EventLog
from src.sim.scenario import Scenario
from src.viz.snapshot import Snapshot

RECENT_EVENTS = 200  # events kept in the snapshot for the dashboard


def make_rng(seed: int, component: str) -> np.random.Generator:
    """One independent generator per component, derived only from `sim.seed` and the component name."""
    return np.random.default_rng(np.random.SeedSequence([seed, zlib.crc32(component.encode("utf-8"))]))


class World:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.dt = cfg.sim.dt
        self.tick = 0
        self.t = 0.0
        self.n_ticks = int(round(cfg.sim.duration / cfg.sim.dt))
        self.log = EventLog()
        self.scenario = Scenario(cfg.scenario_events)
        ids = cfg.drone_ids
        self.drones: dict[str, Drone] = {
            d.id: Drone(d, cfg.navigation, ids) for d in sorted(cfg.drones, key=lambda d: d.id)
        }
        self.snapshot = self._build_snapshot()

    @property
    def finished(self) -> bool:
        return self.tick >= self.n_ticks

    def step(self) -> Snapshot:
        if self.finished:
            return self.snapshot
        self.tick += 1
        # Integer tick counter avoids accumulating floating-point error in the clock.
        self.t = round(self.tick * self.dt, 9)
        self.scenario.apply(self.t, self.drones, self.log)
        for drone_id in sorted(self.drones):
            self.drones[drone_id].step(self.t, self.dt, self._sensor_view)
        self._record()
        return self.snapshot

    def run(self) -> Snapshot:
        while not self.finished:
            self.step()
        return self.snapshot

    def _sensor_view(self, *args, **kwargs) -> list:
        """Sensor model hook. No ground truth exists in Phase 1, so it never returns observations."""
        return []

    def _record(self) -> None:
        for drone_id in sorted(self.drones):
            agent = self.drones[drone_id]
            self.log.extend(agent.events)
            agent.events.clear()
        self.snapshot = self._build_snapshot()

    def _build_snapshot(self) -> Snapshot:
        return Snapshot(
            t=self.t,
            tick=self.tick,
            duration=self.cfg.sim.duration,
            finished=self.finished,
            scenario=self.cfg.name,
            seed=self.cfg.sim.seed,
            map_size=self.cfg.sim.map_size,
            boundary=tuple(self.cfg.geofence.boundary),
            drones=tuple(self.drones[i].export_view() for i in sorted(self.drones)),
            recent_events=tuple(self.log.tail(RECENT_EVENTS)),
        )
