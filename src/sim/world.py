"""Simulation clock and deterministic tick loop (PLAN.md section 4.2).

Tick:
  1. scenario.apply(t)
  2. network.deliver(t): due messages into inboxes
  3. each drone, in sorted ID order: drone.step(t, dt, sensor_view); then its
     events go to the log and its outbox to the network
  4. record: measure true geofence and separation violations (evaluation only,
     never fed back to agents), build the Snapshot

The world owns ground truth and the per-drone sensor models. A drone only
ever receives the `sensor_view` closure for its own sensor, which returns
observations with the ground-truth `is_spurious` flag stripped.
"""

from __future__ import annotations

import math
import zlib

import numpy as np

from src.allocation.baseline import optimal
from src.allocation.cost import BidderState, CostConfig
from src.comms.network import CommNetwork
from src.config import Config, ConfigError
from src.drone.agent import Drone, SharedContext
from src.drone.sensor import Sensor
from src.logging_.event_log import EventLog
from src.logging_.target_log import LogEntry, merge as merge_logbooks
from src.safety.geofence import Geofence
from src.safety.planner import Planner
from src.safety.route_validator import validate_route
from src.sim.ground_truth import GroundTruth
from src.sim.scenario import Scenario
from src.tracking.observation import AgentObservation, Observation
from src.viz.snapshot import FenceView, GroundTruthView, Snapshot

RECENT_EVENTS = 200  # events kept in the snapshot for the dashboard
RECENT_MESSAGES = 120  # message outcomes kept in the snapshot for the dashboard


def make_rng(seed: int, component: str) -> np.random.Generator:
    """One independent generator per component, derived only from `sim.seed` and the component name."""
    return np.random.default_rng(np.random.SeedSequence([seed, zlib.crc32(component.encode("utf-8"))]))


def validate_geometry(cfg: Config, fence: Geofence) -> None:
    """Section 9.1 startup checks: homes, start positions and patrol routes must be geofence-valid."""
    for d in sorted(cfg.drones, key=lambda d: d.id):
        if fence.violation(*d.home):
            raise ConfigError(f"drone {d.id}: home {list(d.home)} is outside the geofence ({fence.violation(*d.home)})")
        start = (d.pos[0], d.pos[1], d.alt)
        if fence.violation(*start):
            raise ConfigError(f"drone {d.id}: start position {list(start)} is invalid ({fence.violation(*start)})")
        loop = [start] + list(d.patrol) + [d.patrol[0]]
        check = validate_route(loop, fence)
        if not check.ok:
            seg = f"segment {check.index}: {list(loop[check.index])} -> {list(loop[min(check.index + 1, len(loop) - 1)])}"
            raise ConfigError(f"drone {d.id}: patrol route is invalid ({check.reason}, {seg})")


class World:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.dt = cfg.sim.dt
        self.tick = 0
        self.t = 0.0
        self.n_ticks = int(round(cfg.sim.duration / cfg.sim.dt))
        self.fence = Geofence.from_config(cfg)
        validate_geometry(cfg, self.fence)
        self.log = EventLog()
        self.scenario = Scenario(cfg.scenario_events)
        self.ground_truth = GroundTruth(cfg.targets)
        ids = tuple(cfg.drone_ids)
        shared = SharedContext(
            drone_ids=ids,
            fence=self.fence,
            navigation=cfg.navigation,
            confirmation=cfg.confirmation,
            association=cfg.association,
            observation=cfg.observation,
            confidence=cfg.confidence,
            track=cfg.track,
            inspection=cfg.inspection,
            allocation=cfg.allocation,
            heartbeat=cfg.heartbeat,
            safety=cfg.safety,
            max_message_age=cfg.network.max_message_age,
            reacquire=cfg.reacquire,
        )
        ordered = sorted(cfg.drones, key=lambda d: d.id)
        self.drones: dict[str, Drone] = {d.id: Drone(d, shared) for d in ordered}
        self.sensors: dict[str, Sensor] = {
            d.id: Sensor(d.id, d.params.sensor, cfg.confidence, make_rng(cfg.sim.seed, f"sensor:{d.id}")) for d in ordered
        }
        self.network = CommNetwork(cfg.network, make_rng(cfg.sim.seed, "network"),
                                   {d.id: d.params.comm_range for d in ordered}, self.log)  # fmt: skip
        self.observations: list[Observation] = []  # every observation produced, with ground-truth flags (eval only)
        self.geofence_violations: list[tuple[float, str, str]] = []  # (t, drone, reason), measured from true state
        self.separation_violations: list[tuple[float, str, str, float]] = []  # (t, a, b, distance), true state
        self._close_pairs: set[tuple[str, str]] = set()
        # True drone states when each allocation started, for the centralized baseline (evaluation only).
        self.allocation_starts: dict[tuple[str, int], dict] = {}
        self._pending_neutralize: list[tuple[str, str, tuple[float, float]]] = []
        self._snapshot: Snapshot | None = None

    @property
    def snapshot(self) -> Snapshot:
        """Read-only view of the current tick, built on first access (headless runs rarely need it)."""
        if self._snapshot is None:
            self._snapshot = self._build_snapshot()
        return self._snapshot

    @property
    def finished(self) -> bool:
        return self.tick >= self.n_ticks

    def step(self) -> Snapshot:
        """Advance one tick and return the snapshot (built for the caller)."""
        self._advance()
        return self.snapshot

    def _advance(self) -> None:
        if self.finished:
            return
        self.tick += 1
        # Integer tick counter avoids accumulating floating-point error in the clock.
        self.t = round(self.tick * self.dt, 9)
        self.scenario.apply(self.t, self.drones, self.ground_truth, self.log)
        failed = self._failed()
        self.network.deliver(self.t, {i: d.inbox for i, d in self.drones.items()}, failed)
        for drone_id in sorted(self.drones):
            agent = self.drones[drone_id]
            agent.step(self.t, self.dt, self._sensor_view(drone_id))
            self._drain(agent)
        self._record()

    def run(self) -> Snapshot:
        while not self.finished:
            self._advance()
        return self.snapshot

    def _sensor_view(self, drone_id: str):
        sensor = self.sensors[drone_id]

        def view(x: float, y: float, alt: float, now: float) -> list[AgentObservation]:
            obs = sensor.sample(now, self.dt, x, y, alt, self.ground_truth.visible(now))
            self.observations.extend(obs)
            return [o.to_agent() for o in obs]

        return view

    def false_confirmations(self) -> list[tuple[str, str]]:
        """(drone, target_id) of CONFIRMED records built only from spurious observations (evaluation only)."""
        spurious = {o.obs_id for o in self.observations if o.is_spurious}
        out = []
        for d in self.snapshot.drones:
            for tv in d.targets:
                if tv.confirmation == "CONFIRMED" and tv.obs_ids and all(o in spurious for o in tv.obs_ids):
                    out.append((d.id, tv.id))
        return out

    def _failed(self) -> set[str]:
        return {i for i, d in self.drones.items() if d.state.mode.value == "FAILED"}

    def _drain(self, agent: Drone) -> None:
        for ev in agent.events:
            if ev["type"] == "TASK_COMPLETED" and self.cfg.mission.neutralize_on_complete:
                self._pending_neutralize.append((agent.id, ev["target_id"], tuple(ev["data"]["est_pos"])))
            if ev["type"] == "BID_COMPUTED":
                key = (ev["target_id"], ev["epoch"])
                if key not in self.allocation_starts:
                    self.allocation_starts[key] = {"t": self.t, "target_est": tuple(ev["data"]["target_est"]),
                                                   "drones": [d.export_view() for _, d in sorted(self.drones.items())]}  # fmt: skip
        self.log.extend(agent.events)
        agent.events.clear()
        while self._pending_neutralize:
            self._neutralize(*self._pending_neutralize.pop(0))
        msgs = agent.outbox.drain()
        if msgs:
            positions = {i: d.state.pos3 for i, d in self.drones.items()}
            failed = self._failed()
            for m in msgs:
                self.network.send(m, self.t, positions, failed)

    def _neutralize(self, drone_id: str, target_id: str, est: tuple[float, float]) -> None:
        """Simulated end of an engage mission: the true target the owner logged stops existing.

        A world-side effect (like the sensor model), never agent knowledge: the nearest visible
        ground-truth target within association.radius of the logged estimate is removed.
        """
        near = [(math.dist(t.pos, est), t) for t in self.ground_truth.visible(self.t)]
        near = [x for x in near if x[0] <= self.cfg.association.radius]
        if not near:
            return
        err, gt = min(near, key=lambda x: (x[0], x[1].id))
        gt.neutralized_at = self.t
        self.log.append(self.t, "TARGET_NEUTRALIZED", "WORLD",
                        {"ground_truth": gt.id, "by": drone_id, "estimate_error_m": err}, target_id=target_id)  # fmt: skip

    def target_log_rows(self) -> list[dict]:
        """Merged per-agent logbooks (13.2): one row per target, aliases resolved."""
        alias_of: dict[str, str] = {}
        for agent in self.drones.values():
            alias_of.update(agent.tracker.alias_of)
        logbooks = {i: list(a.logbook) for i, a in self.drones.items()}
        views = {i: [LogEntry.from_record(r, i) for r in a.tracker.records.values()] for i, a in self.drones.items()}
        used: dict[str, int] = {}
        for ev in self.log.of_type("MSG_SENT"):
            if "target_id" in ev:
                tid = ev["target_id"]
                while tid in alias_of and alias_of[tid] != tid:
                    tid = alias_of[tid]
                used[tid] = used.get(tid, 0) + 1
        return merge_logbooks(logbooks, views, alias_of, used)

    def baseline_results(self) -> list[dict]:
        """Decentralized owner vs centralized optimum for every allocation (evaluation only, after the run)."""
        alias = {}
        for d in self.snapshot.drones:
            for tv in d.targets:
                for a in tv.aliases:
                    alias[a] = tv.id
        committed: dict[tuple[str, int], dict[str, str]] = {}
        for ev in self.log.of_type("ALLOCATION_COMMITTED"):
            key = (alias.get(ev["target_id"], ev["target_id"]), ev["epoch"])
            committed.setdefault(key, {})[ev["actor"]] = ev["data"]["winner"]
        planner = Planner(self.fence, self.cfg.navigation.cruise_alt, self.cfg.navigation.planner_clearance)
        cost_cfg = CostConfig(self.cfg.allocation.weights, self.cfg.inspection, self.cfg.confidence)
        params = {d.id: d.params for d in self.cfg.drones}
        out = []
        for (tid, epoch), start in sorted(self.allocation_starts.items(), key=lambda kv: kv[1]["t"]):
            key = (alias.get(tid, tid), epoch)
            bidders = [
                BidderState(v.id, (v.x, v.y, v.alt), v.battery_wh, params[v.id], v.home, v.mode, v.docked,
                            busy=v.current_task is not None and alias.get(v.current_task, v.current_task) != key[0])
                for v in start["drones"] if v.mode != "FAILED"
            ]  # fmt: skip
            winner, bids = optimal(bidders, key[0], start["target_est"], planner, cost_cfg)
            dec = committed.get(key, {})
            out.append({"target_id": key[0], "epoch": epoch, "t": start["t"], "baseline_winner": winner,
                        "committed": dec, "agree": len(set(dec.values())) <= 1,
                        "match": bool(dec) and set(dec.values()) == {winner},
                        "baseline_bids": {b.drone: (b.feasible, b.value, b.reason) for b in bids}})  # fmt: skip
        if out:  # merged duplicates can start several auctions for one target; keep the first per key
            seen, uniq = set(), []
            for r in out:
                if (r["target_id"], r["epoch"]) not in seen:
                    seen.add((r["target_id"], r["epoch"]))
                    uniq.append(r)
            out = uniq
        return out

    def _record(self) -> None:
        self._snapshot = None
        states = [self.drones[i].state for i in sorted(self.drones)]
        for st in states:
            if st.mode.value == "LANDED":
                continue  # an emergency landing is allowed to go below the altitude floor
            reason = self.fence.violation(st.x, st.y, st.alt)
            if reason:
                self.geofence_violations.append((self.t, st.id, reason))
        airborne = [st for st in states if st.alt > 0.0]
        for i, a in enumerate(airborne):
            for b in airborne[i + 1:]:
                pair = (a.id, b.id)
                dist = math.dist(a.pos3, b.pos3)
                if dist < self.cfg.safety.min_separation:
                    if pair not in self._close_pairs:
                        self._close_pairs.add(pair)
                        self.separation_violations.append((self.t, a.id, b.id, dist))
                else:
                    self._close_pairs.discard(pair)

    def _build_snapshot(self) -> Snapshot:
        f = self.fence
        return Snapshot(
            t=self.t,
            tick=self.tick,
            duration=self.cfg.sim.duration,
            finished=self.finished,
            scenario=self.cfg.name,
            seed=self.cfg.sim.seed,
            map_size=self.cfg.sim.map_size,
            boundary=tuple(self.cfg.geofence.boundary),
            fence=FenceView(
                buffered_boundary=tuple(f.boundary.exterior.coords),
                zones=tuple((z.id, tuple(z.raw.exterior.coords), tuple(z.buffered.exterior.coords)) for z in f.zones),
                altitude_limits=(f.min_alt, f.max_alt),
            ),
            drones=tuple(self.drones[i].export_view() for i in sorted(self.drones)),
            ground_truth=tuple(
                GroundTruthView(t.id, t.pos, t.visible(self.t), t.appear_at <= self.t, t.neutralized_at is not None)
                for t in self.ground_truth.targets
            ),
            recent_events=tuple(list(self.log.important)[-RECENT_EVENTS:]),
            geofence_violations=len(self.geofence_violations),
            messages=tuple(list(self.network.history)[-RECENT_MESSAGES:]),
            network_stats=dict(self.network.stats),
            separation_violations=len(self.separation_violations),
            comm_range=self.cfg.network.comm_range,
        )
