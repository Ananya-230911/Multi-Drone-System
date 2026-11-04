"""The owner's mission: fly to the target, keep a lock, log it, come back (PLAN.md 7.9 to 7.11, Phase 4).

Mixed into `Drone`. Only the committed owner runs a mission; everybody else
keeps patrolling. Lifecycle rows driven here (all owner-only):

    7  ALLOCATED -> ENROUTE          validated route to the inspection point
    8  ALLOCATED -> TRACKING         already at the point with a FRESH track
    9  ENROUTE -> TRACKING           in the inspection region with a FRESH track
    10 TRACKING/ENROUTE -> TRACKING_LOST   track LOST
    11 TRACKING_LOST -> REACQUIRING  square-spiral search around the last estimate
    12 REACQUIRING -> TRACKING       fresh observation in the region again
    13 REACQUIRING -> CANCELLED      reacquire timeout, action "cancel"
    14 TRACKING -> LOGGED            inspection conditions (7.10) met
    15 LOGGED -> COMPLETED           logbook entry written and TARGET_LOGGED broadcast
    16 reopen (release)              battery reserve, route unreachable, reacquire timeout "reallocate"
    17 cue rejected on site          an external cue the owner's own sensor cannot confirm

Inspection conditions (7.10): confirmation CONFIRMED; owner inside the inspection
region (horizontal distance <= its r_eff at current altitude) and geofence-valid;
accumulated time with a FRESH track and confidence >= inspection.min_confidence
reaches inspection.duration. `timer_on_loss` = pause keeps the time when the track
goes stale, reset restarts it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from src.allocation.cost import BidderState, Stop, evaluate, inspection_standoff
from src.drone.fsm import MISSION, DroneMode
from src.drone.sensor import effective_radius
from src.logging_.target_log import LogEntry
from src.safety.planner import inspection_point
from src.tracking.target_state import (
    Confirmation,
    IllegalTransition,
    TargetState,
    TransitionContext,
    apply_transition,
)

if TYPE_CHECKING:  # pragma: no cover
    from src.config import Point2, Point3
    from src.drone.agent import Drone

S = TargetState


@dataclass
class Mission:
    target_id: str
    epoch: int
    started: float
    point: "Point3 | None" = None
    arrived_at: float | None = None
    accum: float = 0.0  # seconds counted toward inspection.duration
    inspection_start: float | None = None
    confidences: list[float] = field(default_factory=list)  # confidences while the timer ran
    reacq_start: float | None = None
    last_progress: int = -1  # last whole second reported in INSPECTION_PROGRESS


def spiral_points(center: "Point2", step: float, max_length: float) -> list["Point2"]:
    """Square spiral around `center`: legs of 1, 1, 2, 2, 3, 3, ... steps, up to `max_length` metres."""
    x, y = center
    pts, length, k = [], 0.0, 1
    dirs = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    d = 0
    while length < max_length:
        for _ in range(2):
            dx, dy = dirs[d % 4]
            x, y = x + dx * step * k, y + dy * step * k
            pts.append((x, y))
            length += step * k
            d += 1
            if length >= max_length:
                break
        k += 1
    return pts


class MissionMixin:
    def _init_mission(self: "Drone") -> None:
        self.mission: Mission | None = None
        self.logbook: list[LogEntry] = []
        self.queue: list[tuple[str, int]] = []  # tasks won while busy (target, epoch), flown one after another

    # ------------------------------------------------------------------ per tick

    def _mission_step(self: "Drone", now: float, dt: float) -> bool:
        """Run the owner's mission. Returns True if it decided this tick (skip patrol logic)."""
        s = self.state
        tid = s.current_task
        rec = self.tracker.record(tid) if tid else None
        still_owner = rec is not None and not rec.terminal and rec.owner == self.id and rec.epoch == s.task_epoch
        if not still_owner:
            if tid is not None or s.mode in MISSION:
                self._end_mission(now, "ownership_lost" if rec is not None and not rec.terminal else "task_ended")
                return True
            return False
        if s.mode in (DroneMode.PATROL, DroneMode.CONFIRM_HOLD, DroneMode.RETURNING) and rec.state is S.ALLOCATED \
                and self.mission is None:  # RETURNING: straight on to the next queued task  # fmt: skip
            self._start_mission(now, rec)
            return True
        if self.mission is None:
            return False
        if s.mode is DroneMode.ENROUTE or (s.mode is DroneMode.INSPECTING and rec.state is S.ENROUTE):
            self._enroute_step(now, rec)
        elif s.mode is DroneMode.INSPECTING and rec.state is S.TRACKING:
            self._inspect_step(now, dt, rec)
        elif s.mode is DroneMode.REACQUIRING:
            self._reacquire_step(now, rec)
        return s.mode in MISSION

    # ------------------------------------------------------------------ start / arrive

    def _start_mission(self: "Drone", now: float, rec) -> None:
        s = self.state
        insp = self.shared.inspection
        self.mission = m = Mission(rec.id, rec.epoch, now)
        r_eff = effective_radius(s.params.sensor, insp.standoff_alt)
        standoff = inspection_standoff(r_eff, self.shared.confidence, insp)
        m.point = inspection_point(rec.est_pos, r_eff, self.planner, insp.standoff_alt, insp.candidate_count,
                                   from_pos=s.pos, standoff=standoff)  # fmt: skip
        if m.point is None:
            self._release(now, "route_unreachable", detail="no_inspection_point")
            return
        tol = self.shared.navigation
        if math.dist(s.pos, m.point[:2]) <= tol.arrival_tolerance and abs(s.alt - m.point[2]) <= tol.alt_tolerance \
                and self.tracker.freshness(rec.id, now) == "FRESH":  # fmt: skip
            # Row 8: already in position with a fresh track.
            self._lifecycle(now, rec, S.TRACKING, "already_in_position", inspection_point_valid=True, track_fresh=True)
            m.arrived_at = now
            self._set_mode(now, DroneMode.INSPECTING, f"in_position:{rec.id}")
            return
        route = self._plan(now, m.point, "mission")
        if route is None:
            self._release(now, "route_unreachable", detail="no_route")
            return
        self._lifecycle(now, rec, S.ENROUTE, "route_valid", route_valid=True)
        self._set_mode(now, DroneMode.ENROUTE, f"owner:{rec.id}")
        self._set_route(route, "hover")

    def _enroute_step(self: "Drone", now: float, rec) -> None:
        s, m = self.state, self.mission
        if s.mode is DroneMode.ENROUTE:
            tol = self.shared.navigation
            if not (self.navigator.finished and math.dist(s.pos, m.point[:2]) <= tol.arrival_tolerance
                    and abs(s.alt - m.point[2]) <= tol.alt_tolerance):  # fmt: skip
                return
            m.arrived_at = now
            self._set_mode(now, DroneMode.INSPECTING, f"arrived:{rec.id}")
        fresh = self.tracker.freshness(rec.id, now)
        in_region = self._in_region(rec)
        if fresh == "FRESH" and in_region:
            self._lifecycle(now, rec, S.TRACKING, "fresh_track_in_region", in_region=True, track_fresh=True)
        elif rec.is_cue and self._cue_timed_out(now, rec):
            self._reject_cue(now, rec)
        elif fresh == "LOST" and now - m.arrived_at >= self.shared.track.lost_after:
            self._lose_track(now, rec)

    # ------------------------------------------------------------------ inspect

    def _in_region(self: "Drone", rec) -> bool:
        s = self.state
        return math.dist(s.pos, rec.est_pos) <= effective_radius(s.params.sensor, s.alt) \
            and self.fence.violation(*s.pos3) is None  # fmt: skip

    def _inspect_step(self: "Drone", now: float, dt: float, rec) -> None:
        m, insp = self.mission, self.shared.inspection
        fresh = self.tracker.freshness(rec.id, now)
        if fresh == "LOST":
            self._lose_track(now, rec)
            return
        if rec.is_cue and self._cue_timed_out(now, rec):
            self._reject_cue(now, rec)
            return
        counting = (fresh == "FRESH" and rec.last_confidence >= insp.min_confidence and self._in_region(rec)
                    and rec.confirmation is Confirmation.CONFIRMED)  # fmt: skip
        if counting:
            if m.inspection_start is None:
                m.inspection_start = now
            m.accum += dt
            m.confidences.append(rec.last_confidence)
        elif fresh != "FRESH" and insp.timer_on_loss == "reset":
            m.accum = 0.0
        if int(m.accum) != m.last_progress:
            m.last_progress = int(m.accum)
            self._emit(now, "INSPECTION_PROGRESS", {"seconds": m.accum, "of": insp.duration, "freshness": fresh,
                                                   "confidence": rec.last_confidence}, target_id=rec.id, epoch=rec.epoch)  # fmt: skip
        if m.accum + 1e-9 >= insp.duration:
            self._log_target(now, rec)

    def _log_target(self: "Drone", now: float, rec) -> None:
        m = self.mission
        if not self._lifecycle(now, rec, S.LOGGED, "inspection_conditions_met", inspection_conditions_met=True):
            return
        entry = LogEntry.from_record(rec, self.id, inspection_start=m.inspection_start, logged_at=now,
                                     inspection_duration=m.accum,
                                     mean_confidence=sum(m.confidences) / len(m.confidences) if m.confidences else 0.0)  # fmt: skip
        self.logbook.append(entry)
        self._emit(now, "TARGET_LOGGED", entry.as_dict(), target_id=rec.id, epoch=rec.epoch)
        self.outbox.send("TARGET_LOGGED", now, {"result": entry.as_dict(), "summary": self.tracker.summary(rec.id)},
                         target_id=rec.id, epoch=rec.epoch)  # fmt: skip
        self._lifecycle(now, rec, S.COMPLETED, "logged_and_broadcast", logged=True)
        self._emit(now, "TASK_COMPLETED", {"est_pos": list(rec.est_pos), "duration": m.accum}, target_id=rec.id, epoch=rec.epoch)
        self._end_mission(now, "completed")

    # ------------------------------------------------------------------ lose / reacquire

    def _lose_track(self: "Drone", now: float, rec) -> None:
        if not self._lifecycle(now, rec, S.TRACKING_LOST, "track_lost", track_lost=True, in_region=True):
            return
        self._emit(now, "TRACK_LOST", {"last_obs": rec.t_last_obs, "est_pos": list(rec.est_pos)}, target_id=rec.id, epoch=rec.epoch)
        route = self._spiral_route(now, rec)
        if route is None:
            self._release(now, "route_unreachable", detail="no_search_route")
            return
        if not self._lifecycle(now, rec, S.REACQUIRING, "spiral_search", battery_ok=True, route_valid=True):
            return
        self.mission.reacq_start = now
        self._emit(now, "REACQUIRE_START", {"waypoints": len(route)}, target_id=rec.id, epoch=rec.epoch)
        self._set_mode(now, DroneMode.REACQUIRING, f"track_lost:{rec.id}")
        self._set_route(route, "hover")

    def _spiral_route(self: "Drone", now: float, rec) -> "list[Point3] | None":
        """Inspection point of the last estimate, then a square spiral; every leg planned and validated."""
        s, rq = self.state, self.shared.reacquire
        alt = self.shared.inspection.standoff_alt
        pts = [(rec.est_pos[0], rec.est_pos[1], alt)]
        pts += [(x, y, alt) for x, y in spiral_points(rec.est_pos, rq.leg_step, s.params.max_speed * rq.timeout)]
        route: list = [s.pos3]
        for p in pts:
            if self.fence.violation(*p):
                continue  # clipped to the geofence
            leg = self.planner.plan(route[-1], p)
            if leg.route is None:
                continue
            route += leg.route[1:]
        if len(route) < 2:
            return None
        self._emit(now, "ROUTE_PLANNED", {"kind": "reacquire", "waypoints": [list(w) for w in route], "length": 0.0})
        return route

    def _reacquire_step(self: "Drone", now: float, rec) -> None:
        m, rq = self.mission, self.shared.reacquire
        if rec.state is not S.REACQUIRING:
            return
        if rec.is_cue and self._cue_timed_out(now, rec):
            self._reject_cue(now, rec)
            return
        if self.tracker.freshness(rec.id, now) == "FRESH" and self._in_region(rec):
            self._lifecycle(now, rec, S.TRACKING, "reacquired", in_region=True, track_fresh=True)
            self._emit(now, "REACQUIRE_SUCCESS", {"after": now - m.reacq_start}, target_id=rec.id, epoch=rec.epoch)
            self._set_mode(now, DroneMode.INSPECTING, f"reacquired:{rec.id}")
            self._return_to_inspection_point(now, rec)
            return
        if now - m.reacq_start >= rq.timeout:
            self._emit(now, "REACQUIRE_TIMEOUT", {"action": rq.timeout_action}, target_id=rec.id, epoch=rec.epoch)
            if rq.timeout_action == "cancel":
                self._lifecycle(now, rec, S.CANCELLED, "reacquire_timeout", trigger="reacquire_timeout", reacquire_action="cancel")
                self._emit(now, "TASK_CANCELLED", {"reason": "reacquire_timeout"}, target_id=rec.id, epoch=rec.epoch)
                self._end_mission(now, "reacquire_timeout")
            else:
                self._release(now, "reacquire_timeout")

    def _return_to_inspection_point(self: "Drone", now: float, rec) -> None:
        """After a reacquisition the spiral may have left the drone far from the target, where the
        confidence is too low for the timer: fly back to an inspection point of the new estimate."""
        s, insp, m = self.state, self.shared.inspection, self.mission
        r_eff = effective_radius(s.params.sensor, insp.standoff_alt)
        point = inspection_point(rec.est_pos, r_eff, self.planner, insp.standoff_alt, insp.candidate_count,
                                 from_pos=s.pos, standoff=inspection_standoff(r_eff, self.shared.confidence, insp))  # fmt: skip
        route = self._plan(now, point, "mission") if point is not None else None
        if route is None:
            from src.drone.navigation import Navigator

            self.navigator = Navigator([s.pos3], loop=False)  # hold where it is
            self.on_patrol_loop, self._then = False, "hover"
            return
        m.point = point
        self._set_route(route, "hover")

    # ------------------------------------------------------------------ cues

    def _cue_timed_out(self: "Drone", now: float, rec) -> bool:
        m = self.mission
        return (rec.confirmation is Confirmation.PENDING and m.arrived_at is not None
                and now - m.arrived_at > self.shared.confirmation.timeout)  # fmt: skip

    def _reject_cue(self: "Drone", now: float, rec) -> None:
        rec.confirmation = Confirmation.REJECTED
        self._emit(now, "CONFIRMATION_RESULT", {"result": "REJECTED", "via": "on_site_timeout"}, target_id=rec.id, epoch=rec.epoch)
        if self._lifecycle(now, rec, S.CANCELLED, "cue_not_confirmed", in_region=True):
            self.logbook.append(LogEntry.from_record(rec, self.id))
            self._emit(now, "TASK_CANCELLED", {"reason": "cue_not_confirmed"}, target_id=rec.id, epoch=rec.epoch)
        self._end_mission(now, "cue_not_confirmed")

    # ------------------------------------------------------------------ release / end

    def _release(self: "Drone", now: float, reason: str, detail: str = "") -> None:
        """Give the task back (row 16): epoch + 1, this drone excluded; or cancel when reopens run out."""
        tid = self.state.current_task
        if tid:
            self._release_target(now, tid, reason, detail)
        self._end_mission(now, f"released:{reason}")

    def _release_target(self: "Drone", now: float, tid: str, reason: str, detail: str = "", exclude: bool = True) -> None:
        rec = self.tracker.record(tid) if tid else None
        if rec is not None and not rec.terminal:
            to = S.DETECTED if rec.is_cue else S.CONFIRMED
            trig = {"battery_reserve": "battery_reserve", "route_unreachable": "route_unreachable"}.get(reason, "reacquire_timeout")
            old_epoch = rec.epoch
            self.outbox.send("RELEASE", now, {"reason": reason, "detail": detail}, target_id=rec.id, epoch=old_epoch)
            if not self._lifecycle(now, rec, to, f"release:{reason}", trigger=trig, exclude_owner=exclude,
                                   reacquire_action="reallocate", max_reallocations=self.shared.allocation.max_reallocations):  # fmt: skip
                self._lifecycle(now, rec, S.CANCELLED, "max_reallocations", trigger="max_reallocations")
                self._emit(now, "TASK_CANCELLED", {"reason": "max_reallocations"}, target_id=rec.id, epoch=rec.epoch)
            else:
                self._emit(now, "REALLOCATION", {"reason": reason, "detail": detail, "released_by": self.id,
                                                 "new_epoch": rec.epoch}, target_id=rec.id, epoch=rec.epoch)  # fmt: skip

    # ------------------------------------------------------------------ task queue (allocation.max_queue > 1)

    def _queued_ok(self: "Drone", tid: str, epoch: int):
        rec = self.tracker.record(tid)
        ok = rec is not None and not rec.terminal and rec.owner == self.id and rec.epoch == epoch and rec.state is S.ALLOCATED
        return rec if ok else None

    def _stops(self: "Drone") -> tuple[Stop, ...]:
        """The tasks this drone holds, active one first, as the bid function needs them."""
        s, insp = self.state, self.shared.inspection
        out: list[Stop] = []
        rec = self.tracker.record(s.current_task) if s.current_task else None
        if rec is not None and not rec.terminal and rec.owner == self.id:
            m = self.mission
            on = m is not None and m.target_id == rec.id
            out.append(Stop(rec.est_pos, max(insp.duration - m.accum, 0.0) if on else insp.duration,
                            m.point if on else None, True))  # fmt: skip
        for tid, epoch in self.queue:
            r = self._queued_ok(tid, epoch)
            if r is not None:
                out.append(Stop(r.est_pos, insp.duration))
        return tuple(out)

    def _free_return(self: "Drone") -> bool:
        """Flying back to the patrol loop (not home for its battery), or on its way to its next queued task: free to take new work."""
        return self.state.mode is DroneMode.RETURNING and (self._then == "patrol" or self.state.current_task is not None)

    def _accept_win(self: "Drone", now: float, tid: str, epoch: int) -> None:
        """This drone won `tid`. An idle drone starts on it; a busy one re-checks the task against what it holds NOW
        (the bid was computed earlier) and queues it, or gives it back so the others can re-auction it."""
        s = self.state
        if s.current_task in (None, tid):
            s.current_task, s.task_epoch = tid, epoch
            return
        if any(q == tid for q, _ in self.queue):
            return
        reason = "queue_full"
        al = self.shared.allocation
        rec = self.tracker.record(tid)
        stops = self._stops()
        if al.max_queue > 1 and len(stops) < al.max_queue and rec is not None:
            bidder = BidderState(self.id, s.pos3, self.battery.charge_wh, s.params, s.home, s.mode.value, self.docked,
                                 busy=True, stops=stops, free_return=self._free_return())  # fmt: skip
            bid = evaluate(bidder, tid, rec.est_pos, self.planner, self._cost_cfg)
            if bid.feasible:
                pos = max(bid.breakdown["insert_at"] - (1 if stops and stops[0].active else 0), 0)
                self.queue.insert(min(pos, len(self.queue)), (tid, epoch))
                self._emit(now, "TASK_QUEUED", {"position": pos, "held": len(stops) + 1, "cost": bid.breakdown["cost"]},
                           target_id=tid, epoch=epoch)  # fmt: skip
                return
            reason = f"not_feasible_now:{bid.reason}"
        elif al.max_queue <= 1:
            reason = "busy"
        # It only lacked room: the next auction needs no exclusion (its own bid will say queue_full / not feasible),
        # and excluding it would strand the task when it is the only drone left.
        self._release_target(now, tid, reason, exclude=False)

    def _release_queue(self: "Drone", now: float, reason: str) -> None:
        queued, self.queue = self.queue, []
        for tid, epoch in queued:
            if self._queued_ok(tid, epoch) is not None:
                self._release_target(now, tid, reason)

    def _take_next(self: "Drone", now: float) -> tuple[str, int] | None:
        """Next queued task that is still ours and still affordable; the others are given back."""
        s = self.state
        while self.queue:
            tid, epoch = self.queue.pop(0)
            rec = self._queued_ok(tid, epoch)
            if rec is None:
                continue
            bidder = BidderState(self.id, s.pos3, self.battery.charge_wh, s.params, s.home, "PATROL", self.docked)
            if evaluate(bidder, tid, rec.est_pos, self.planner, self._cost_cfg).feasible:
                return tid, epoch
            self._release_target(now, tid, "no_longer_feasible")
        return None

    def _end_mission(self: "Drone", now: float, reason: str) -> None:
        s = self.state
        self.mission = None
        s.current_task = None
        nxt = self._take_next(now) if self.queue else None
        if nxt is not None:  # fly straight on to the next task instead of going back to the patrol loop
            s.current_task, s.task_epoch = nxt
            if s.mode in MISSION:
                self._set_mode(now, DroneMode.RETURNING, reason)
        elif s.mode in MISSION:
            self._set_mode(now, DroneMode.RETURNING, reason)
            self._back_to_patrol(now)
        elif s.mode is DroneMode.SAFETY_HOLD and self._safety_prev in MISSION:
            idx = self._nearest_patrol_index()
            self._safety_prev = DroneMode.RETURNING
            self._safety_goal = (s.patrol_route[idx], "patrol", idx)

    def _back_to_patrol(self: "Drone", now: float) -> None:
        """RETURNING -> PATROL: fly to the nearest patrol waypoint (6.2)."""
        idx = self._nearest_patrol_index()
        self._go_patrol(now, idx)

    # ------------------------------------------------------------------ lifecycle helper

    def _lifecycle(self: "Drone", now: float, rec, to: TargetState, reason: str, **ctx) -> bool:
        """Apply an owner transition locally, log it, and tell the peers (TARGET_STATE)."""
        c = TransitionContext(actor=self.id, now=now, **ctx)
        try:
            ev = apply_transition(rec, to, c, reason)
        except IllegalTransition as exc:
            self._emit(now, "ILLEGAL_TRANSITION", {"to": to.value, "error": str(exc)}, target_id=rec.id, epoch=rec.epoch)
            return False
        data = {"from": ev.frm.value, "to": ev.to.value, "row": ev.row, "reason": ev.reason, "by": self.id}
        self._emit(now, "TARGET_TRANSITION", data, target_id=rec.id, epoch=ev.epoch)
        self._announce_state(now, rec.id, data)
        return True
