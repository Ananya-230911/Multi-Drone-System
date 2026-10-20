"""Run a web-map scenario headless and return everything the page needs to replay it.

The simulation runs to completion on the server; the page then animates the recorded
flight. Frames are sampled every `FRAME_S` simulated seconds; the page interpolates
between them. Coordinates are returned as latitude / longitude.
"""

from __future__ import annotations

from typing import Any

from src.config import ConfigError
from src.sim.world import World
from src.viz.describe import describe
from src.webmap.geo import GeoFrame
from src.webmap.scenario_builder import RequestError, build_config

FRAME_S = 0.5
AFTER_DONE_S = 12.0  # keep recording this long after the mission ends (drone flies home)
EVENT_TYPES = {
    "DRONE_MODE", "TARGET_TRANSITION", "ALLOCATION_COMMITTED", "ALLOCATION_FAILED", "REALLOCATION",
    "TARGET_LOGGED", "TARGET_NEUTRALIZED", "TASK_COMPLETED", "TASK_CANCELLED", "SAFETY_HOLD", "SAFETY_REPLAN",
    "SAFETY_RETURN", "SAFETY_LAND", "SEPARATION_YIELD", "DRONE_FAILED", "TRACK_LOST", "REACQUIRE_START",
    "REACQUIRE_SUCCESS", "REACQUIRE_TIMEOUT", "CONFIRMATION_RESULT", "SCENARIO_EVENT", "CLAIM_CONFLICT",
}  # fmt: skip
DONE_EVENTS = {"TARGET_NEUTRALIZED", "TARGET_LOGGED", "TASK_CANCELLED", "ALLOCATION_FAILED"}
MODES = ["PATROL", "CONFIRM_HOLD", "ENROUTE", "INSPECTING", "REACQUIRING", "RETURNING", "SAFETY_HOLD", "LANDED", "FAILED"]
TARGET_STATES = ["NONE", "DETECTED", "CONFIRMED", "ALLOCATED", "ENROUTE", "TRACKING", "TRACKING_LOST", "REACQUIRING",
                 "LOGGED", "COMPLETED", "CANCELLED"]  # fmt: skip


def _target_view(snap) -> tuple[str, str | None, str | None, float | None, tuple | None]:
    """(state, target id, owner, lock seconds, est position) of the most relevant record, from the drones' views."""
    best = None
    for d in snap.drones:
        for tv in d.targets:
            # live drones only (a failed drone's view is frozen), newest allocation round first
            rank = (d.mode != "FAILED", tv.epoch, tv.owner == d.id, tv.state != "CANCELLED", tv.created_at)
            if best is None or rank > best[0]:
                best = (rank, d, tv)
    if best is None:
        return "NONE", None, None, None, None
    _, d, tv = best
    owner_view = next((x for x in snap.drones if x.id == tv.owner and x.mode != "FAILED"), d)
    own = next((t for t in owner_view.targets if t.id == tv.id), tv)
    return own.state, own.id, own.owner, own.insp_seconds, own.est_pos


def run(req: dict) -> dict[str, Any]:
    cfg, fr = build_config(req)
    try:
        w = World(cfg)
    except ConfigError as exc:
        raise RequestError(f"the scenario is not valid: {exc}") from exc
    ids = [d.id for d in sorted(cfg.drones, key=lambda d: d.id)]
    frames: list[dict] = []
    seen = 0
    done_at: float | None = None
    every = max(1, round(FRAME_S / w.dt))
    while not w.finished:
        w._advance()
        recs = w.log.records
        for r in recs[seen:]:
            if r["type"] in DONE_EVENTS and done_at is None:
                done_at = w.t
        seen = len(recs)
        if w.tick % every == 0 or w.tick == 1:
            frames.append(_frame(w, fr))
        if done_at is not None and w.t >= done_at + AFTER_DONE_S:
            break
    if not frames or frames[-1]["t"] < w.t - 1e-9:
        frames.append(_frame(w, fr))
    return {
        "frame_s": FRAME_S,
        "drone_ids": ids,
        "modes": MODES,
        "target_states": TARGET_STATES,
        "frames": frames,
        "events": _events(w),
        "allocations": _allocations(w),
        "routes": _routes(w, fr),
        "zones": [[list(fr.to_latlon(*p)) for p in z.raw.exterior.coords[:-1]] for z in w.fence.zones],
        "target": {"lat": req["target"]["lat"], "lon": req["target"]["lon"]},
        "summary": _summary(w),
        "t_end": w.t,
    }


def _frame(w: World, fr: GeoFrame) -> dict:
    snap = w.snapshot
    state, tid, owner, lock, est = _target_view(snap)
    drones = []
    for d in snap.drones:
        lat, lon = fr.to_latlon(d.x, d.y)
        drones.append([round(lat, 7), round(lon, 7), round(d.alt, 1), MODES.index(d.mode), round(d.battery_pct, 4),
                       round(d.r_eff, 1), 1 if d.docked else 0])  # fmt: skip
    gt = snap.ground_truth[0] if snap.ground_truth else None
    return {
        "t": round(w.t, 2), "d": drones, "ts": TARGET_STATES.index(state), "owner": owner,
        "lock": None if lock is None else round(lock, 1), "neutralized": bool(gt and gt.neutralized),
        "gt_visible": bool(gt and gt.visible),
    }  # fmt: skip


def _events(w: World) -> list[dict]:
    out = []
    shown: set[tuple] = set()  # every drone repeats the same lifecycle step; show the first one
    for e in w.log.records:
        if e["type"] not in EVENT_TYPES:
            continue
        data = e.get("data", {})
        if e["type"] == "TARGET_TRANSITION" and str(data.get("reason", "")).startswith(("adopted_from", "relayed_from")):
            continue  # peers copying the owner's progress; still in events.jsonl
        if e["type"] == "ALLOCATION_COMMITTED" and e["actor"] != data.get("winner"):
            continue  # every agent commits; show the owner's own record only
        if e["type"] == "TARGET_TRANSITION" and data.get("row") is None:
            continue
        if e["type"] == "CONFIRMATION_RESULT" and str(data.get("via", "")).startswith("peer_evidence"):
            continue  # a peer re-checking shared evidence; the owner's own confirmation is shown
        if e["type"] in ("TARGET_TRANSITION", "CONFIRMATION_RESULT"):
            key = (e["type"], e.get("target_id"), e.get("epoch"), data.get("to") or data.get("result"))
            if key in shown:
                continue
            shown.add(key)
        out.append({"t": e["t"], "type": e["type"], "actor": e["actor"], "text": describe(e)})
    return out


def _allocations(w: World) -> list[dict]:
    """Bid tables (one per allocation round) with the committed winner, for the page's BIDS panel."""
    rounds: dict[tuple[str, int], dict] = {}
    for e in w.log.of_type("BID_COMPUTED"):
        key = (e["target_id"], e["epoch"])
        r = rounds.setdefault(key, {"target_id": key[0], "epoch": key[1], "t": e["t"], "bids": [], "winner": None,
                                    "committed_at": None})  # fmt: skip
        b, d = e["data"]["breakdown"], e["data"]
        r["bids"].append({
            "drone": e["actor"], "feasible": d["feasible"], "reason": d["reason"], "cost": b.get("cost"),
            "t_go": b.get("t_go"), "E_need": b.get("E_need"), "batt_after": b.get("batt_after"),
            "L_go": b.get("L_go"), "climb": b.get("climb"), "descent": b.get("descent"), "r_eff": b.get("r_eff"),
            "battery_wh": b.get("battery_wh"), "reserve_wh": b.get("reserve_wh"),
        })  # fmt: skip
    for e in w.log.of_type("ALLOCATION_COMMITTED"):
        r = rounds.get((e["target_id"], e["epoch"]))
        if r is not None and r["winner"] is None:
            r["winner"], r["committed_at"] = e["data"]["winner"], e["t"]
    out = sorted(rounds.values(), key=lambda r: r["t"])
    for r in out:
        r["bids"].sort(key=lambda b: b["drone"])
    return out


def _routes(w: World, fr: GeoFrame) -> list[dict]:
    out = []
    for e in w.log.of_type("ROUTE_PLANNED"):
        if e["data"]["kind"] in ("mission", "replan", "home", "reacquire"):
            out.append({"t": e["t"], "actor": e["actor"], "kind": e["data"]["kind"],
                        "points": [[*fr.to_latlon(p[0], p[1])] for p in e["data"]["waypoints"]]})  # fmt: skip
    return out


def _summary(w: World) -> dict:
    rows = w.target_log_rows()
    base = w.baseline_results()
    return {
        "target_log": [{k: r.get(k) for k in ("target_id", "final_state", "owner", "detected_at", "confirmed_at",
                                              "allocated_at", "inspection_start", "logged_at", "inspection_duration",
                                              "mean_confidence", "cancel_reason", "reallocations", "messages_used",
                                              "duplicate_inspection")} for r in rows],  # fmt: skip
        "baseline_match": [r["match"] for r in base],
        "baseline_winner": [r["baseline_winner"] for r in base],
        "geofence_violations": len(w.geofence_violations),
        "separation_violations": len(w.separation_violations),
        "messages": dict(w.network.stats),
        "neutralized": [e["data"] for e in w.log.of_type("TARGET_NEUTRALIZED")],
    }
