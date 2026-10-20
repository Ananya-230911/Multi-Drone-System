"""Plain-text description of an event, shared by the pygame panel and the web map."""

from __future__ import annotations


def describe(e: dict) -> str:
    d = e.get("data", {})
    t = f"{e['t']:7.1f}"
    tid = e.get("target_id", "")
    if e["type"] == "DRONE_MODE":
        return f"{t} {e['actor']} {d.get('from')} -> {d.get('to')} ({d.get('reason')})"
    if e["type"] == "SCENARIO_EVENT":
        who = f" {d['drone']}" if "drone" in d else (f" {d['target']}" if "target" in d else "")
        ok = "" if d.get("applied") else " [ignored]"
        return f"{t} scenario {d.get('type')}{who}{ok}"
    if e["type"] == "SAFETY_RETURN":
        e_home = d.get("e_home_wh")
        return f"{t} {e['actor']} SAFETY_RETURN {d.get('reason')}" + (f" (E_home {e_home:.1f} Wh)" if e_home is not None else "")
    if e["type"] in ("SAFETY_HOLD", "SAFETY_REPLAN", "SAFETY_LAND"):
        return f"{t} {e['actor']} {e['type']} {d.get('reason', d.get('result', ''))}"
    if e["type"] == "HOME_REACHED":
        nxt = "resume patrol" if d.get("resume_patrol") else "parked"
        return f"{t} {e['actor']} home, batt {d.get('battery_pct', 0) * 100:.1f}% -> {nxt}"
    if e["type"] == "TARGET_TRANSITION":
        return f"{t} {e['actor']} {tid} {d.get('from')} -> {d.get('to')} (row {d.get('row')}, {d.get('reason')})"
    if e["type"] == "CONFIRMATION_RESULT":
        via = f" via {d['via']}" if "via" in d else ""
        return f"{t} {e['actor']} {tid} {d.get('result')} ({d.get('observations_in_window')} obs, mean {d.get('mean_confidence', 0):.2f}){via}"
    if e["type"] == "TARGET_MERGED":
        return f"{t} {e['actor']} merged {d.get('alias')} into {d.get('survivor')}"
    if e["type"] == "TARGET_CREATED":
        src = f" (from {d['relayed_from']})" if "relayed_from" in d else ""
        return f"{t} {e['actor']} created {tid}{src}"
    if e["type"] == "BID_COMPUTED":
        if d.get("feasible"):
            return f"{t} {e['actor']} bid {d.get('bid'):.2f} for {tid}"
        return f"{t} {e['actor']} cannot bid for {tid}: {d.get('reason')}"
    if e["type"] == "ALLOCATION_COMMITTED":
        return f"{t} {e['actor']} committed owner {d.get('winner')} for {tid} ({d.get('how')})"
    if e["type"] == "ALLOCATION_FAILED":
        return f"{t} {e['actor']} allocation failed for {tid}: {d.get('reason')}"
    if e["type"] == "CLAIM_CONFLICT":
        return f"{t} {e['actor']} claim conflict on {tid}: {d.get('result', d.get('reason'))}"
    if e["type"] == "TARGET_NEUTRALIZED":
        return f"{t} {d.get('ground_truth')} NEUTRALIZED by {d.get('by')} ({tid})"
    if e["type"] == "TARGET_LOGGED":
        return f"{t} {e['actor']} LOGGED {tid} ({d.get('inspection_duration', 0):.0f} s lock, mean conf {d.get('mean_confidence', 0):.2f})"
    if e["type"] in ("TASK_COMPLETED", "TASK_CANCELLED", "TRACK_LOST", "REACQUIRE_START", "REACQUIRE_SUCCESS", "REACQUIRE_TIMEOUT"):
        return f"{t} {e['actor']} {e['type']} {tid} {d.get('reason', '')}"
    if e["type"] == "REALLOCATION":
        return f"{t} {e['actor']} REALLOCATE {tid}: {d.get('reason')} (epoch {d.get('new_epoch', e.get('epoch'))})"
    if e["type"] == "SEPARATION_YIELD":
        return f"{t} {e['actor']} yields to {d.get('to')} (separation)"
    if e["type"] == "ROUTE_REJECTED":
        return f"{t} {e['actor']} ROUTE_REJECTED {d.get('kind')} {d.get('reason')}"
    if e["type"] == "OBSERVATION_REJECTED":
        return f"{t} {e['actor']} observation rejected: {d.get('reason')}"
    return f"{t} {e['actor']} {e['type']} {tid}"
