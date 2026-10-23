"""Validation of what the crew proposes, and conversion into a web-map request.

The language model proposes numbers; nothing it says is trusted until it passes these checks.
A failed check returns a message the agent is asked to fix (CrewAI guardrail), and after the
retries the offline planner supplies that part instead.
"""

from __future__ import annotations

import math
from typing import Any

from src.sim.world import World
from src.webmap.geo import GeoFrame
from src.webmap.scenario_builder import build_config

SIDE_MIN, SIDE_MAX = 800.0, 12_000.0
EDGE = 150.0  # drones are kept this far inside the area edge (the builder needs 120)
MIN_TARGET_EDGE = 300.0
MIN_DRONE_GAP = 70.0


class PlanError(ValueError):
    """A proposal is out of range or inconsistent. The text goes back to the agent."""


def _f(d: dict, key: str, lo: float, hi: float, what: str) -> float:
    try:
        v = float(d[key])
    except (KeyError, TypeError, ValueError):
        raise PlanError(f"{what}: '{key}' is missing or not a number") from None
    if not lo <= v <= hi:
        raise PlanError(f"{what}: '{key}' must be between {lo:g} and {hi:g}, got {v:g}")
    return v


def check_place(d: Any, seen: list[dict]) -> dict:
    """The scout must pick a place that its search tool really returned."""
    if not isinstance(d, dict):
        raise PlanError("answer must be a JSON object with name, lat, lon")
    lat, lon = _f(d, "lat", -85, 85, "place"), _f(d, "lon", -180, 180, "place")
    for c in seen:
        if abs(c["lat"] - lat) < 2e-3 and abs(c["lon"] - lon) < 2e-3:
            return {"name": str(d.get("name") or c["name"])[:140], "lat": c["lat"], "lon": c["lon"],
                    "width_m": c.get("width_m", 0), "height_m": c.get("height_m", 0),
                    "reason": str(d.get("reason", ""))[:300]}  # fmt: skip
    raise PlanError("lat/lon must be copied exactly from one of the search_place results; "
                    "call search_place first and choose one of its results")


def check_area(d: Any) -> dict:
    if not isinstance(d, dict):
        raise PlanError("answer must be a JSON object")
    w = _f(d, "width_m", SIDE_MIN, SIDE_MAX, "area")
    h = _f(d, "height_m", SIDE_MIN, SIDE_MAX, "area")
    dx = _f(d, "target_dx_m", -SIDE_MAX, SIDE_MAX, "area")
    dy = _f(d, "target_dy_m", -SIDE_MAX, SIDE_MAX, "area")
    if abs(dx) > w / 2 - MIN_TARGET_EDGE or abs(dy) > h / 2 - MIN_TARGET_EDGE:
        raise PlanError(f"target offset must keep the target at least {MIN_TARGET_EDGE:.0f} m inside the area: "
                        f"|target_dx_m| <= {w / 2 - MIN_TARGET_EDGE:.0f} and |target_dy_m| <= {h / 2 - MIN_TARGET_EDGE:.0f}")  # fmt: skip
    mode = d.get("mode", "neutralize")
    if mode not in ("neutralize", "inspect"):
        raise PlanError("mode must be 'neutralize' or 'inspect'")
    nf = d.get("no_fly", False)
    if not isinstance(nf, bool):
        raise PlanError("no_fly must be true or false")
    return {"width_m": round(w), "height_m": round(h), "target_dx_m": round(dx), "target_dy_m": round(dy),
            "mode": mode, "no_fly": nf, "reason": str(d.get("reason", ""))[:300]}  # fmt: skip


def check_fleet(d: Any, area: dict) -> dict:
    drones = d.get("drones") if isinstance(d, dict) else d
    if not isinstance(drones, list) or not 3 <= len(drones) <= 6:
        raise PlanError("provide a list of 3 to 6 drones under the key 'drones'")
    far = 0.47 * min(area["width_m"], area["height_m"]) - EDGE
    out = []
    for i, x in enumerate(drones):
        if not isinstance(x, dict):
            raise PlanError(f"drone {i + 1} must be an object")
        what = f"drone {i + 1}"
        bat = _f(x, "battery_pct", 0.05, 100.0, what)
        bat = bat / 100.0 if bat > 1.0 else bat  # accept 80 as well as 0.8
        if not 0.2 <= bat <= 1.0:
            raise PlanError(f"{what}: battery must be between 20% and 100%")
        out.append({"id": f"D{i + 1}", "bearing_deg": _f(x, "bearing_deg", 0, 360, what) % 360,
                    "distance_m": min(_f(x, "distance_m", 150, 6000, what), max(far, 150.0)),
                    "alt": _f(x, "alt", 30, 110, what), "battery_pct": round(bat, 2),
                    "sensor_radius": _f(x, "sensor_radius", 20, 200, what),
                    "note": str(x.get("note", ""))[:200]})  # fmt: skip
    if sum(x["battery_pct"] >= 0.45 for x in out) < 2:
        raise PlanError("at least two drones need 45% battery or more, or nobody can complete the task")
    if len({x["battery_pct"] for x in out}) < min(3, len(out)) or len({x["alt"] for x in out}) < 2:
        raise PlanError("the drones must differ from each other (battery, altitude, sensor radius, distance)")
    return {"drones": out}


# ---------------------------------------------------------------------------- request building


def _no_fly_candidates(tx: float, ty: float, drones_xy: list[tuple[float, float]], w: float, h: float) -> list[list]:
    """Rectangles 260 m from the target in 8 directions, those lying toward a drone first."""
    cands = []
    for k in range(8):
        ang = math.radians(45 * k)
        cx, cy = tx + 260 * math.sin(ang), ty + 260 * math.cos(ang)
        half = 90.0
        x0, x1, y0, y1 = cx - half, cx + half, cy - half, cy + half
        if x0 < EDGE or y0 < EDGE or x1 > w - EDGE or y1 > h - EDGE:
            continue
        if any(x0 - 70 <= x <= x1 + 70 and y0 - 70 <= y <= y1 + 70 for x, y in drones_xy):
            continue
        toward = min(abs((math.degrees(math.atan2(x - tx, y - ty)) - 45 * k + 180) % 360 - 180) for x, y in drones_xy)
        cands.append((toward, [x0, y0, x1, y1]))
    return [r for _, r in sorted(cands)]


def to_request(place: dict, area: dict, fleet: dict, *, validate: bool = True) -> dict[str, Any]:
    """Build the /api/run request. Raises PlanError when the scenario cannot be built."""
    w, h = float(area["width_m"]), float(area["height_m"])
    fr = GeoFrame(place["lat"], place["lon"], w, h)
    tx, ty = w / 2 + area["target_dx_m"], h / 2 + area["target_dy_m"]
    pts: list[tuple[float, float]] = []
    for d in fleet["drones"]:
        b, r = math.radians(d["bearing_deg"]), d["distance_m"]
        x = min(max(tx + r * math.sin(b), EDGE), w - EDGE)
        y = min(max(ty + r * math.cos(b), EDGE), h - EDGE)
        for _ in range(40):  # nudge apart drones that would start on top of each other
            if all(math.hypot(x - px, y - py) >= MIN_DRONE_GAP for px, py in pts):
                break
            b += math.radians(12)
            x = min(max(tx + r * math.sin(b), EDGE), w - EDGE)
            y = min(max(ty + r * math.cos(b), EDGE), h - EDGE)
        pts.append((x, y))
    south, west = fr.to_latlon(0, 0)
    north, east = fr.to_latlon(w, h)
    tl = fr.to_latlon(tx, ty)

    def req(zone: list | None) -> dict[str, Any]:
        r: dict[str, Any] = {
            "bounds": {"south": round(south, 7), "west": round(west, 7), "north": round(north, 7), "east": round(east, 7)},
            "target": {"lat": round(tl[0], 7), "lon": round(tl[1], 7)},
            "zones": [] if zone is None else [{"corners": [
                {"lat": round(v, 7), "lon": round(u, 7)} for v, u in (fr.to_latlon(zone[0], zone[1]), fr.to_latlon(zone[2], zone[3]))
            ]}],
            "mode": area["mode"],
            "drones": [], "seed": 7,
        }  # fmt: skip
        for d, (x, y) in zip(fleet["drones"], pts):
            la, lo = fr.to_latlon(x, y)
            r["drones"].append({"id": d["id"], "lat": round(la, 7), "lon": round(lo, 7), "alt": round(d["alt"]),
                                "battery_pct": d["battery_pct"], "sensor_radius": round(d["sensor_radius"])})  # fmt: skip
        return r

    options: list[list | None] = [None]
    if area.get("no_fly"):
        options = [*_no_fly_candidates(tx, ty, pts, w, h), None]
    last: Exception | None = None
    for zone in options:
        cand = req(zone)
        if not validate:
            return cand
        try:
            World(build_config(cand)[0])
            return cand
        except Exception as exc:  # RequestError / ConfigError: try the next zone, then no zone
            last = exc
    raise PlanError(f"the scenario could not be built: {last}")
