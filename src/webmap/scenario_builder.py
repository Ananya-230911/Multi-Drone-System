"""Turn a web-map request into a validated simulation `Config`.

Request (JSON) fields:
    bounds      {south, west, north, east}   the operating area (geofence) as a rectangle, degrees;
                each side 800 m to 12 km
    center, size_m   alternative to `bounds`: a square of size_m metres (800 to 12000) around `center`
    target      {lat, lon}   where the target is
    zones       [{id?, corners: [{lat, lon}, {lat, lon}]}]   optional no-fly rectangles
    drones      [{id, lat, lon, alt, battery_pct, sensor_radius, fail_at?}]   2 to 6 drones
                (fail_at: simulated failure time in seconds, to show the others taking over)
    mode        "neutralize" (simulated engage) or "inspect"
    seed, duration

The drones are told where the target is (an external cue at t = 5 s, so the location is
coordinates only, never evidence). The owner flies there, confirms it with its own sensor,
locks on, logs it and, in "neutralize" mode, the simulated target is marked NEUTRALIZED.
Everything is simulation: nothing here talks to a real vehicle.
"""

from __future__ import annotations

from typing import Any

from src.config import Config, ConfigError, config_from_dict
from src.webmap.geo import GeoFrame

MIN_SIDE, MAX_SIDE = 800.0, 12_000.0  # metres, each side of the operating area
CRUISE_SPEED = 15.0  # m/s, only used to size the default simulation length
EDGE_CLEARANCE = 120.0  # metres from the area edge: boundary inset + margin + patrol loop
PATROL_HALF = 40.0  # half side of each drone's small holding loop, metres
MAX_DRONES = 6


class RequestError(ValueError):
    """The request is not usable. The message is shown to the user as is."""


def _num(obj: dict, key: str, lo: float, hi: float, what: str) -> float:
    try:
        v = float(obj[key])
    except (KeyError, TypeError, ValueError):
        raise RequestError(f"{what}: '{key}' is missing or not a number") from None
    if not lo <= v <= hi:
        if key == "battery_pct":  # the page works in percent
            raise RequestError(f"{what}: battery must be between {lo * 100:g}% and {hi * 100:g}%, got {v * 100:g}%")
        raise RequestError(f"{what}: {key.replace('_', ' ')} must be between {lo:g} and {hi:g}, got {v:g}")
    return v


def _point(obj: Any, what: str) -> tuple[float, float]:
    if not isinstance(obj, dict):
        raise RequestError(f"{what} is missing")
    return _num(obj, "lat", -85.0, 85.0, what), _num(obj, "lon", -180.0, 180.0, what)


def frame_of(req: dict) -> GeoFrame:
    b = req.get("bounds")
    if b is not None:
        if not isinstance(b, dict):
            raise RequestError("operating area bounds are malformed")
        s, n = _num(b, "south", -85.0, 85.0, "operating area"), _num(b, "north", -85.0, 85.0, "operating area")
        w, e = _num(b, "west", -180.0, 180.0, "operating area"), _num(b, "east", -180.0, 180.0, "operating area")
        if not (s < n and w < e):
            raise RequestError("operating area: south must be below north and west left of east")
        fr = GeoFrame.from_bounds(s, w, n, e)
    else:
        lat, lon = _point(req.get("center"), "operating area centre")
        fr = GeoFrame(lat, lon, _num(req, "size_m", MIN_SIDE, MAX_SIDE, "operating area"))
    for side, name in ((fr.width, "width"), (fr.h, "height")):
        if not MIN_SIDE <= side <= MAX_SIDE:
            raise RequestError(f"operating area {name} is {side / 1000:.1f} km; it must be between "
                               f"{MIN_SIDE / 1000:.1f} and {MAX_SIDE / 1000:.0f} km")  # fmt: skip
    return fr


def build_dict(req: dict) -> tuple[dict[str, Any], GeoFrame]:
    if not isinstance(req, dict):
        raise RequestError("request must be a JSON object")
    fr = frame_of(req)
    width, height = fr.width, fr.h
    inside = lambda x, y: (EDGE_CLEARANCE <= x <= width - EDGE_CLEARANCE  # noqa: E731
                           and EDGE_CLEARANCE <= y <= height - EDGE_CLEARANCE)  # fmt: skip

    tx, ty = fr.to_xy(*_point(req.get("target"), "target"))
    if not (20.0 <= tx <= width - 20.0 and 20.0 <= ty <= height - 20.0):
        raise RequestError("the target is outside the operating area (or too close to its edge)")

    drones_in = req.get("drones")
    if not isinstance(drones_in, list) or not 2 <= len(drones_in) <= MAX_DRONES:
        raise RequestError(f"between 2 and {MAX_DRONES} drones are needed")
    drones = []
    fail_events: list[dict] = []
    for i, d in enumerate(drones_in):
        did = str(d.get("id", f"D{i + 1}")) if isinstance(d, dict) else f"D{i + 1}"
        if not isinstance(d, dict):
            raise RequestError(f"drone {did} is malformed")
        x, y = fr.to_xy(*_point(d, f"drone {did}"))
        if not inside(x, y):
            raise RequestError(f"drone {did} is too close to the edge of the operating area (keep it at least "
                               f"{EDGE_CLEARANCE:.0f} m inside the rectangle)")  # fmt: skip
        alt = _num(d, "alt", 30.0, 110.0, f"drone {did}")
        bat = _num(d, "battery_pct", 0.05, 1.0, f"drone {did}")
        rad = _num(d, "sensor_radius", 20.0, 200.0, f"drone {did}")
        if d.get("fail_at") not in (None, ""):
            fail_events.append({"t": _num(d, "fail_at", 1.0, 600.0, f"drone {did}"), "type": "fail_drone", "drone": did})
        h = PATROL_HALF
        drones.append({
            "id": did, "pos": [round(x, 2), round(y, 2)], "alt": alt, "battery_pct": bat,
            "home": [round(x, 2), round(y, 2), 0],
            "patrol": [[round(x + dx, 2), round(y + dy, 2), alt] for dx, dy in ((-h, -h), (h, -h), (h, h), (-h, h))],
            "sensor": {"radius": rad},
        })  # fmt: skip
    if len({d["id"] for d in drones}) != len(drones):
        raise RequestError("drone IDs must be unique")

    zones = []
    for i, z in enumerate(req.get("zones") or []):
        corners = z.get("corners") if isinstance(z, dict) else None
        if not isinstance(corners, list) or len(corners) != 2:
            raise RequestError("a no-fly zone needs two opposite corners")
        (ax, ay), (bx, by) = (fr.to_xy(*_point(c, "no-fly zone corner")) for c in corners)
        x0, x1, y0, y1 = min(ax, bx), max(ax, bx), min(ay, by), max(ay, by)
        if x1 - x0 < 30 or y1 - y0 < 30:
            raise RequestError("a no-fly zone must be at least 30 m wide and 30 m tall")
        zones.append({"id": str(z.get("id") or f"NFZ-{i + 1}"),
                      "polygon": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]})  # fmt: skip

    mode = req.get("mode", "neutralize")
    if mode not in ("neutralize", "inspect"):
        raise RequestError("mode must be 'neutralize' or 'inspect'")
    diag = (width ** 2 + height ** 2) ** 0.5
    # Long enough for the farthest drone to fly there, lock on and fly home, with a margin.
    auto = min(2400.0, max(300.0, 2.5 * diag / CRUISE_SPEED + 90.0))
    duration = _num(req, "duration", 30.0, 3600.0, "simulation") if "duration" in req else auto
    comm = max(1500.0, diag * 1.1)  # the radio covers the whole area
    seed = int(_num(req, "seed", 0, 2**31 - 1, "simulation")) if "seed" in req else 7

    cfg = {
        "sim": {"duration": duration, "seed": seed, "map_size": [width, height]},
        "mission": {"neutralize_on_complete": mode == "neutralize"},
        "geofence": {
            "boundary": [[20, 20], [width - 20, 20], [width - 20, height - 20], [20, height - 20]],
            "margin": 15, "altitude_limits": [15, 120], "restricted_zones": zones,
        },  # fmt: skip
        "network": {"comm_range": comm},
        "association": {"ignore_after_complete": 400},
        "drone_defaults": {"comm_range": comm},
        "drones": drones,
        "targets": [{"id": "TARGET", "pos": [round(tx, 2), round(ty, 2)], "alt": 0, "appear_at": 0}],
        "scenario_events": [{"t": 5, "type": "external_cue", "pos": [round(tx, 2), round(ty, 2)]}] + fail_events,
    }
    return cfg, fr


def build_config(req: dict) -> tuple[Config, GeoFrame]:
    """Config and geo frame, or RequestError with a message fit for the user.

    Geometry checks (drone inside a no-fly zone, patrol loop crossing one, ...) run when the
    World is created; `runner.run` turns those ConfigErrors into RequestErrors too.
    """
    cfg_dict, fr = build_dict(req)
    try:
        return config_from_dict(cfg_dict, name="web_map"), fr
    except ConfigError as exc:
        raise RequestError(f"the scenario is not valid: {exc}") from exc
