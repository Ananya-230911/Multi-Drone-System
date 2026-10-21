import copy
import math

import pytest

from src.webmap.geo import GeoFrame
from src.webmap.scenario_builder import RequestError, build_config, build_dict

FR = GeoFrame(48.8566, 2.3522, 2000.0)


def ll(dx, dy):
    lat, lon = FR.to_latlon(1000 + dx, 1000 + dy)
    return {"lat": lat, "lon": lon}


def request(**over):
    req = {
        "center": {"lat": FR.lat0, "lon": FR.lon0}, "size_m": 2000, "target": ll(100, 50), "mode": "neutralize",
        "drones": [dict(id="D1", alt=60, battery_pct=0.85, sensor_radius=80, **ll(-550, 450)),
                   dict(id="D2", alt=90, battery_pct=0.7, sensor_radius=80, **ll(500, 350)),
                   dict(id="D3", alt=60, battery_pct=0.3, sensor_radius=80, **ll(-300, -500)),
                   dict(id="D4", alt=60, battery_pct=0.95, sensor_radius=40, **ll(450, -450))],
    }  # fmt: skip
    req.update(over)
    return req


def test_geo_round_trip_and_scale():
    for x, y in [(0, 0), (1000, 1000), (1999, 17)]:
        assert FR.to_xy(*FR.to_latlon(x, y)) == pytest.approx((x, y), abs=1e-6)
    la0, lo0 = FR.to_latlon(1000, 1000)
    assert (la0, lo0) == pytest.approx((FR.lat0, FR.lon0))
    la1, lo1 = FR.to_latlon(1100, 1000)  # 100 m east
    assert math.dist(FR.to_xy(la1, lo1), (1100, 1000)) < 1e-6
    assert (lo1 - lo0) == pytest.approx(100 / (111320 * math.cos(math.radians(FR.lat0))))
    with pytest.raises(ValueError):
        GeoFrame(89.0, 0.0, 1000)


def test_builds_valid_config_with_cue_and_ground_truth():
    cfg, fr = build_config(request())
    assert cfg.mission.neutralize_on_complete and [d.id for d in cfg.drones] == ["D1", "D2", "D3", "D4"]
    assert cfg.scenario_events[0].type == "external_cue" and cfg.scenario_events[0].t == 5.0
    cue, gt = cfg.scenario_events[0].data["pos"], cfg.targets[0].pos
    assert tuple(cue) == pytest.approx(gt)
    assert gt == pytest.approx(fr.to_xy(*[request()["target"][k] for k in ("lat", "lon")]), abs=0.01)
    d4 = next(d for d in cfg.drones if d.id == "D4")
    assert d4.params.sensor.radius == 40 and d4.battery_pct == 0.95
    assert d4.home[2] == 0 and d4.patrol[0][2] == 60


def test_inspect_mode_and_fail_event():
    req = request(mode="inspect")
    req["drones"][1]["fail_at"] = 25
    cfg, _ = build_config(req)
    assert not cfg.mission.neutralize_on_complete
    assert [(e.type, e.t) for e in cfg.scenario_events] == [("external_cue", 5.0), ("fail_drone", 25.0)]


def test_zone_from_two_corners():
    d, _ = build_dict(request(zones=[{"corners": [ll(-50, -100), ll(60, 200)]}]))
    poly = d["geofence"]["restricted_zones"][0]["polygon"]
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    assert (max(xs) - min(xs), max(ys) - min(ys)) == pytest.approx((110, 300), abs=0.5)


@pytest.mark.parametrize("mutate, text", [
    (lambda r: r.pop("target"), "target is missing"),
    (lambda r: r.update(target=ll(1500, 0)), "target is outside"),
    (lambda r: r.update(size_m=50), "operating area"),
    (lambda r: r.update(mode="bomb"), "mode must be"),
    (lambda r: r["drones"].__setitem__(0, dict(r["drones"][0], **ll(-980, 0))), "too close to the edge"),
    (lambda r: r["drones"][0].update(battery_pct=5), "battery must be between 5% and 100%"),
    (lambda r: r["drones"][0].update(alt=500), "alt must be between"),
    (lambda r: r["drones"][1].update(id="D1"), "unique"),
    (lambda r: r.update(drones=r["drones"][:1]), "between 2 and"),
    (lambda r: r.update(zones=[{"corners": [ll(0, 0), ll(5, 5)]}]), "at least 30 m"),
    (lambda r: r.update(zones=[{"corners": [ll(0, 0)]}]), "two opposite corners"),
    (lambda r: r.update(center={"lat": 95, "lon": 0}), "lat must be between"),
])  # fmt: skip
def test_bad_requests_get_clear_messages(mutate, text):
    req = copy.deepcopy(request())
    mutate(req)
    with pytest.raises(RequestError, match=text):
        build_config(req)


# ------------------------------------------------------------------ rectangles and bounds


def test_geoframe_rectangle_round_trip():
    fr = GeoFrame(28.5705, 77.36, 5000.0, 3000.0)
    assert (fr.width, fr.h) == (5000.0, 3000.0)
    for x, y in [(0, 0), (2500, 1500), (4999, 2999), (123, 2750)]:
        assert fr.to_xy(*fr.to_latlon(x, y)) == pytest.approx((x, y), abs=1e-6)
    assert fr.to_latlon(2500, 1500) == pytest.approx((28.5705, 77.36))
    assert GeoFrame(1.0, 2.0, 700.0).h == 700.0  # height defaults to the width


def test_geoframe_from_bounds_matches_real_distances():
    fr = GeoFrame.from_bounds(28.55, 77.33, 28.58, 77.39)
    assert fr.h == pytest.approx(0.03 * 111320)
    assert fr.width == pytest.approx(0.06 * 111320 * math.cos(math.radians(28.565)))
    assert fr.to_xy(28.55, 77.33) == pytest.approx((0.0, 0.0), abs=1e-6)  # south-west corner is the origin
    assert fr.to_xy(28.58, 77.39) == pytest.approx((fr.width, fr.h), abs=1e-6)
    with pytest.raises(ValueError):
        GeoFrame.from_bounds(28.58, 77.33, 28.55, 77.39)


def bounds_request(**over):
    fr = GeoFrame.from_bounds(28.55, 77.33, 28.58, 77.39)  # about 5.9 x 3.3 km

    def at(fx, fy):
        la, lo = fr.to_latlon(fx * fr.width, fy * fr.h)
        return {"lat": la, "lon": lo}

    req = {"bounds": {"south": 28.55, "west": 77.33, "north": 28.58, "east": 77.39}, "target": at(0.6, 0.5), "mode": "neutralize",
           "drones": [dict(id=f"D{i + 1}", alt=60, battery_pct=0.9, sensor_radius=80, **at(fx, fy))
                      for i, (fx, fy) in enumerate([(0.1, 0.2), (0.9, 0.2), (0.1, 0.8), (0.9, 0.8)])]}  # fmt: skip
    req.update(over)
    return req, fr, at


def test_bounds_make_a_rectangular_geofence():
    req, fr, _ = bounds_request()
    d, frame = build_dict(req)
    assert (frame.width, frame.h) == pytest.approx((fr.width, fr.h))
    poly = d["geofence"]["boundary"]
    assert poly[2] == pytest.approx([fr.width - 20, fr.h - 20]) and d["sim"]["map_size"] == pytest.approx([fr.width, fr.h])
    assert d["sim"]["duration"] > 300  # a larger area gets a longer default run
    cfg, _ = build_config(req)
    assert cfg.geofence.boundary[1][0] == pytest.approx(fr.width - 20)


@pytest.mark.parametrize("south, north, text", [
    (28.5500, 28.5505, "height is 0.1 km"),   # far too small
    (28.5000, 28.7000, "height is 22.3 km"),  # far too big
])  # fmt: skip
def test_area_size_limits_are_enforced(south, north, text):
    req, _, _ = bounds_request()
    req["bounds"].update(south=south, north=north)
    with pytest.raises(RequestError, match=text):
        build_config(req)


def test_bounds_must_be_ordered():
    req, _, _ = bounds_request()
    req["bounds"].update(south=28.6)
    with pytest.raises(RequestError, match="south must be below north"):
        build_config(req)


def test_target_and_drones_must_be_inside_the_rectangle():
    req, fr, at = bounds_request()
    req["target"] = at(1.2, 0.5)
    with pytest.raises(RequestError, match="target is outside"):
        build_config(req)
    req, fr, at = bounds_request()
    req["drones"][0].update(at(0.001, 0.5))
    with pytest.raises(RequestError, match="too close to the edge"):
        build_config(req)
