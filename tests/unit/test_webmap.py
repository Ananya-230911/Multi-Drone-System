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
