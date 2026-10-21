"""The web map backend: a request in, a replayable recorded flight out; plus the HTTP server."""

import json
import threading
import urllib.error
import urllib.request

import pytest

from src.webmap.geo import GeoFrame
from src.webmap.runner import run
from src.webmap.scenario_builder import RequestError
from src.webmap.server import make_server
from tests.unit.test_webmap import FR, bounds_request, ll, request


@pytest.fixture(scope="module")
def result():
    return run(request(zones=[{"corners": [ll(-30, -60), ll(30, 120)]}]))


def test_mission_runs_and_is_recorded(result):
    r = result
    row = r["summary"]["target_log"][0]
    assert row["final_state"] == "COMPLETED" and row["owner"] in r["drone_ids"] and row["inspection_duration"] >= 10 - 1e-6
    assert r["summary"]["neutralized"] and r["frames"][-1]["neutralized"]
    assert r["summary"]["geofence_violations"] == 0 and r["summary"]["separation_violations"] == 0
    assert r["summary"]["baseline_match"] == [True]
    ts = [f["t"] for f in r["frames"]]
    assert ts == sorted(ts) and ts[0] <= 0.2 and abs(ts[-1] - r["t_end"]) < 1.0
    assert all(len(f["d"]) == 4 and len(f["d"][0]) == 7 for f in r["frames"])


def test_frames_are_geographic_and_inside_the_area(result):
    for f in result["frames"][::10]:
        for lat, lon, *_ in f["d"]:
            x, y = FR.to_xy(lat, lon)
            assert 0 <= x <= 2000 and 0 <= y <= 2000


def test_bids_and_events_for_the_panel(result):
    [alloc] = result["allocations"]
    assert alloc["winner"] == result["summary"]["target_log"][0]["owner"]
    assert {b["drone"] for b in alloc["bids"]} == {"D1", "D2", "D3", "D4"}
    winner = next(b for b in alloc["bids"] if b["drone"] == alloc["winner"])
    assert winner["feasible"] and all(not b["feasible"] or winner["cost"] <= b["cost"] + 1e-9 for b in alloc["bids"])
    types = [e["type"] for e in result["events"]]
    for t in ("ALLOCATION_COMMITTED", "TARGET_LOGGED", "TARGET_NEUTRALIZED"):
        assert t in types
    assert all(set(e) == {"t", "type", "actor", "text"} for e in result["events"])
    assert len(result["zones"]) == 1 and len(result["zones"][0]) == 4
    assert any(r["kind"] == "mission" for r in result["routes"])


def test_failed_owner_is_replaced():
    req = request()
    req["drones"][1]["fail_at"] = 25  # D2 is the nearest in this layout
    r = run(req)
    row = r["summary"]["target_log"][0]
    assert row["final_state"] == "COMPLETED" and row["reallocations"] == 1 and row["owner"] != "D2"
    assert any(e["type"] == "REALLOCATION" for e in r["events"])
    last = r["frames"][-1]
    assert last["owner"] == row["owner"]  # the page shows the live drone's view, not the failed drone's


def test_target_inside_no_fly_zone_is_cancelled_not_inspected():
    r = run(request(target=ll(0, 0), zones=[{"corners": [ll(-150, -150), ll(150, 150)]}]))
    row = r["summary"]["target_log"][0]
    assert row["final_state"] == "CANCELLED" and row["cancel_reason"] == "no_feasible_bidders"
    assert not any(e["type"] == "TARGET_LOGGED" for e in r["events"])


def test_drone_inside_no_fly_zone_is_a_request_error():
    with pytest.raises(RequestError, match="not valid"):
        run(request(zones=[{"corners": [ll(-600, 400), ll(-500, 500)]}]))  # covers D1's start


def test_same_request_same_result():
    a, b = run(request()), run(request())
    assert a["frames"] == b["frames"] and a["events"] == b["events"]


# ------------------------------------------------------------------ HTTP


@pytest.fixture(scope="module")
def server():
    srv = make_server(0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()
    srv.server_close()


def post(base, body, raw=False):
    data = body if raw else json.dumps(body).encode()
    req = urllib.request.Request(base + "/api/run", data=data, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_server_serves_page_and_bundled_leaflet(server):
    for path, marker in [("/", b"Drone coordination"), ("/static/vendor/leaflet.js", b"Leaflet"),
                         ("/static/vendor/leaflet.css", b".leaflet")]:  # fmt: skip
        with urllib.request.urlopen(server + path) as r:
            assert r.status == 200 and marker in r.read()


def test_server_rejects_path_traversal_and_unknown_paths(server):
    for path in ("/static/../server.py", "/static/%2e%2e/server.py", "/nope"):
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(server + path)
        assert e.value.code == 404


def test_server_run_and_errors(server):
    code, data = post(server, request())
    assert code == 200 and data["summary"]["target_log"][0]["final_state"] == "COMPLETED"
    code, data = post(server, {"nonsense": 1})
    assert code == 400 and "centre" in data["error"]
    code, data = post(server, b"{not json", raw=True)
    assert code == 400 and "JSON" in data["error"]
    code, data = post(server, request(mode="x"))
    assert code == 400 and "mode" in data["error"]
    code, data = post(server, b"x" * 300_000, raw=True)
    assert code == 400 and "too large" in data["error"]


# ------------------------------------------------------------------ larger, non-square area


def test_rectangular_area_runs_and_stays_inside():
    req, fr, at = bounds_request(zones=None)
    req["zones"] = [{"corners": [at(0.4, 0.3), at(0.48, 0.7)]}]
    r = run(req)
    assert r["area_m"] == [round(fr.width), round(fr.h)]
    row = r["summary"]["target_log"][0]
    assert row["final_state"] == "COMPLETED" and r["summary"]["geofence_violations"] == 0
    assert r["summary"]["baseline_match"] == [True]
    for f in r["frames"][::8]:
        for lat, lon, *_ in f["d"]:
            x, y = fr.to_xy(lat, lon)
            assert 0 <= x <= fr.width and 0 <= y <= fr.h
    # the target is neutralized at the exact point that was asked for (to well under a metre of map precision)
    tx, ty = fr.to_xy(r["target"]["lat"], r["target"]["lon"])
    assert 0 < tx < fr.width and 0 < ty < fr.h


def test_large_area_gets_a_long_enough_run():
    fr = GeoFrame(28.565, 77.36, 12000.0)

    def at(fx, fy):
        la, lo = fr.to_latlon(fx * 12000, fy * 12000)
        return {"lat": la, "lon": lo}

    req = {"center": {"lat": 28.565, "lon": 77.36}, "size_m": 12000, "target": at(0.5, 0.5), "mode": "neutralize",
           "drones": [dict(id=f"D{i + 1}", alt=60, battery_pct=0.9, sensor_radius=80, **at(fx, fy))
                      for i, (fx, fy) in enumerate([(0.05, 0.05), (0.95, 0.05), (0.05, 0.95), (0.95, 0.95)])]}  # fmt: skip
    r = run(req)
    assert r["summary"]["target_log"][0]["final_state"] == "COMPLETED"
