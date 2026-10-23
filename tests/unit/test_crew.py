"""AI crew pieces that need no CrewAI and no network: settings, validators, offline planner, service, HTTP."""

import json
import threading
import urllib.error
import urllib.request

import pytest

from src.crew import env, geocode, offline, plan, service
from src.webmap.server import make_server

PLACE = {"name": "Sector 50, Noida", "lat": 28.5706, "lon": 77.3272, "kind": "suburb", "width_m": 1100, "height_m": 1000}
OFFLINE = env.Settings("", "m", "u")


def fake_geo(q):
    return [dict(PLACE, name=q)]


# ---------------------------------------------------------------------------------- settings


def test_env_reads_dotenv_and_real_environment_wins(tmp_path):
    (tmp_path / ".env").write_text("# c\nGROQ_API_KEY = 'abc'\nexport GROQ_MODEL=m1\n")
    s = env.load(tmp_path, {})
    assert (s.api_key, s.model) == ("abc", "m1")
    assert env.load(tmp_path, {"GROQ_API_KEY": "zzz"}).api_key == "zzz"
    assert env.load(tmp_path / "none", {}).model == env.DEFAULT_MODEL


def test_status_never_contains_the_key():
    st = env.status(env.Settings("sk-secret", "m", "u"))
    assert "sk-secret" not in json.dumps(st) and st["has_key"] is True


# ---------------------------------------------------------------------------------- validators


def test_place_must_come_from_search_results():
    assert plan.check_place({"lat": 28.5706, "lon": 77.3272}, [PLACE])["name"] == PLACE["name"]
    with pytest.raises(plan.PlanError, match="search_place"):
        plan.check_place({"lat": 10, "lon": 10}, [PLACE])
    with pytest.raises(plan.PlanError):
        plan.check_place({"lat": 28.5706, "lon": 77.3272}, [])


@pytest.mark.parametrize("patch,text", [({"width_m": 100}, "between"), ({"target_dx_m": 2000}, "inside"),
                                        ({"mode": "bomb"}, "mode"), ({"no_fly": "yes"}, "no_fly")])
def test_area_rejects_nonsense(patch, text):
    good = {"width_m": 2000, "height_m": 2000, "target_dx_m": 0, "target_dy_m": 0}
    assert plan.check_area(good)["mode"] == "neutralize"
    with pytest.raises(plan.PlanError, match=text):
        plan.check_area({**good, **patch})


def test_fleet_rejects_identical_or_dead_drones_and_accepts_percent():
    area = plan.check_area({"width_m": 2000, "height_m": 2000, "target_dx_m": 0, "target_dy_m": 0})
    d = lambda b, a=60, r=80, dist=300: {"bearing_deg": 0, "distance_m": dist, "alt": a, "battery_pct": b, "sensor_radius": r}  # noqa: E731
    with pytest.raises(plan.PlanError, match="differ"):
        plan.check_fleet({"drones": [d(80)] * 4}, area)
    with pytest.raises(plan.PlanError, match="45%"):
        plan.check_fleet({"drones": [d(30), d(31, 70), d(32, 80), d(33, 90)]}, area)
    with pytest.raises(plan.PlanError, match="3 to 6"):
        plan.check_fleet({"drones": [d(80)]}, area)
    ok = plan.check_fleet({"drones": [d(80), d(0.6, 70), d(50, 80), d(95, 90)]}, area)
    assert sorted(x["battery_pct"] for x in ok["drones"]) == [0.5, 0.6, 0.8, 0.95]


# ---------------------------------------------------------------------------------- offline planner


def test_query_and_count_are_taken_from_the_sentence():
    assert offline.query_of("Send drones to Sector 50 Noida") == "Sector 50 Noida"
    assert offline.query_of("Send 5 drones to Pune and destroy the target") == "Pune"
    assert offline.wanted_count("send four drones to Pune") == 4 and offline.wanted_count("send drones to Pune") is None


def test_each_place_gets_its_own_scenario():
    outs = []
    for lat, lon, text in ((28.57, 77.33, "Send drones to Sector 50 Noida"), (18.52, 73.85, "Send drones to Pune"),
                           (12.82, 80.04, "Send drones to Potheri")):
        s = service.plan_from_text(text, OFFLINE, lambda q, lat=lat, lon=lon: [dict(PLACE, lat=lat, lon=lon)])
        outs.append(s)
    sigs = {(tuple(o["area_m"]), tuple((d["alt"], d["battery_pct"], d["sensor_radius"]) for d in o["request"]["drones"])) for o in outs}
    assert len(sigs) == 3  # not the same defaults every time
    again = service.plan_from_text("Send drones to Pune", OFFLINE, lambda q: [dict(PLACE, lat=18.52, lon=73.85)])
    assert again["request"] == outs[1]["request"]  # same sentence and place, same scenario


def test_offline_service_builds_runnable_request_and_honours_wishes():
    s = service.plan_from_text("Send 5 drones to Sector 50 Noida and inspect it, avoid the no-fly zone", OFFLINE, fake_geo)
    r = s["request"]
    assert s["engine"] == "offline" and len(r["drones"]) == 5 and r["mode"] == "inspect"
    assert r["zones"], "the sentence asks to respect a no-fly zone"
    from src.webmap.runner import run

    res = run(r)
    assert res["summary"]["target_log"][0]["final_state"] == "COMPLETED" and res["summary"]["geofence_violations"] == 0


def test_coordinates_in_the_sentence_skip_the_search():
    def boom(q):
        raise AssertionError("no search expected")

    s = service.plan_from_text("go to 28.6139, 77.2090", OFFLINE, boom)
    assert s["place"].startswith("28.61390") and s["agents"][0]["name"] == "Geo Scout"


def test_user_errors_are_clear():
    with pytest.raises(service.CrewError, match="one sentence"):
        service.plan_from_text("  ", OFFLINE, fake_geo)

    def nothing(q):
        raise geocode.GeocodeError("no place found for 'x'")

    with pytest.raises(service.CrewError, match="no place found"):
        service.plan_from_text("Send drones to xyzzy", OFFLINE, nothing)


def test_offline_debrief_names_winner_and_ruled_out_drones():
    res = {"allocations": [{"epoch": 0, "winner": "D2", "bids": [
        {"drone": "D1", "feasible": True, "cost": 30.0, "t_go": 20, "batt_after": 50, "battery_wh": 100},
        {"drone": "D2", "feasible": True, "cost": 20.0, "t_go": 10, "batt_after": 80, "battery_wh": 100},
        {"drone": "D3", "feasible": False, "reason": "battery below reserve", "cost": None}]}],
        "summary": {"neutralized": [{}], "baseline_match": [True]}}
    t = service.offline_debrief(res)
    assert "D2 won" in t and "D3 was ruled out: battery below reserve" in t and "20.00 against D1's 30.00" in t
    assert "D2" in service.digest(res) and "ruled out" in service.digest(res)


# ---------------------------------------------------------------------------------- HTTP


@pytest.fixture()
def server(monkeypatch):
    monkeypatch.setattr(geocode, "search", fake_geo)
    monkeypatch.setattr(env, "load", lambda *a, **k: OFFLINE)
    srv = make_server(0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()


def post(base, path, obj):
    req = urllib.request.Request(base + path, json.dumps(obj).encode(), {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_http_crew_status_and_debrief(server):
    with urllib.request.urlopen(server + "/api/status") as r:
        st = json.loads(r.read())
    assert st["crew_ready"] is False and "offline" in st["message"]
    code, out = post(server, "/api/crew", {"text": "Send drones to Sector 50 Noida"})
    assert code == 200 and out["request"]["drones"] and out["agents"]
    code, run_res = post(server, "/api/run", out["request"])
    assert code == 200
    code, d = post(server, "/api/debrief", {"allocations": run_res["allocations"], "summary": run_res["summary"]})
    assert code == 200 and d["engine"] == "offline" and "won the bidding" in d["text"]
    code, err = post(server, "/api/crew", {"text": ""})
    assert code == 400 and "one sentence" in err["error"]
