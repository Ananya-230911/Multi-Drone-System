"""The real CrewAI agents driven by a scripted fake model (no network, no API key).

Skipped when CrewAI is not installed (it needs Python 3.10 to 3.13).
"""

import json

import pytest

pytest.importorskip("crewai")

from crewai.llms.base_llm import BaseLLM  # noqa: E402

from src.crew import env, service  # noqa: E402

PLACE = {"name": "Sector 50, Noida", "lat": 28.5706, "lon": 77.3272, "kind": "suburb", "width_m": 1100, "height_m": 1000}
AREA = {"width_m": 2400, "height_m": 1800, "target_dx_m": 120, "target_dy_m": -80, "mode": "neutralize", "no_fly": True,
        "reason": "Covers the sector with room"}  # fmt: skip
FLEET = {"drones": [
    {"bearing_deg": 20, "distance_m": 400, "alt": 60, "battery_pct": 90, "sensor_radius": 100, "note": "a"},
    {"bearing_deg": 110, "distance_m": 600, "alt": 80, "battery_pct": 55, "sensor_radius": 70, "note": "b"},
    {"bearing_deg": 200, "distance_m": 300, "alt": 45, "battery_pct": 35, "sensor_radius": 120, "note": "c"},
    {"bearing_deg": 290, "distance_m": 500, "alt": 95, "battery_pct": 75, "sensor_radius": 90, "note": "d"},
]}  # fmt: skip


class ScriptedLLM(BaseLLM):
    """Answers by reading which agent is asking; speaks CrewAI's text (ReAct) protocol."""

    llm_type: str = "scripted"
    calls: list = []
    bad_first: bool = False

    def call(self, messages, tools=None, callbacks=None, available_functions=None, from_task=None, from_agent=None,
             response_model=None):
        text = messages if isinstance(messages, str) else "\n".join(str(m.get("content", "")) for m in messages)
        self.calls.append(text)
        role = from_agent.role if from_agent is not None else ""
        if role == "Geo Scout":
            if "\"width_m\": 1100" not in text:  # the tool result has not come back yet
                return ('Thought: I need to search.\nAction: search_place\nAction Input: {"query": "Sector 50 Noida"}')
            return "Thought: done\nFinal Answer: " + json.dumps({"name": PLACE["name"], "lat": PLACE["lat"], "lon": PLACE["lon"],
                                                                 "reason": "Only match"})  # fmt: skip
        if role == "Mission Planner":
            return "Thought: done\nFinal Answer: " + json.dumps(AREA)
        if role == "Fleet Engineer":
            bad = {"drones": [dict(FLEET["drones"][0])] * 4}  # identical drones: guardrail must reject
            return "Thought: done\nFinal Answer: " + json.dumps(bad if self.bad_first and "identical" not in text.lower() and "differ" not in text else FLEET)
        return "Thought: done\nFinal Answer: The winner had the lowest cost."

    def supports_function_calling(self) -> bool:
        return False

    def get_context_window_size(self) -> int:
        return 8000


SETTINGS = env.Settings("key", "m", "u")


def test_crew_plans_a_runnable_scenario():
    llm = ScriptedLLM(model="scripted")
    out = service.plan_from_text("Send drones to Sector 50 Noida", SETTINGS, lambda q: [PLACE], lambda s: llm)
    assert out["engine"] == "crew", out["warnings"]
    assert [a["engine"] for a in out["agents"]] == ["crew"] * 3
    r = out["request"]
    assert len(r["drones"]) == 4 and sorted(d["battery_pct"] for d in r["drones"]) == [0.35, 0.55, 0.75, 0.9]
    from src.webmap.runner import run

    res = run(r)
    assert res["summary"]["neutralized"] and res["allocations"][0]["winner"] in {"D1", "D2", "D3", "D4"}


def test_guardrail_makes_the_agent_fix_its_answer():
    llm = ScriptedLLM(model="scripted", bad_first=True)
    out = service.plan_from_text("Send drones to Sector 50 Noida", SETTINGS, lambda q: [PLACE], lambda s: llm)
    assert out["engine"] == "crew", out["warnings"]
    assert any("differ" in c for c in llm.calls)  # the retry carried the guardrail's complaint


def test_debrief_by_crew():
    out = service.debrief({"allocations": [], "summary": {}}, SETTINGS, lambda s: ScriptedLLM(model="scripted"))
    assert out == {"text": "The winner had the lowest cost.", "engine": "crew"}


def test_scout_cannot_invent_coordinates():
    class Inventor(ScriptedLLM):
        def call(self, messages, *a, **k):
            from_agent = k.get("from_agent")
            if from_agent is not None and from_agent.role == "Geo Scout":
                return "Thought: done\nFinal Answer: " + json.dumps({"name": "Moon", "lat": 1.0, "lon": 1.0})
            return super().call(messages, *a, **k)

    out = service.plan_from_text("Send drones to Sector 50 Noida", SETTINGS, lambda q: [PLACE], lambda s: Inventor(model="scripted"))
    assert out["agents"][0]["engine"] == "offline" and out["request"]["target"]["lat"] == pytest.approx(28.57, abs=0.02)
    assert any("Geo Scout" in w for w in out["warnings"])
