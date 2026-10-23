"""The CrewAI agents. Imported only when the crew is used (CrewAI needs Python 3.10 to 3.13).

Four single-agent crews run one after another, each with its own guardrail, so a bad answer
from one agent is retried and, failing that, replaced by the offline planner for that part only:

    Geo Scout        searches the map (tool: search_place) and chooses the place
    Mission Planner  chooses area size, target position, no-fly block and mode
    Fleet Engineer   chooses each drone's distance, altitude, battery and sensor radius
    Debrief Officer  explains, after the run, why the drones chose the winner
"""

from __future__ import annotations

import json
import os
import re
from typing import Any, Callable

from src.crew import plan
from src.crew.env import Settings

os.environ.setdefault("CREWAI_DISABLE_TELEMETRY", "true")
os.environ.setdefault("OTEL_SDK_DISABLED", "true")
os.environ.setdefault("CREWAI_TESTING", "false")

RETRIES = 2


def make_llm(s: Settings):
    from crewai import LLM

    # Groq speaks the OpenAI protocol, so CrewAI's built-in OpenAI provider talks to it directly.
    return LLM(model=f"openai/{s.model}", base_url=s.base_url, api_key=s.api_key, temperature=0.7, timeout=60,
               max_tokens=1200, api="completions")  # fmt: skip


def parse_json(text: str) -> Any:
    t = re.sub(r"^```(?:json)?|```$", "", text.strip(), flags=re.M).strip()
    try:
        return json.loads(t)
    except json.JSONDecodeError:
        m = re.search(r"\{.*\}", t, re.S)
        if not m:
            raise plan.PlanError("answer must be a single JSON object and nothing else") from None
        try:
            return json.loads(m.group(0))
        except json.JSONDecodeError as exc:
            raise plan.PlanError(f"answer is not valid JSON ({exc.msg})") from None


def _run(llm, *, role: str, goal: str, backstory: str, description: str, expected: str, tools=None,
         check: Callable[[Any], Any] | None = None) -> Any:
    from crewai import Agent, Crew, Process, Task

    box: dict[str, Any] = {}

    def guard(out):
        try:
            box["v"] = check(parse_json(out.raw))
        except plan.PlanError as exc:
            return False, str(exc)
        return True, json.dumps(box["v"])

    agent = Agent(role=role, goal=goal, backstory=backstory, llm=llm, tools=tools or [], allow_delegation=False,
                  verbose=False, max_iter=6)  # fmt: skip
    kw: dict[str, Any] = {"guardrail": guard, "guardrail_max_retries": RETRIES} if check else {}
    task = Task(description=description, expected_output=expected, agent=agent, **kw)
    result = Crew(agents=[agent], tasks=[task], process=Process.sequential, verbose=False, tracing=False).kickoff()
    return box["v"] if check else str(result.raw if hasattr(result, "raw") else result).strip()


def run_scout(llm, query: str, geocoder, seen: list[dict]) -> dict:
    from crewai.tools import tool

    @tool("search_place")
    def search_place(query: str) -> str:
        """Search OpenStreetMap for a place name. Returns up to 5 candidates with lat, lon and the
        size in metres of the place's outline (width_m, height_m)."""
        try:
            found = geocoder(query)
        except Exception as exc:
            return f"search failed: {exc}"
        seen.extend(c for c in found if c not in seen)
        return json.dumps(found)

    return _run(
        llm, role="Geo Scout",
        goal="Find the exact map location the operator means.",
        backstory="You are a careful GIS analyst. You never invent coordinates: you only use what the search tool returns.",
        description=(f"The operator wrote: \"{query}\".\nUse the search_place tool to find the place (try a refined query if "
                     "the first one gives nothing useful) and choose the best result.\nAnswer with ONLY a JSON object: "
                     '{"name": str, "lat": number, "lon": number, "reason": "one sentence"} where lat and lon are copied '
                     "exactly from one search result."),
        expected="A JSON object with name, lat, lon, reason.", tools=[search_place],
        check=lambda d: plan.check_place(d, seen))  # fmt: skip


def run_planner(llm, text: str, place: dict) -> dict:
    return _run(
        llm, role="Mission Planner",
        goal="Size the operating area and place the target sensibly for this location.",
        backstory="You plan small-area drone inspection simulations. Areas are rectangles between 800 m and 12000 m per side.",
        description=(f"Operator request: \"{text}\".\nPlace: {place['name']} (lat {place['lat']}, lon {place['lon']}); its map "
                     f"outline is about {place['width_m']} m wide and {place['height_m']} m tall.\nChoose: width_m and height_m "
                     "of the operating area (cover the place with some room, typically 1500 to 4000), the target's offset from the "
                     "area centre target_dx_m (east+) and target_dy_m (north+) (keep at least 300 m from every edge), mode "
                     "('neutralize' unless the operator only wants to inspect/observe), and no_fly (true to add a small "
                     "restricted block beside the target to force a detour).\nAnswer with ONLY a JSON object: "
                     '{"width_m": n, "height_m": n, "target_dx_m": n, "target_dy_m": n, "mode": str, "no_fly": bool, '
                     '"reason": "one sentence"}'),
        expected="A JSON object with the area, target offset, mode and no_fly.", check=plan.check_area)  # fmt: skip


def run_fleet(llm, text: str, place: dict, area: dict, count: int | None) -> dict:
    n = f"exactly {count}" if count else "4 or 5"
    return _run(
        llm, role="Fleet Engineer",
        goal="Define a realistic, varied set of drones positioned around the target.",
        backstory="You configure a small fleet of surveillance quadcopters. No two drones are identical: they differ in "
                  "battery, altitude, sensor and distance from the target.",
        description=(f"Operator request: \"{text}\". Operating area {area['width_m']} x {area['height_m']} m around "
                     f"{place['name']}; the target is the reference point.\nDefine {n} drones. For each give: bearing_deg "
                     "(0-360, compass direction from the target to the drone, spread them around), distance_m (150 to "
                     f"{max(200, int(0.47 * min(area['width_m'], area['height_m'])) - 150)}), alt (30-110 m), battery_pct "
                     "(percent 20-100, at least two drones >= 45), sensor_radius (20-200 m) and a short note.\nAnswer with ONLY "
                     'a JSON object: {"drones": [{"bearing_deg": n, "distance_m": n, "alt": n, "battery_pct": n, '
                     '"sensor_radius": n, "note": str}, ...]}'),
        expected="A JSON object with a drones list.", check=lambda d: plan.check_fleet(d, area))  # fmt: skip


def run_debrief(llm, digest: str) -> str:
    return _run(
        llm, role="Debrief Officer",
        goal="Explain to a student why the drones chose this winner, in plain language.",
        backstory="You explain decentralized multi-robot coordination clearly and honestly. You only state facts from the data.",
        description=("Here is the data of one simulated run (the drones decided among themselves with a bid auction: the lowest "
                     f"cost wins; cost mixes travel time, energy and remaining battery):\n{digest}\n\nWrite 4 to 6 short sentences: "
                     "which drone was chosen and why (compare its numbers with the others), whether any drone was ruled out and "
                     "why, and how the run ended. Plain text, no markdown, no invented numbers."),
        expected="4 to 6 plain sentences.")
