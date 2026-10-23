"""Glue: one sentence in, a ready-to-run scenario request (plus what each agent decided) out."""

from __future__ import annotations

import logging
from typing import Any, Callable

from src.crew import agents as ag  # importing is cheap: CrewAI itself is only loaded when an agent runs
from src.crew import env, geocode, offline, plan

log = logging.getLogger("crew")


class CrewError(RuntimeError):
    """Something the user must fix (no such place, empty sentence). The message is shown as is."""


def _short(exc: Exception) -> str:
    return f"{type(exc).__name__}: {str(exc)[:160]}"


def plan_from_text(text: str, settings: env.Settings | None = None, geocoder: Callable | None = None,
                   llm_factory: Callable | None = None) -> dict[str, Any]:
    text = " ".join(str(text).split())
    if not 3 <= len(text) <= 300:
        raise CrewError("write one sentence, for example: Send drones to Sector 50 Noida")
    settings = settings or env.load()
    geocoder = geocoder or geocode.search
    use_crew = settings.crew_ready or llm_factory is not None
    llm = None
    warnings: list[str] = []
    if use_crew:
        try:
            llm = (llm_factory or ag.make_llm)(settings)
        except Exception as exc:  # e.g. CrewAI import problem
            warnings.append(f"AI crew could not start ({_short(exc)}); used the offline planner")
            use_crew = False
    elif not settings.crew_ready:
        warnings.append(env.status(settings)["message"])
    agents: list[dict] = []

    def step(name: str, role: str, fn_crew: Callable | None, fn_off: Callable):
        if use_crew and fn_crew is not None:
            try:
                v = fn_crew()
                agents.append({"name": name, "role": role, "engine": "crew", "says": v.get("reason", "")})
                return v
            except Exception as exc:
                log.warning("%s failed: %s", name, exc)
                warnings.append(f"{name}: AI answer not usable ({_short(exc)}); offline rule used instead")
        v = fn_off()
        agents.append({"name": name, "role": role, "engine": "offline", "says": v.get("reason", "")})
        return v

    # 1. Geo Scout --------------------------------------------------------------------------
    coords = geocode.parse_coords(text)
    query = offline.query_of(text)
    seen: list[dict] = []
    if coords:
        place = {"name": f"{coords[0]:.5f}, {coords[1]:.5f}", "lat": coords[0], "lon": coords[1], "width_m": 0, "height_m": 0,
                 "reason": "Coordinates were written in the request, so no search was needed."}  # fmt: skip
        agents.append({"name": "Geo Scout", "role": "Finds the location", "engine": "offline", "says": place["reason"]})
    else:
        def first() -> dict:
            try:
                cands = geocoder(query)
            except geocode.GeocodeError as exc:
                raise CrewError(str(exc)) from exc
            seen.extend(cands)
            c = cands[0]
            return {**c, "reason": f"Best match on OpenStreetMap for '{query}': {c['name']}."}

        place = step("Geo Scout", "Finds the location", (lambda: ag.run_scout(llm, query, geocoder, seen)) if use_crew else None,
                     first)  # fmt: skip

    rng = offline.rng_for(text, place)
    # 2. Mission Planner -----------------------------------------------------------------------
    area = step("Mission Planner", "Sizes the area, places the target",
                (lambda: ag.run_planner(llm, text, place)) if use_crew else None,
                lambda: plan.check_area(offline.plan_area(text, place, rng)))  # fmt: skip
    # the operator marks the real target on the map; the drones are laid out around the area centre
    area = {**area, "target_dx_m": 0, "target_dy_m": 0}
    # 3. Fleet Engineer ---------------------------------------------------------------------------
    count = offline.wanted_count(text)
    fleet = step("Fleet Engineer", "Chooses drone positions and specs",
                 (lambda: ag.run_fleet(llm, text, place, area, count)) if use_crew else None,
                 lambda: plan.check_fleet(offline.plan_fleet(text, area, rng), area))  # fmt: skip
    for a in agents:
        if a["name"] == "Fleet Engineer":
            a["says"] = f"{len(fleet['drones'])} drones, each with its own battery, altitude and sensor."

    try:
        req = plan.to_request(place, area, fleet)
    except plan.PlanError as exc:
        if not use_crew:
            raise CrewError(str(exc)) from exc
        warnings.append(f"AI scenario was not buildable ({exc}); offline rules used")
        area = plan.check_area(offline.plan_area(text, place, rng))
        fleet = plan.check_fleet(offline.plan_fleet(text, area, rng), area)
        try:
            req = plan.to_request(place, area, fleet)
        except plan.PlanError as exc2:
            raise CrewError(str(exc2)) from exc2
    engines = {a["engine"] for a in agents}
    return {"request": req, "place": place["name"], "agents": agents, "warnings": warnings,
            "engine": engines.pop() if len(engines) == 1 else "mixed",
            "area_m": [area["width_m"], area["height_m"]], "mode": area["mode"],
            "drone_notes": {d["id"]: d["note"] for d in fleet["drones"]}}  # fmt: skip


# ------------------------------------------------------------------------------------ debrief


def digest(res: dict) -> str:
    """Compact facts about one run, taken from the runner's result (never the raw frames)."""
    lines = []
    allocs = res.get("allocations") or []
    for r in allocs[:3]:
        lines.append(f"Auction round {r['epoch']} winner: {r['winner']}")
        for b in r["bids"]:
            if b["feasible"]:
                left = 100 * b["batt_after"] / b["battery_wh"] if b.get("batt_after") is not None and b.get("battery_wh") else None
                lines.append(f"  {b['drone']}: cost {b['cost']:.2f}, travel {b['t_go']:.0f}s"
                             + (f", battery left on arrival ~{left:.0f}%" if left is not None else ""))
            else:
                lines.append(f"  {b['drone']}: ruled out ({b['reason']})")
    s = res.get("summary") or {}
    for row in s.get("target_log", [])[:2]:
        lines.append(f"Target log: final state {row.get('final_state')}, owner {row.get('owner')}")
    if s.get("neutralized"):
        lines.append("The simulated target was neutralized.")
    lines.append(f"Matches centralized optimum: {s.get('baseline_match')}; geofence violations: {s.get('geofence_violations')}")
    return "\n".join(lines)


def offline_debrief(res: dict) -> str:
    allocs = [r for r in (res.get("allocations") or []) if r.get("winner")]
    if not allocs:
        return "No drone won the auction in this run, so nothing was assigned."
    r = allocs[0]
    ok = sorted((b for b in r["bids"] if b["feasible"] and b.get("cost") is not None), key=lambda b: b["cost"])
    out = [f"{r['winner']} won the bidding: every drone computed its own cost from travel time, energy and remaining battery, "
           "and the lowest cost took the task."]
    if len(ok) > 1:
        a, b = ok[0], ok[1]
        out.append(f"{a['drone']} scored {a['cost']:.2f} against {b['drone']}'s {b['cost']:.2f} (next best).")
    for b in r["bids"]:
        if not b["feasible"]:
            out.append(f"{b['drone']} was ruled out: {b['reason']}.")
    s = res.get("summary") or {}
    if s.get("neutralized"):
        out.append("The winner reached the target, locked on, logged it and the simulated target was neutralized.")
    if s.get("baseline_match") and all(s["baseline_match"]):
        out.append("A centralized check (used only for evaluation) picked the same drone.")
    return " ".join(out)


def debrief(res: dict, settings: env.Settings | None = None, llm_factory: Callable | None = None) -> dict:
    settings = settings or env.load()
    if settings.crew_ready or llm_factory is not None:
        try:
            llm = (llm_factory or ag.make_llm)(settings)
            return {"text": ag.run_debrief(llm, digest(res)), "engine": "crew"}
        except Exception as exc:
            log.warning("debrief failed: %s", exc)
    return {"text": offline_debrief(res), "engine": "offline"}
