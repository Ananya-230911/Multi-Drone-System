"""Offline planner: the same three decisions the crew makes, from rules and a seeded random generator.

Used when there is no Groq key / CrewAI, or when the crew's answer for one part fails validation.
The seed comes from the place and the sentence, so every place gets its own area, drone layout
and drone specs, while the same sentence always gives the same scenario.
"""

from __future__ import annotations

import hashlib
import random
import re

from src.crew.plan import MIN_TARGET_EDGE, SIDE_MAX, SIDE_MIN

INSPECT_WORDS = re.compile(r"\b(inspect|observe|survey|monitor|watch|check|scout|recon\w*)\b", re.I)
AVOID_WORDS = re.compile(r"no[- ]?fly|restricted|avoid|forbidden|exclusion", re.I)
COUNT = re.compile(r"\b([3-6]|three|four|five|six)\s+(?:\w+\s+)?drones?\b", re.I)
WORDS = {"three": 3, "four": 4, "five": 5, "six": 6}


def rng_for(text: str, place: dict) -> random.Random:
    h = hashlib.sha256(f"{' '.join(text.lower().split())}|{place['lat']:.4f}|{place['lon']:.4f}".encode()).digest()
    return random.Random(int.from_bytes(h[:8], "big"))


def query_of(text: str) -> str:
    """The place part of a sentence like 'Send 4 drones to Sector 50 Noida and destroy the target'."""
    q = re.sub(r"\b(send|deploy|dispatch|launch)\b|\b(\w+\s+)?drones?\b|\bthe\b|\band (inspect|destroy|neutrali[sz]e)\b.*$", " ",
               text, flags=re.I)
    q = re.sub(r"\b(to|at|over|near|towards?|in|into)\b(?=\s|$)", " ", q, count=1, flags=re.I)
    q = re.sub(r"[.!?]+$", "", q)
    return " ".join(q.split()) or text.strip()


def wanted_count(text: str) -> int | None:
    m = COUNT.search(text)
    if not m:
        return None
    v = m.group(1).lower()
    return WORDS.get(v) or int(v)


def plan_area(text: str, place: dict, rng: random.Random) -> dict:
    bw, bh = place.get("width_m", 0), place.get("height_m", 0)
    base = max(bw, bh)
    # the place's own outline gives the scale; a point-like result gets a neighbourhood-sized area
    size = min(max(base * 1.3, 1500.0), 6000.0) if base >= 300 else rng.uniform(1500, 2600)
    w = min(max(size * rng.uniform(0.95, 1.3), SIDE_MIN), SIDE_MAX)
    h = min(max(size * rng.uniform(0.8, 1.05), SIDE_MIN), SIDE_MAX)
    mx, my = w / 2 - MIN_TARGET_EDGE, h / 2 - MIN_TARGET_EDGE
    dx, dy = rng.uniform(-0.6, 0.6) * mx, rng.uniform(-0.6, 0.6) * my
    mode = "inspect" if INSPECT_WORDS.search(text) and not re.search(r"destroy|neutrali|engage|eliminate", text, re.I) else "neutralize"
    no_fly = bool(AVOID_WORDS.search(text)) or rng.random() < 0.4
    return {"width_m": round(w), "height_m": round(h), "target_dx_m": round(dx), "target_dy_m": round(dy), "mode": mode,
            "no_fly": no_fly,
            "reason": f"Area sized from the {max(bw, bh) / 1000:.1f} km outline of the place on the map, target placed "
                      f"{abs(dx):.0f} m E/W and {abs(dy):.0f} m N/S of the centre."
                      + (" A small no-fly block is added next to the target." if no_fly else "")}  # fmt: skip


def plan_fleet(text: str, area: dict, rng: random.Random) -> dict:
    n = wanted_count(text) or rng.choice([4, 5])
    lo = 160.0
    hi = max(lo + 60, 0.47 * min(area["width_m"], area["height_m"]) - 150)
    start = rng.uniform(0, 360)
    drones = []
    used_bat: set[float] = set()
    for i in range(n):
        bat = round(rng.uniform(0.4, 1.0), 2)
        while bat in used_bat:
            bat = round(bat + 0.01, 2)
        used_bat.add(bat)
        drones.append({
            "bearing_deg": (start + i * 360 / n + rng.uniform(-25, 25)) % 360,
            "distance_m": round(rng.uniform(lo, hi)),
            "alt": float(rng.randrange(40, 105, 5)),
            "battery_pct": bat,
            "sensor_radius": float(rng.randrange(60, 160, 10)),
            "note": "",
        })  # fmt: skip
    if sum(d["battery_pct"] >= 0.45 for d in drones) < 2:
        drones[0]["battery_pct"], drones[1]["battery_pct"] = 0.9, 0.7
    if len({d["alt"] for d in drones}) < 2:
        drones[0]["alt"] = 45.0
        drones[1]["alt"] = 90.0
    for d in drones:
        d["note"] = (f"{d['distance_m']:.0f} m from the target, flies at {d['alt']:.0f} m, "
                     f"{d['battery_pct'] * 100:.0f}% battery, sees {d['sensor_radius']:.0f} m")  # fmt: skip
    return {"drones": drones}
