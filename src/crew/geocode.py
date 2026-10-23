"""Place search for the crew (OpenStreetMap Nominatim, run on the server side)."""

from __future__ import annotations

import json
import re
import threading
import time
import urllib.parse
import urllib.request

URL = "https://nominatim.openstreetmap.org/search"
UA = "multi-drone-sim/1.0 (university simulation project)"
_cache: dict[str, list[dict]] = {}
_lock = threading.Lock()
_last = 0.0

_COORDS = re.compile(r"(-?\d{1,2}(?:\.\d+)?)\s*[, ]\s*(-?\d{1,3}(?:\.\d+)?)")


class GeocodeError(RuntimeError):
    pass


def parse_coords(text: str) -> tuple[float, float] | None:
    """'28.5706, 77.3272' typed inside the sentence."""
    m = _COORDS.search(text)
    if not m or "." not in m.group(0):
        return None
    lat, lon = float(m.group(1)), float(m.group(2))
    return (lat, lon) if -85 <= lat <= 85 and -180 <= lon <= 180 else None


def search(query: str, limit: int = 5) -> list[dict]:
    """[{name, lat, lon, kind, width_m, height_m}] best match first. Raises GeocodeError."""
    global _last
    q = " ".join(query.split())
    if not q:
        raise GeocodeError("empty place name")
    with _lock:
        if q.lower() in _cache:
            return _cache[q.lower()]
        wait = 1.1 - (time.monotonic() - _last)  # Nominatim allows one request per second
        if wait > 0:
            time.sleep(wait)
        url = f"{URL}?{urllib.parse.urlencode({'q': q, 'format': 'jsonv2', 'limit': limit})}"
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": UA}), timeout=15) as r:
                rows = json.loads(r.read())
        except Exception as exc:
            raise GeocodeError(f"place search failed ({type(exc).__name__}); check the internet connection") from exc
        finally:
            _last = time.monotonic()
        out = [c for c in (_row(r) for r in rows) if c]
        if not out:
            raise GeocodeError(f"no place found for '{q}'")
        _cache[q.lower()] = out
        return out


def _row(r: dict) -> dict | None:
    try:
        lat, lon = float(r["lat"]), float(r["lon"])
        s, n, w, e = (float(v) for v in r["boundingbox"])
    except (KeyError, TypeError, ValueError):
        return None
    import math

    kx = 111_320.0 * math.cos(math.radians(lat))
    return {"name": r.get("display_name", "")[:140], "lat": round(lat, 6), "lon": round(lon, 6),
            "kind": r.get("type", ""), "width_m": round((e - w) * kx), "height_m": round((n - s) * 111_320.0)}  # fmt: skip
