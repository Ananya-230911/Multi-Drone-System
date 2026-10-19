"""Geofence model (PLAN.md section 9.1).

- Operating boundary: flight allowed only strictly inside it.
- Restricted zones: full-height keep-out polygons.
- Altitude limits: global [min_alt, max_alt].
- Margin: the boundary is shrunk and zones are grown by `margin` (mitre joins,
  so a grown rectangle stays a rectangle; that is a superset of the round
  buffer, i.e. slightly more conservative).
- Edge rule: a point exactly on a buffered edge counts as outside.

Home corridor: each drone's home may sit below `min_alt` (it takes off and
lands there). A point exactly above a home position is exempt from the
minimum altitude, and so is a purely vertical segment at a home position.
Nowhere else may a drone fly below `min_alt`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from shapely.validation import explain_validity

from src.config import Config, ConfigError, Point2

_EPS = 1e-6


@dataclass(frozen=True)
class ZoneShape:
    id: str
    raw: Polygon
    buffered: Polygon


class Geofence:
    def __init__(
        self,
        boundary: list[Point2],
        zones: list[tuple[str, list[Point2]]],
        margin: float,
        altitude_limits: tuple[float, float],
        homes: list[Point2] = (),
    ):
        self.margin = margin
        self.min_alt, self.max_alt = altitude_limits
        self.raw_boundary = Polygon(boundary)
        self.boundary = self.raw_boundary.buffer(-margin, join_style="mitre")
        self.zones = [
            ZoneShape(zid, Polygon(poly), Polygon(poly).buffer(margin, join_style="mitre")) for zid, poly in zones
        ]
        self.homes = tuple((float(x), float(y)) for x, y in homes)
        self.allowed = self.boundary.difference(unary_union([z.buffered for z in self.zones])) if self.zones else self.boundary
        self._boundary_lines = self.boundary.boundary
        for g in [self.raw_boundary, self.boundary, self._boundary_lines] + [z.buffered for z in self.zones]:
            shapely.prepare(g)

    @classmethod
    def from_config(cls, cfg: Config) -> "Geofence":
        g = cfg.geofence
        validate_polygons(cfg)
        fence = cls(
            g.boundary,
            [(z.id, z.polygon) for z in g.restricted_zones],
            g.margin,
            g.altitude_limits,
            homes=[d.home[:2] for d in cfg.drones],
        )
        if fence.boundary.is_empty or fence.boundary.area <= 0:
            raise ConfigError("geofence: boundary is empty after shrinking by the margin")
        return fence

    # ------------------------------------------------------------------ points

    def at_home(self, x: float, y: float) -> bool:
        return any(abs(x - hx) <= _EPS and abs(y - hy) <= _EPS for hx, hy in self.homes)

    def horizontal_violation(self, x: float, y: float) -> str | None:
        if not shapely.contains_xy(self.boundary, x, y):  # points on the edge are not "contained": edge = outside
            return "OUT_OF_BOUNDARY"
        for z in self.zones:
            if shapely.intersects_xy(z.buffered, x, y):  # touching the zone edge counts as inside the zone
                return f"IN_ZONE:{z.id}"
        return None

    def altitude_violation(self, x: float, y: float, alt: float) -> str | None:
        if alt > self.max_alt:
            return "ALT_ABOVE_MAX"
        if alt < self.min_alt and not self.at_home(x, y):
            return "ALT_BELOW_MIN"
        return None

    def violation(self, x: float, y: float, alt: float) -> str | None:
        """Reason code if (x, y, alt) is not allowed, else None."""
        return self.horizontal_violation(x, y) or self.altitude_violation(x, y, alt)

    def contains_point(self, x: float, y: float, alt: float) -> bool:
        return self.violation(x, y, alt) is None

    def in_operating_area(self, x: float, y: float) -> bool:
        """Inside the raw (un-buffered) operating boundary. Used to validate target observations."""
        return bool(self.raw_boundary.covers(Point(x, y)))

    # ------------------------------------------------------------------ segments

    def horizontal_segment_violation(self, a: Point2, b: Point2) -> str | None:
        if math.dist(a, b) <= _EPS:  # vertical-only move: check the single horizontal position
            v = self.horizontal_violation(*a)
            return v.replace("IN_ZONE:", "CROSSES_ZONE:") if v else None
        line = LineString([a, b])
        if not self.boundary.contains(line) or line.intersects(self._boundary_lines):
            return "OUT_OF_BOUNDARY"
        for z in self.zones:
            if line.intersects(z.buffered):
                return f"CROSSES_ZONE:{z.id}"
        return None


def validate_polygons(cfg: Config) -> None:
    """Geometric checks from section 9.1 that need shapely."""
    g = cfg.geofence
    b = Polygon(g.boundary)
    if not b.is_valid:
        raise ConfigError(f"geofence.boundary is not a valid polygon: {explain_validity(b)}")
    for i, z in enumerate(g.restricted_zones):
        zp = Polygon(z.polygon)
        if not zp.is_valid:
            raise ConfigError(f"geofence.restricted_zones[{i}] ({z.id}) is not valid: {explain_validity(zp)}")
        if not zp.intersects(b) or zp.intersection(b).area <= 0:
            raise ConfigError(f"geofence.restricted_zones[{i}] ({z.id}) does not overlap the operating boundary")
