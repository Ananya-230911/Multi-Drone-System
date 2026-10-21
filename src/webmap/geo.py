"""Local metres <-> latitude / longitude for the web map.

The simulation works in a flat rectangle of `size` x `height` metres (x east, y north,
origin at the south-west corner). The web map shows that rectangle on a real map, centred
on (lat0, lon0). `height` defaults to `size` (a square). The conversion is the usual small-area approximation (one degree of
latitude = 111 320 m, one degree of longitude shrinks with cos(latitude)); it is
accurate to well under a metre over the few kilometres the simulation covers.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

M_PER_DEG = 111_320.0


@dataclass(frozen=True)
class GeoFrame:
    lat0: float  # latitude of the area centre
    lon0: float  # longitude of the area centre
    size: float  # width of the area (east-west), metres
    height: float | None = None  # north-south extent, metres; None = same as size

    def __post_init__(self) -> None:
        if not -85.0 <= self.lat0 <= 85.0:
            raise ValueError(f"latitude {self.lat0} is outside the usable range [-85, 85]")
        if not -180.0 <= self.lon0 <= 180.0:
            raise ValueError(f"longitude {self.lon0} is outside [-180, 180]")
        if self.size <= 0 or (self.height is not None and self.height <= 0):
            raise ValueError("area size must be positive")

    @property
    def width(self) -> float:
        return self.size

    @property
    def h(self) -> float:
        return self.size if self.height is None else self.height

    @classmethod
    def from_bounds(cls, south: float, west: float, north: float, east: float) -> "GeoFrame":
        """Frame for the rectangle with these edges (degrees)."""
        if not (-85.0 <= south < north <= 85.0):
            raise ValueError("south must be below north, both within [-85, 85]")
        if not (-180.0 <= west < east <= 180.0):
            raise ValueError("west must be left of east, both within [-180, 180]")
        lat0, lon0 = (south + north) / 2, (west + east) / 2
        kx = M_PER_DEG * math.cos(math.radians(lat0))
        return cls(lat0, lon0, (east - west) * kx, (north - south) * M_PER_DEG)

    @property
    def _kx(self) -> float:
        return M_PER_DEG * math.cos(math.radians(self.lat0))

    def to_xy(self, lat: float, lon: float) -> tuple[float, float]:
        return (self.size / 2 + (lon - self.lon0) * self._kx, self.h / 2 + (lat - self.lat0) * M_PER_DEG)

    def to_latlon(self, x: float, y: float) -> tuple[float, float]:
        return (self.lat0 + (y - self.h / 2) / M_PER_DEG, self.lon0 + (x - self.size / 2) / self._kx)
