import copy
from pathlib import Path

import pytest
import yaml

from src.config import ConfigError, config_from_dict
from src.safety.geofence import Geofence

SQUARE = [(50, 50), (1950, 50), (1950, 1950), (50, 1950)]
ZONE = ("RZ-1", [(800, 600), (1100, 600), (1100, 1000), (800, 1000)])


@pytest.fixture
def fence():
    return Geofence(SQUARE, [ZONE], margin=15, altitude_limits=(15, 120), homes=[(100, 100)])


def test_inside_and_outside(fence):
    assert fence.contains_point(500, 500, 60)
    assert fence.violation(20, 500, 60) == "OUT_OF_BOUNDARY"
    assert fence.violation(950, 800, 60) == "IN_ZONE:RZ-1"


def test_margin_shrinks_boundary_and_grows_zones(fence):
    assert fence.violation(60, 500, 60) == "OUT_OF_BOUNDARY"  # inside the raw boundary, within the margin
    assert fence.violation(1110, 800, 60) == "IN_ZONE:RZ-1"  # outside the raw zone, within the margin
    assert fence.contains_point(1116, 800, 60)


def test_point_exactly_on_buffered_edge_is_outside(fence):
    assert fence.violation(65, 500, 60) == "OUT_OF_BOUNDARY"
    assert fence.contains_point(65.001, 500, 60)
    assert fence.violation(1115, 800, 60) == "IN_ZONE:RZ-1"  # on the buffered zone edge
    assert fence.contains_point(1115.001, 800, 60)


def test_mitre_margin_keeps_rectangular_corners(fence):
    # A round buffer would allow this corner point; the mitre buffer (conservative) does not.
    assert fence.violation(1112, 1012, 60) == "IN_ZONE:RZ-1"


def test_altitude_limits(fence):
    assert fence.violation(500, 500, 14.9) == "ALT_BELOW_MIN"
    assert fence.violation(500, 500, 120.1) == "ALT_ABOVE_MAX"
    assert fence.contains_point(500, 500, 15) and fence.contains_point(500, 500, 120)


def test_home_corridor_allows_low_altitude_only_at_home(fence):
    assert fence.contains_point(100, 100, 0)
    assert fence.violation(101, 100, 0) == "ALT_BELOW_MIN"


def test_concave_boundary():
    # L-shaped boundary: the top-right quadrant is outside.
    L_shape = [(0, 0), (1000, 0), (1000, 500), (500, 500), (500, 1000), (0, 1000)]
    f = Geofence(L_shape, [], margin=10, altitude_limits=(15, 120))
    assert f.contains_point(250, 750, 60)
    assert f.contains_point(750, 250, 60)
    assert f.violation(750, 750, 60) == "OUT_OF_BOUNDARY"
    assert f.violation(495, 750, 60) == "OUT_OF_BOUNDARY"  # within the margin of the inner corner edge


def test_operating_area_uses_raw_boundary(fence):
    assert fence.in_operating_area(55, 500)  # within the margin but inside the operating area
    assert not fence.in_operating_area(40, 500)


def _raw():
    return yaml.safe_load((Path(__file__).resolve().parents[2] / "config" / "default.yaml").read_text())


def test_self_intersecting_boundary_rejected():
    raw = copy.deepcopy(_raw())
    raw["geofence"]["boundary"] = [[0, 0], [1000, 1000], [1000, 0], [0, 1000]]  # bow tie
    with pytest.raises(ConfigError, match="not a valid polygon"):
        Geofence.from_config(config_from_dict(raw))


def test_zone_outside_boundary_rejected():
    raw = copy.deepcopy(_raw())
    raw["geofence"]["restricted_zones"] = [{"id": "FAR", "polygon": [[5000, 5000], [5100, 5000], [5100, 5100]]}]
    with pytest.raises(ConfigError, match="does not overlap"):
        Geofence.from_config(config_from_dict(raw))
