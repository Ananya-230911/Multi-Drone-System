import pytest

from src.safety.geofence import Geofence
from src.safety.route_validator import route_length, validate_route, validate_segment

SQUARE = [(50, 50), (1950, 50), (1950, 1950), (50, 1950)]
ZONE = ("RZ-1", [(800, 600), (1100, 600), (1100, 1000), (800, 1000)])  # buffered: 785..1115 x 585..1015


@pytest.fixture
def fence():
    return Geofence(SQUARE, [ZONE], margin=15, altitude_limits=(15, 120), homes=[(100, 100)])


def test_clear_segment_valid(fence):
    assert validate_segment((200, 200, 60), (600, 300, 60), fence).ok


def test_segment_through_zone(fence):
    r = validate_segment((700, 800, 60), (1200, 800, 60), fence)
    assert not r.ok and r.reason == "CROSSES_ZONE:RZ-1"


def test_segment_clipping_zone_corner(fence):
    # Cuts across the buffered corner at (1115, 1015) without reaching the raw zone.
    r = validate_segment((1100, 1030, 60), (1130, 1000, 60), fence)
    assert not r.ok and r.reason == "CROSSES_ZONE:RZ-1"


def test_segment_touching_zone_edge_is_invalid(fence):
    r = validate_segment((1115, 1100, 60), (1115, 1015, 60), fence)  # ends exactly on the buffered edge
    assert not r.ok and r.reason == "CROSSES_ZONE:RZ-1"
    assert validate_segment((1116, 1100, 60), (1116, 900, 60), fence).ok  # 1 m clear


def test_segment_leaving_and_reentering_concave_boundary():
    L_shape = [(0, 0), (1000, 0), (1000, 500), (500, 500), (500, 1000), (0, 1000)]
    f = Geofence(L_shape, [], margin=10, altitude_limits=(15, 120))
    # Both ends inside, but the straight line cuts across the outside notch.
    assert f.contains_point(250, 900, 60) and f.contains_point(900, 250, 60)
    r = validate_segment((250, 900, 60), (900, 250, 60), f)
    assert not r.ok and r.reason == "OUT_OF_BOUNDARY"


def test_segment_touching_boundary_edge_is_invalid(fence):
    assert validate_segment((65, 500, 60), (300, 500, 60), fence).reason == "OUT_OF_BOUNDARY"


@pytest.mark.parametrize("alt, reason", [(10, "ALT_BELOW_MIN"), (130, "ALT_ABOVE_MAX")])
def test_altitude_out_of_range(fence, alt, reason):
    assert validate_segment((200, 200, 60), (300, 300, alt), fence).reason == reason


def test_vertical_segment_at_home_may_go_below_min(fence):
    assert validate_segment((100, 100, 0), (100, 100, 60), fence).ok
    # leaving home diagonally at ground level would fly below the floor away from home
    assert validate_segment((100, 100, 0), (300, 300, 60), fence).reason == "ALT_BELOW_MIN"


def test_validate_route_reports_first_failure(fence):
    route = [(200, 200, 60), (700, 800, 60), (1200, 800, 60), (1300, 800, 60)]
    r = validate_route(route, fence)
    assert not r.ok and r.index == 1 and r.reason == "CROSSES_ZONE:RZ-1"


def test_validate_route_start_and_goal(fence):
    assert validate_route([(20, 20, 60), (300, 300, 60)], fence).reason == "START_INVALID"
    r = validate_route([(300, 300, 60), (950, 800, 60)], fence)
    assert r.reason == "GOAL_INVALID" and r.index == 1
    assert validate_route([], fence).reason == "GOAL_INVALID"


def test_route_length_is_horizontal():
    assert route_length([(0, 0, 0), (0, 0, 60), (300, 400, 60)]) == pytest.approx(500)
