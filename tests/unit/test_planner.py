import math

import pytest

from src.safety.geofence import Geofence
from src.safety.planner import Planner, inspection_point
from src.safety.route_validator import route_length, validate_route

SQUARE = [(50, 50), (1950, 50), (1950, 1950), (50, 1950)]
ZONE = ("RZ-1", [(800, 600), (1100, 600), (1100, 1000), (800, 1000)])


@pytest.fixture
def planner():
    f = Geofence(SQUARE, [ZONE], margin=15, altitude_limits=(15, 120), homes=[(100, 100)])
    return Planner(f, cruise_alt=60, clearance=1.0)


def test_direct_route_when_clear(planner):
    res = planner.plan((200, 200, 60), (600, 300, 60))
    assert res.route == [(200, 200, 60), (600, 300, 60)]


def test_detour_around_zone(planner):
    start, goal = (700, 800, 60), (1300, 800, 60)
    res = planner.plan(start, goal)
    assert res.ok and len(res.route) > 2
    assert validate_route(res.route, planner.fence).ok
    assert route_length(res.route) > math.dist(start[:2], goal[:2])
    # shortest way round a 330 x 430 buffered box from its middle: via two corners
    assert route_length(res.route) == pytest.approx(
        math.dist((700, 800), (784, 584)) + 332 + math.dist((1116, 584), (1300, 800)), abs=1.0
    )


def test_planning_is_deterministic(planner):
    a = planner.plan((700, 800, 60), (1300, 800, 60)).route
    b = planner.plan((700, 800, 60), (1300, 800, 60)).route
    assert a == b


def test_no_route_when_goal_invalid(planner):
    assert planner.plan((200, 200, 60), (950, 800, 60)).reason == "GOAL_INVALID"
    assert planner.plan((20, 20, 60), (300, 300, 60)).reason == "START_INVALID"


def test_no_route_when_region_is_cut_off():
    # A zone spanning the full height splits the area in two.
    f = Geofence(SQUARE, [("WALL", [(900, 0), (1100, 0), (1100, 2000), (900, 2000)])], 15, (15, 120))
    p = Planner(f, 60, 1.0)
    res = p.plan((300, 1000, 60), (1600, 1000, 60))
    assert res.route is None and res.reason == "NO_PATH"


def test_concave_boundary_routes_through_inner_corner():
    L_shape = [(0, 0), (1000, 0), (1000, 500), (500, 500), (500, 1000), (0, 1000)]
    f = Geofence(L_shape, [], margin=10, altitude_limits=(15, 120))
    res = Planner(f, 60, 1.0).plan((250, 900, 60), (900, 250, 60))
    assert res.ok and len(res.route) == 3
    assert validate_route(res.route, f).ok
    corner = res.route[1]
    assert corner[0] == pytest.approx(489) and corner[1] == pytest.approx(489)


def test_takeoff_and_landing_are_vertical_at_home(planner):
    up = planner.plan((100, 100, 0), (600, 300, 60)).route
    assert up[:2] == [(100, 100, 0), (100, 100, 60)]
    down = planner.plan((600, 300, 60), (100, 100, 0)).route
    assert down[-2:] == [(100, 100, 60), (100, 100, 0)]


def test_interior_waypoints_at_cruise_altitude(planner):
    route = planner.plan((700, 800, 40), (1300, 800, 90)).route
    assert route[0][2] == 40 and route[-1][2] == 90
    assert all(w[2] == 60 for w in route[1:-1])


def test_inspection_point_above_target_when_allowed(planner):
    assert inspection_point((1300, 900), 60, planner, 60, 16, from_pos=(1500, 400)) == (1300, 900, 60)


def test_inspection_point_at_zone_edge_for_target_inside(planner):
    p = inspection_point((1090, 800), 60, planner, 60, 16, from_pos=(1300, 800))
    assert p is not None
    assert planner.fence.contains_point(*p)
    assert math.dist(p[:2], (1090, 800)) < 60
    assert p[0] == pytest.approx(1116, abs=0.01)


def test_inspection_point_none_when_target_deep_inside(planner):
    assert inspection_point((950, 800), 60, planner, 60, 16, from_pos=(1300, 800)) is None


def test_inspection_point_circle_fallback_without_approach(planner):
    # No straight approach (from_pos inside the zone shadow): falls back to circle candidates.
    p = inspection_point((1090, 990), 60, planner, 60, 16)
    assert p is not None and planner.fence.contains_point(*p)
    assert math.dist(p[:2], (1090, 990)) == pytest.approx(54)
