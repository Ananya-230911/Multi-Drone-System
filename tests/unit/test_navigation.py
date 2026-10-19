import math

import pytest

from src.drone.navigation import Navigator

V, C = 15.0, 3.0  # max_speed, climb_rate


def test_horizontal_speed_limited():
    nav = Navigator([(1000, 0, 60)])
    r = nav.step(0, 0, 60, 0.1, V, C)
    assert r.x == pytest.approx(1.5) and r.y == pytest.approx(0)
    assert r.horizontal == pytest.approx(1.5)


def test_vertical_speed_limited_and_simultaneous():
    nav = Navigator([(1000, 0, 100)])
    r = nav.step(0, 0, 60, 1.0, V, C)
    assert r.x == pytest.approx(15) and r.alt == pytest.approx(63)
    assert r.climb == pytest.approx(3) and r.descent == 0


def test_leftover_time_carries_past_waypoint():
    nav = Navigator([(1, 0, 60), (1, 10, 60)])
    r = nav.step(0, 0, 60, 0.2, V, C)  # 3 m of travel: 1 m to the corner, 2 m on the next leg
    assert r.reached == [0]
    assert (r.x, r.y) == pytest.approx((1, 2))
    assert r.horizontal == pytest.approx(3)


def test_non_looping_route_finishes_and_reports_idle_time():
    nav = Navigator([(1.5, 0, 60)])
    r = nav.step(0, 0, 60, 0.2, V, C)
    assert r.finished and nav.finished
    assert r.idle_time == pytest.approx(0.1)
    assert nav.remaining() == []


def test_loop_wraps_to_first_waypoint():
    nav = Navigator([(0, 0, 60), (1.5, 0, 60)], loop=True, start_index=1)
    r = nav.step(0, 0, 60, 0.2, V, C)
    assert r.reached == [1, 0]
    assert not r.finished and nav.index == 1


def test_single_point_loop_hovers_without_hanging():
    nav = Navigator([(5, 5, 60)], loop=True)
    r = nav.step(5, 5, 60, 0.1, V, C)
    assert (r.x, r.y, r.alt) == (5, 5, 60)
    assert r.idle_time == pytest.approx(0.1)


def test_patrol_lap_time_matches_perimeter():
    route = [(0, 0, 60), (300, 0, 60), (300, 300, 60), (0, 300, 60)]
    nav = Navigator(route, loop=True)
    x, y, z, t, laps = 0.0, 0.0, 60.0, 0.0, 0
    while laps < 2 and t < 1000:
        r = nav.step(x, y, z, 0.1, V, C)
        x, y, z = r.x, r.y, r.alt
        t += 0.1
        laps += r.reached.count(0)
    # first reach of index 0 is immediate (start), the second after one full 1200 m lap
    assert t == pytest.approx(1200 / V, abs=0.11)
    assert math.hypot(x, y) == pytest.approx(0, abs=1e-6)


def test_invalid_construction():
    with pytest.raises(ValueError):
        Navigator([])
    with pytest.raises(ValueError):
        Navigator([(0, 0, 0)], start_index=3)
