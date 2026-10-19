import pytest

from src.config import DroneParams
from src.drone.battery import Battery, EnergyModel, route_cost

P = DroneParams()  # section 12 defaults
M = EnergyModel.from_params(P)


def test_level_flight_energy_is_linear_in_distance():
    assert M.move_energy(100, 0, 0) == pytest.approx(100 * P.k_h)
    assert M.move_energy(200, 0, 0) == pytest.approx(2 * M.move_energy(100, 0, 0))


def test_climbing_costs_more_than_level_flight_per_meter():
    assert M.move_energy(0, 10, 0) > M.move_energy(10, 0, 0)


def test_descending_costs_less_than_climbing():
    assert M.move_energy(0, 0, 10) < M.move_energy(0, 10, 0)


def test_same_horizontal_path_with_climb_costs_more():
    flat = route_cost((0, 0, 60), [(1000, 0, 60)], M)
    climbing = route_cost((0, 0, 20), [(1000, 0, 100)], M)
    assert climbing.horizontal == pytest.approx(flat.horizontal)
    assert climbing.energy_wh > flat.energy_wh


def test_hover_energy():
    assert M.hover_energy(10) == pytest.approx(10 * P.hover_power)


def test_negative_inputs_rejected():
    with pytest.raises(ValueError):
        M.move_energy(-1, 0, 0)
    with pytest.raises(ValueError):
        M.hover_energy(-1)


def test_route_cost_separates_climb_and_descent():
    rc = route_cost((0, 0, 0), [(0, 0, 60), (300, 400, 60), (300, 400, 20)], M)
    assert rc.horizontal == pytest.approx(500)
    assert rc.climb == pytest.approx(60)
    assert rc.descent == pytest.approx(40)
    assert rc.energy_wh == pytest.approx(500 * P.k_h + 60 * P.k_c + 40 * P.k_d)


def test_route_cost_uses_route_length_not_straight_line():
    straight = route_cost((0, 0, 60), [(1000, 0, 60)], M)
    detour = route_cost((0, 0, 60), [(500, 500, 60), (1000, 0, 60)], M)
    assert detour.horizontal > straight.horizontal
    assert detour.energy_wh > straight.energy_wh


def test_battery_drain_and_clamp():
    b = Battery.from_params(P, 0.5)
    assert b.charge_wh == pytest.approx(50)
    assert b.drain(10) == pytest.approx(10)
    assert b.pct == pytest.approx(0.4)
    assert b.drain(100) == pytest.approx(40)  # only what is left
    assert b.charge_wh == 0 and b.empty
    with pytest.raises(ValueError):
        b.drain(-1)


def test_battery_reserve_and_critical_levels():
    b = Battery.from_params(P, 1.0)
    assert b.reserve_wh == pytest.approx(P.reserve_pct * P.capacity_wh)
    assert b.critical_wh == pytest.approx(P.critical_pct * P.capacity_wh)


def test_reserve_violation_rule():
    b = Battery(capacity_wh=100, charge_wh=30, reserve_pct=0.2, critical_pct=0.08)
    assert not b.reserve_violated(10)  # 30 - 10 = 20, exactly at reserve
    assert b.reserve_violated(10.001)


def test_set_pct_clamped():
    b = Battery.from_params(P, 0.5)
    b.set_pct(1.5)
    assert b.pct == 1.0
    b.set_pct(-0.2)
    assert b.pct == 0.0
