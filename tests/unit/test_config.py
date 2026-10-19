import copy
from pathlib import Path

import pytest
import yaml

from src.config import ConfigError, config_from_dict, load_config

ROOT = Path(__file__).resolve().parents[2]
DEFAULT = ROOT / "config" / "default.yaml"


@pytest.fixture
def raw():
    return yaml.safe_load(DEFAULT.read_text())


def test_default_config_loads_with_section_12_values():
    cfg = load_config(DEFAULT)
    assert cfg.name == "default"
    assert cfg.sim.dt == 0.1 and cfg.sim.seed == 42
    assert cfg.drone_ids == ["D1", "D2", "D3", "D4"]
    assert cfg.geofence.restricted_zones[0].id == "RZ-1"
    assert cfg.allocation.weights.energy == 0.5
    d3 = next(d for d in cfg.drones if d.id == "D3")
    assert d3.battery_pct == 0.30 and d3.params.sensor.radius == 80


def test_later_phase_sections_reported_as_info():
    cfg = load_config(DEFAULT)
    assert any("'reacquire'" in m for m in cfg.info)  # used from Phase 4
    assert not any("'allocation'" in m for m in cfg.info)
    assert not any("'drones'" in m for m in cfg.info)


def test_missing_sections_use_defaults(raw):
    minimal = {"drones": raw["drones"]}
    cfg = config_from_dict(minimal)
    assert cfg.sim.duration == 400 and cfg.track.lost_after == 3.0


def test_drone_overrides_merge_with_defaults(raw):
    raw["drones"][0]["max_speed"] = 20
    raw["drones"][0]["sensor"] = {"radius": 120}
    cfg = config_from_dict(raw)
    d1 = next(d for d in cfg.drones if d.id == "D1")
    assert d1.params.max_speed == 20
    assert d1.params.sensor.radius == 120
    assert d1.params.sensor.half_fov_deg == 45  # untouched default kept


@pytest.mark.parametrize(
    "mutate, match",
    [
        (lambda r: r.update(bogus=1), "unknown top-level"),
        (lambda r: r["sim"].update(dtt=0.1), "unknown key"),
        (lambda r: r["sim"].update(dt=-1), "sim.dt"),
        (lambda r: r["sim"].update(seed="x"), "expected an integer"),
        (lambda r: r["network"].update(packet_loss=1.5), "packet_loss"),
        (lambda r: r["track"].update(lost_after=1.0), "lost_after"),
        (lambda r: r["reacquire"].update(timeout_action="explode"), "must be one of"),
        (lambda r: r["drones"][1].update(id="D1"), "duplicate id"),
        (lambda r: r["drones"][0].update(patrol=[]), "patrol"),
        (lambda r: r["drones"][0].update(battery_pct=1.2), "battery_pct"),
        (lambda r: r["drones"][0].update(pos=[1, 2, 3]), "list of 2"),
        (lambda r: r["drones"][0].pop("home"), "missing required key 'home'"),
        (lambda r: r["drone_defaults"].update(critical_pct=0.5), "critical_pct"),
        (lambda r: r["geofence"].update(boundary=[[0, 0], [1, 1]]), "at least 3 points"),
        (lambda r: r["navigation"].update(cruise_alt=500), "cruise_alt"),
        (lambda r: r.update(drones=[]), "at least one drone"),
        (lambda r: r.update(scenario_events=[{"t": 5, "type": "fail_drone", "drone": "D9"}]), "unknown drone"),
        (lambda r: r.update(scenario_events=[{"t": 5, "type": "teleport"}]), "unknown event type"),
        (lambda r: r.update(scenario_events=[{"t": 5, "type": "set_battery", "drone": "D1"}]), "requires 'battery_pct'"),
        (lambda r: r["network"].update(partitions=[{"start": 1, "end": 2, "groups": [["D1"], ["DX"]]}]), "unknown drone"),
    ],
)
def test_invalid_configs_rejected(raw, mutate, match):
    bad = copy.deepcopy(raw)
    mutate(bad)
    with pytest.raises(ConfigError, match=match):
        config_from_dict(bad)


def test_missing_file():
    with pytest.raises(ConfigError, match="not found"):
        load_config(ROOT / "config" / "nope.yaml")
