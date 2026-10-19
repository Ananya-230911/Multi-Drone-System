import math
from dataclasses import replace

import numpy as np
import pytest

from src.config import ConfidenceConfig, SensorConfig
from src.drone.sensor import Sensor, confidence_category, effective_radius
from src.sim.ground_truth import GroundTruth, GroundTruthTarget
from src.config import TargetConfig

S = SensorConfig()  # radius 80, half_fov 45 deg, band [20, 120], 2 Hz, p_detect 0.9
C = ConfidenceConfig()
T = GroundTruthTarget("GT-1", (0.0, 0.0))


def sensor(**kw):
    return Sensor("D1", replace(S, **kw), C, np.random.default_rng(0))


def run(s, x, y, alt, seconds, targets=(T,), dt=0.1):
    out, t = [], 0.0
    for _ in range(int(round(seconds / dt))):
        t = round(t + dt, 9)
        out += s.sample(t, dt, x, y, alt, list(targets))
    return out


def test_effective_radius():
    assert effective_radius(S, 60) == pytest.approx(60)  # alt * tan(45) = 60 < 80
    assert effective_radius(S, 100) == pytest.approx(80)  # capped by radius
    assert effective_radius(S, 19.9) == 0.0 and effective_radius(S, 120.1) == 0.0


def test_inside_vs_outside_effective_radius():
    s = sensor(p_detect=1.0)
    assert run(s, 59, 0, 60, 2)
    assert not run(sensor(p_detect=1.0), 61, 0, 60, 2)


@pytest.mark.parametrize("alt, expect", [(19.99, False), (20.0, True), (120.0, True), (120.01, False)])
def test_altitude_band_edges(alt, expect):
    assert bool(run(sensor(p_detect=1.0), 0, 0, alt, 2)) is expect


def test_rate_hz_limits_samples():
    obs = run(sensor(p_detect=1.0), 0, 0, 60, 10)
    assert len(obs) == 20  # 2 Hz for 10 s
    ts = [o.t for o in obs]
    assert all(b - a == pytest.approx(0.5) for a, b in zip(ts, ts[1:]))


def test_p_detect_statistics():
    obs = run(sensor(p_detect=0.9, rate_hz=10), 0, 0, 60, 300)
    assert len(obs) / 3000 == pytest.approx(0.9, abs=0.02)


def test_hidden_interval_produces_no_observations():
    gt = GroundTruth([TargetConfig("GT-1", (0.0, 0.0), 0.0, 0.0, [(2.0, 4.0)])])
    s = sensor(p_detect=1.0)
    seen = []
    for k in range(1, 61):
        t = round(k * 0.1, 9)
        seen += [(t, o) for o in s.sample(t, 0.1, 0, 0, 60, gt.visible(t))]
    assert seen and not [t for t, _ in seen if 2.0 <= t < 4.0]
    assert [t for t, _ in seen if t >= 4.0]


def test_confidence_falls_with_distance():
    near = run(sensor(p_detect=1.0, position_noise_std=0.0), 5, 0, 60, 20)
    far = run(sensor(p_detect=1.0, position_noise_std=0.0), 45, 0, 60, 20)
    assert np.mean([o.confidence for o in near]) > np.mean([o.confidence for o in far]) + 0.4
    assert all(0.0 <= o.confidence <= 1.0 for o in near + far)


def test_position_noise():
    obs = run(sensor(p_detect=1.0, rate_hz=10, position_noise_std=3.0), 0, 0, 60, 200)
    xs = np.array([o.pos[0] for o in obs])
    assert abs(xs.mean()) < 0.3 and xs.std() == pytest.approx(3.0, rel=0.1)


def test_spurious_observations():
    s = sensor(false_obs_rate=1.0)
    obs = run(s, 500, 500, 60, 200, targets=())
    assert len(obs) == pytest.approx(200, rel=0.15)
    assert all(o.is_spurious for o in obs)
    lo, hi = C.spurious_conf_range
    assert all(lo <= o.confidence <= hi for o in obs)
    assert all(math.dist(o.pos, (500, 500)) <= 60 for o in obs)
    assert not hasattr(obs[0].to_agent(), "is_spurious")


def test_observation_ids_are_unique_and_sequential():
    obs = run(sensor(p_detect=1.0), 0, 0, 60, 3)
    assert [o.obs_id for o in obs] == [f"D1-{i}" for i in range(1, len(obs) + 1)]


def test_same_seed_same_observations():
    a = run(sensor(), 10, 0, 60, 20)
    b = run(sensor(), 10, 0, 60, 20)
    assert a == b


def test_confidence_category():
    assert confidence_category(0.39, C) == "LOW"
    assert confidence_category(0.4, C) == "MEDIUM"
    assert confidence_category(0.7, C) == "HIGH"
