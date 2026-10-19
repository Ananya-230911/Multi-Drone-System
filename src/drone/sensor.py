"""Simulated downward-looking sensor (PLAN.md sections 7.2 and 7.3).

The world owns one `Sensor` per drone and calls `sample` with the drone's pose
and the visible ground-truth targets; the drone only receives the resulting
observations (with the ground-truth `is_spurious` flag stripped).

Confidence is a SIMULATION PARAMETER that exercises the decision logic. It does
not represent real-world detection accuracy.
"""

from __future__ import annotations

import math
from typing import Iterable, Protocol

import numpy as np

from src.config import ConfidenceConfig, SensorConfig
from src.tracking.observation import Observation


class _Target(Protocol):
    pos: tuple[float, float]
    alt: float


def effective_radius(sensor: SensorConfig, alt: float) -> float:
    """r_eff = min(radius, alt * tan(half_fov)); 0 outside the sensor's altitude band."""
    lo, hi = sensor.alt_band
    if not lo <= alt <= hi:
        return 0.0
    return min(sensor.radius, alt * math.tan(math.radians(sensor.half_fov_deg)))


def confidence_category(conf: float, cfg: ConfidenceConfig) -> str:
    if conf < cfg.low_threshold:
        return "LOW"
    if conf < cfg.medium_threshold:
        return "MEDIUM"
    return "HIGH"


class Sensor:
    def __init__(self, observer: str, cfg: SensorConfig, conf_cfg: ConfidenceConfig, rng: np.random.Generator):
        self.observer = observer
        self.cfg = cfg
        self.conf_cfg = conf_cfg
        self.rng = rng
        self._seq = 0
        self._next_sample: float | None = None

    def sample(self, now: float, dt: float, x: float, y: float, alt: float, targets: Iterable[_Target]) -> list[Observation]:
        r_eff = effective_radius(self.cfg, alt)
        if r_eff <= 0.0:
            return []
        out: list[Observation] = []
        # Real detections at rate_hz.
        if self._next_sample is None:
            self._next_sample = now
        if now + 1e-9 >= self._next_sample:
            while self._next_sample <= now + 1e-9:
                self._next_sample += 1.0 / self.cfg.rate_hz
            for t in targets:
                d = math.hypot(t.pos[0] - x, t.pos[1] - y)
                if d > r_eff:
                    continue
                if self.rng.random() >= self.cfg.p_detect:
                    continue
                c = self.conf_cfg
                noise = self.rng.normal(0.0, self.cfg.position_noise_std, size=2) if self.cfg.position_noise_std > 0 else (0.0, 0.0)
                conf = c.base * (1.0 - d / r_eff) ** c.falloff_exp
                if c.noise_std > 0:
                    conf += self.rng.normal(0.0, c.noise_std)
                out.append(self._make(now, (t.pos[0] + noise[0], t.pos[1] + noise[1]), t.alt, conf, False))
        # Spurious observations: Poisson process at false_obs_rate, anywhere in the footprint.
        if self.cfg.false_obs_rate > 0:
            for _ in range(self.rng.poisson(self.cfg.false_obs_rate * dt)):
                rr = r_eff * math.sqrt(self.rng.random())
                ang = 2 * math.pi * self.rng.random()
                lo, hi = self.conf_cfg.spurious_conf_range
                conf = float(self.rng.uniform(lo, hi))
                out.append(self._make(now, (x + rr * math.cos(ang), y + rr * math.sin(ang)), 0.0, conf, True))
        return out

    def _make(self, now: float, pos, alt: float, conf: float, spurious: bool) -> Observation:
        self._seq += 1
        return Observation(
            obs_id=f"{self.observer}-{self._seq}",
            observer=self.observer,
            source="sensor",
            t=now,
            pos=(float(pos[0]), float(pos[1])),
            alt=alt,
            confidence=float(min(max(conf, 0.0), 1.0)),
            is_spurious=spurious,
        )
