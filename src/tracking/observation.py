"""Observations and their validation (PLAN.md sections 5 and 7.6).

`Observation` carries the ground-truth `is_spurious` flag for tests and
evaluation. Agents only ever receive `AgentObservation`, which does not have it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from src.config import ObservationConfig, Point2
from src.safety.geofence import Geofence

Source = Literal["sensor", "external_cue", "relayed"]


@dataclass(frozen=True)
class AgentObservation:
    obs_id: str  # f"{observer}-{seq}"
    observer: str  # drone ID, or "EXT" for an external cue
    source: Source
    t: float
    pos: Point2
    alt: float | None
    confidence: float  # SIMULATED score in [0, 1]; not a real detection accuracy


@dataclass(frozen=True)
class Observation(AgentObservation):
    is_spurious: bool = False  # ground truth; stripped before an agent sees the observation

    def to_agent(self) -> AgentObservation:
        return AgentObservation(self.obs_id, self.observer, self.source, self.t, self.pos, self.alt, self.confidence)


def validate(
    obs: AgentObservation,
    now: float,
    fence: Geofence,
    cfg: ObservationConfig,
    known_observers: frozenset[str] | set[str],
    seen_ids: set[str] | frozenset[str] = frozenset(),
) -> tuple[bool, str]:
    """(ok, reason). Reason codes match the OBSERVATION_REJECTED event."""
    if obs.t > now + 1e-9:
        return False, "FUTURE_TIMESTAMP"
    if now - obs.t > cfg.max_observation_age:
        return False, "TOO_OLD"
    if not all(math.isfinite(v) for v in obs.pos) or not fence.in_operating_area(*obs.pos):
        return False, "OUT_OF_BOUNDARY"
    if not 0.0 <= obs.confidence <= 1.0:
        return False, "BAD_CONFIDENCE"
    if obs.observer not in known_observers:
        return False, "UNKNOWN_OBSERVER"
    if obs.obs_id in seen_ids:
        return False, "DUPLICATE"
    return True, "OK"
