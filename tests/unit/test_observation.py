from dataclasses import replace

import pytest

from src.config import ObservationConfig
from src.safety.geofence import Geofence
from src.tracking.observation import AgentObservation, Observation, validate

FENCE = Geofence([(50, 50), (1950, 50), (1950, 1950), (50, 1950)], [], 15, (15, 120))
CFG = ObservationConfig()  # max_observation_age 3.0
OBS = AgentObservation("D1-1", "D1", "sensor", 10.0, (500.0, 500.0), 0.0, 0.8)
KNOWN = frozenset({"D1", "D2", "EXT"})


def check(obs, now=10.0, seen=frozenset()):
    return validate(obs, now, FENCE, CFG, KNOWN, seen)


def test_valid_observation():
    assert check(OBS) == (True, "OK")


@pytest.mark.parametrize(
    "change, now, reason",
    [
        ({"t": 10.5}, 10.0, "FUTURE_TIMESTAMP"),
        ({"t": 6.9}, 10.0, "TOO_OLD"),
        ({"pos": (20.0, 500.0)}, 10.0, "OUT_OF_BOUNDARY"),
        ({"pos": (float("nan"), 500.0)}, 10.0, "OUT_OF_BOUNDARY"),
        ({"confidence": 1.2}, 10.0, "BAD_CONFIDENCE"),
        ({"confidence": -0.1}, 10.0, "BAD_CONFIDENCE"),
        ({"observer": "D9"}, 10.0, "UNKNOWN_OBSERVER"),
    ],
)
def test_rejections(change, now, reason):
    assert check(replace(OBS, **change), now) == (False, reason)


def test_exactly_max_age_is_accepted():
    assert check(replace(OBS, t=7.0))[0]


def test_duplicate_obs_id():
    assert check(OBS, seen={"D1-1"}) == (False, "DUPLICATE")


def test_observation_near_boundary_inside_operating_area_is_valid():
    # Inside the raw boundary but within the flight margin: a target there is still real.
    assert check(replace(OBS, pos=(55.0, 500.0)))[0]


def test_to_agent_strips_ground_truth_flag():
    o = Observation("D1-1", "D1", "sensor", 1.0, (1.0, 2.0), 0.0, 0.5, is_spurious=True)
    a = o.to_agent()
    assert type(a) is AgentObservation and not hasattr(a, "is_spurious")
    assert (a.obs_id, a.pos, a.confidence) == ("D1-1", (1.0, 2.0), 0.5)
