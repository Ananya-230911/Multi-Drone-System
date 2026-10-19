import pytest

from src.config import SafetyConfig
from src.safety.monitor import PeerTrack, check_separation, min_distance

CFG = SafetyConfig()  # min 25 + buffer 10 = 35 m, horizon 3 s


def test_min_distance_head_on_and_parallel():
    assert min_distance((100.0, 0.0, 0.0), (-30.0, 0.0, 0.0), 3.0) == pytest.approx(10.0)  # closes 90 m in 3 s
    assert min_distance((100.0, 0.0, 0.0), (-30.0, 0.0, 0.0), 10.0) == pytest.approx(0.0)
    assert min_distance((0.0, 50.0, 0.0), (0.0, 0.0, 0.0), 3.0) == pytest.approx(50.0)
    assert min_distance((40.0, 0.0, 0.0), (10.0, 0.0, 0.0), 3.0) == pytest.approx(40.0)  # moving apart


def converging(peer_id="D2", has_task=False):
    return PeerTrack(peer_id, (60.0, 0.0, 60.0), (-15.0, 0.0, 0.0), has_task)


def test_higher_id_yields_when_neither_has_a_task():
    me = ((0.0, 0.0, 60.0), (15.0, 0.0, 0.0))
    assert check_separation("D3", *me, False, [converging("D2")], CFG).kind == "HOLD"
    assert check_separation("D1", *me, False, [converging("D2")], CFG).ok


def test_drone_without_task_yields_to_drone_with_task():
    me = ((0.0, 0.0, 60.0), (15.0, 0.0, 0.0))
    assert check_separation("D1", *me, False, [converging("D2", has_task=True)], CFG).reason == "SEPARATION:D2"
    assert check_separation("D3", *me, True, [converging("D2", has_task=False)], CFG).ok


def test_no_conflict_when_far_or_vertically_separated():
    me = ((0.0, 0.0, 60.0), (15.0, 0.0, 0.0))
    far = PeerTrack("D1", (500.0, 0.0, 60.0), (-15.0, 0.0, 0.0), False)
    above = PeerTrack("D1", (60.0, 0.0, 100.0), (-15.0, 0.0, 0.0), False)
    assert check_separation("D2", *me, False, [far, above], CFG).ok
