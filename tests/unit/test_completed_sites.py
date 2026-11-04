"""Fix 3: a record that appears on top of an already completed target is never auctioned again."""

from src.config import AllocationConfig
from src.tracking.target_state import TargetState
from tests.unit.test_tracker import obs, tracker


def test_on_completed_site_has_no_time_limit():
    tr = tracker()
    tr.ingest(obs(1.0, 100, 100), 1.0)
    done = tr.records["T-D1-001"]
    done.state, done.terminal_at = TargetState.COMPLETED, 10.0
    new = tr.ingest(obs(500.0, 105, 100), 500.0)  # long after ignore_after_complete: a new record is created
    assert new.kind == "created"
    rec = tr.records[new.target_id]
    assert tr.on_completed_site(rec)  # ... but it is recognised as the same, already logged target
    assert not tr.on_completed_site(done)


def test_other_places_are_not_completed_sites():
    tr = tracker()
    tr.ingest(obs(1.0, 100, 100), 1.0)
    tr.records["T-D1-001"].state = TargetState.COMPLETED
    far = tr.ingest(obs(2.0, 400, 400), 2.0)
    assert not tr.on_completed_site(tr.records[far.target_id])


def test_skip_completed_sites_key_defaults_on_and_can_be_switched_off():
    assert AllocationConfig().skip_completed_sites is True
    assert AllocationConfig(skip_completed_sites=False).skip_completed_sites is False


def test_cue_merged_into_a_sensor_record_keeps_the_record_allocatable():
    """A cue that arrives while a drone's own sensor already sees the target must not be lost when the two merge."""
    tr = tracker()
    tr.ingest(obs(1.0, 300, 700), 1.0)  # sensor record T-D1-001 (smaller ID than the cue's)
    tr.add_cue("T-EXT-001", (302, 699), 2.0)
    tr._merge_duplicates("T-EXT-001")
    rec = tr.records["T-D1-001"]
    assert "T-EXT-001" in rec.aliases and rec.is_cue  # allocatable as a cue, not timed out as an unconfirmed detection
