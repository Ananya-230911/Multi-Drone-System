"""Target log (PLAN.md section 13.2).

Each agent writes to its own logbook (the owner when it logs a target, or when
a cue it went to verify is rejected). At the end of a run the logbooks are
merged by target ID with aliases resolved, together with every agent's final
view of each record. A row reaches final_state=COMPLETED only through the
TRACKING -> LOGGED -> COMPLETED path. If two owners logged the same target
(possible after a network partition), the earliest is the primary entry and the
others are marked duplicate_inspection.
"""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
from pathlib import Path

COLUMNS = [
    "target_id", "aliases", "source", "detected_at", "detected_by", "confirmed_at", "allocated_at", "owner", "epoch",
    "inspection_start", "logged_at", "inspection_duration", "mean_confidence", "est_x", "est_y", "est_alt",
    "final_state", "cancel_reason", "reallocations", "messages_used", "duplicate_inspection",
]  # fmt: skip


@dataclass
class LogEntry:
    target_id: str
    aliases: str
    source: str
    detected_at: float
    detected_by: str
    confirmed_at: float | None
    allocated_at: float | None
    owner: str | None
    epoch: int
    inspection_start: float | None
    logged_at: float | None
    inspection_duration: float | None
    mean_confidence: float | None
    est_x: float
    est_y: float
    est_alt: float | None
    final_state: str
    cancel_reason: str | None
    reallocations: int
    written_by: str

    @classmethod
    def from_record(cls, rec, written_by: str, inspection_start=None, logged_at=None, inspection_duration=None,
                    mean_confidence=None) -> "LogEntry":  # fmt: skip
        allocated = next((h.t for h in rec.history if h.to.value == "ALLOCATED"), None)
        return cls(
            target_id=rec.id, aliases=";".join(sorted(rec.aliases)), source=rec.source,
            detected_at=rec.created_at, detected_by=rec.created_by, confirmed_at=rec.confirmed_at,
            allocated_at=allocated, owner=rec.owner, epoch=rec.epoch, inspection_start=inspection_start,
            logged_at=logged_at, inspection_duration=inspection_duration, mean_confidence=mean_confidence,
            est_x=rec.est_pos[0], est_y=rec.est_pos[1], est_alt=rec.est_alt,
            final_state="COMPLETED" if logged_at is not None else rec.state.value,
            cancel_reason=rec.cancel_reason, reallocations=rec.reallocations, written_by=written_by,
        )  # fmt: skip

    def as_dict(self) -> dict:
        return asdict(self)


def merge(logbooks: dict[str, list[LogEntry]], final_views: dict[str, list[LogEntry]],
          alias_of: dict[str, str], messages_used: dict[str, int]) -> list[dict]:  # fmt: skip
    """One row per target (plus duplicate-inspection rows). Logged entries win over plain final views."""
    def key(tid: str) -> str:
        while tid in alias_of and alias_of[tid] != tid:
            tid = alias_of[tid]
        return tid

    logged: dict[str, list[LogEntry]] = {}
    for _, entries in sorted(logbooks.items()):
        for e in entries:
            logged.setdefault(key(e.target_id), []).append(e)
    views: dict[str, list[LogEntry]] = {}
    for _, entries in sorted(final_views.items()):
        for e in entries:
            views.setdefault(key(e.target_id), []).append(e)

    rows = []
    for tid in sorted(set(logged) | set(views)):
        done = sorted([e for e in logged.get(tid, []) if e.final_state == "COMPLETED"], key=lambda e: e.logged_at)
        others = logged.get(tid, []) + views.get(tid, [])
        aliases = sorted({a for e in others for a in e.aliases.split(";") if a and a != tid} | ({e.target_id for e in others} - {tid}))
        detected = min(others, key=lambda e: e.detected_at)
        if done:
            chosen = [(done[0], False)] + [(e, True) for e in done[1:]]
        else:
            # No logged inspection: the most advanced view; ties -> the owner's own view, then lowest writer ID.
            rank = {"CANCELLED": 9, "COMPLETED": 9, "LOGGED": 8, "REACQUIRING": 7, "TRACKING_LOST": 6, "TRACKING": 5,
                    "ENROUTE": 4, "ALLOCATED": 3, "CONFIRMED": 2, "DETECTED": 1}  # fmt: skip
            best = max(others, key=lambda e: (rank.get(e.final_state, 0), e.written_by == e.owner, [-ord(c) for c in e.written_by]))
            chosen = [(best, False)]
        for e, dup in chosen:
            row = {c: getattr(e, c, None) for c in COLUMNS if hasattr(e, c)}
            row.update(target_id=tid, aliases=";".join(aliases), detected_at=detected.detected_at,
                       detected_by=detected.detected_by, messages_used=messages_used.get(tid, 0),
                       duplicate_inspection=dup)  # fmt: skip
            if e.final_state != "COMPLETED":
                row.update(inspection_start=None, logged_at=None, inspection_duration=None, mean_confidence=None)
            rows.append(row)
    return rows


def write_csv(rows: list[dict], path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow({c: ("" if r.get(c) is None else (round(r[c], 3) if isinstance(r[c], float) else r[c])) for c in COLUMNS})
    return path
