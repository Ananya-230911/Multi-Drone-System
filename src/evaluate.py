"""Batch evaluation: python -m src.evaluate [--seeds 10] [--scenarios a b] [--output output/eval]

Runs every scenario headless over N seeds, checks each scenario's expectations, and writes
`metrics.csv` (one row per run), `summary.csv` / `summary.md` (mean per scenario) and plots to the
output directory. Exit code is 1 if any expectation fails. Simulation only.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import statistics
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any, Callable

from src.config import load_config
from src.sim.world import World

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
FALSE_CONFIRM_RADIUS = 100.0  # a confirmed target farther than this from every real one is a false confirmation


# ---------------------------------------------------------------------------------- metrics


def _count(w: World, etype: str, pred: Callable[[dict], bool] = lambda r: True) -> int:
    return sum(1 for r in w.log.of_type(etype) if pred(r))


def measure(w: World) -> dict[str, Any]:
    rows = w.target_log_rows()
    done = [r for r in rows if r["final_state"] == "COMPLETED"]
    main = min(done, key=lambda r: r["logged_at"]) if done else None
    base = w.baseline_results()
    committed = [r for r in w.log.of_type("ALLOCATION_COMMITTED")]
    rounds = {(r["target_id"], r["epoch"]) for r in committed}
    failed = [r["t"] for r in w.log.of_type("DRONE_FAILED")]
    t_realloc = None
    if failed:
        later = [r["t"] for r in committed if (r.get("epoch") or 0) > 0 and r["t"] >= failed[0]]
        t_realloc = min(later) - failed[0] if later else None
    confirmed = [r for r in rows if r["confirmed_at"] is not None]
    gts = w.ground_truth.targets
    false_conf = [r for r in confirmed
                  if not any(math.hypot(r["est_x"] - g.pos[0], r["est_y"] - g.pos[1]) <= FALSE_CONFIRM_RADIUS for g in gts)]  # fmt: skip
    lost, reacq = _count(w, "TRACK_LOST"), _count(w, "REACQUIRE_SUCCESS")
    g = lambda k: None if main is None else main[k]  # noqa: E731
    return {
        "completed": len(done), "cancelled": sum(r["final_state"] == "CANCELLED" for r in rows), "targets": len(rows),
        "owner": g("owner"),
        "t_detect": min((r["detected_at"] for r in rows), default=None), "t_confirm": g("confirmed_at"),
        "t_allocate": g("allocated_at"), "t_first_track": g("inspection_start"), "t_log": g("logged_at"),
        "msgs_per_allocation": None if not rounds else round(w.network.stats["sent"] / len(rounds), 1),
        "messages_sent": w.network.stats["sent"],
        "baseline_agreement_pct": None if not base else round(100 * sum(b["match"] for b in base) / len(base), 1),
        "allocations": len(base),
        "t_realloc_after_failure": None if t_realloc is None else round(t_realloc, 2),
        "owner_timeouts": _count(w, "REALLOCATION", lambda r: r["data"].get("reason") == "owner_timeout"),
        "track_lost": lost, "reacquired": reacq,
        "false_confirmations": len(false_conf), "confirmed_targets": len(confirmed),
        "geofence_violations": len(w.geofence_violations), "separation_violations": len(w.separation_violations),
        "duplicate_inspections": sum(bool(r["duplicate_inspection"]) for r in rows),
        "duplicate_owner_seen": _count(w, "DUPLICATE_OWNER_SEEN"),
        "claim_conflicts_lost": _count(w, "CLAIM_CONFLICT", lambda r: r["data"].get("result") == "lost"),
        "neutralized": _count(w, "TARGET_NEUTRALIZED"),
    }  # fmt: skip


# ---------------------------------------------------------------------------------- expectations

Check = Callable[[dict], str | None]  # returns a problem description, or None


def _all(m):
    if m["geofence_violations"]:
        return f"{m['geofence_violations']} geofence violation(s)"
    if m["separation_violations"]:
        return f"{m['separation_violations']} separation violation(s)"
    return None


def _completes(n: int) -> Check:
    return lambda m: None if m["completed"] == n else f"expected {n} completed target(s), got {m['completed']}"


def _none_completed(m):
    return None if m["completed"] == 0 else "a target was logged as completed but should not have been"


def _no_dup(m):
    return None if m["duplicate_inspections"] == 0 else "duplicate inspection left unresolved"


def _failure(m):
    if m["owner_timeouts"] < 1:
        return "the owner's failure was never noticed (no owner_timeout)"
    if m["owner"] == "D1":
        return "the failed drone is recorded as the owner of the completed task"
    if m["t_realloc_after_failure"] is None:
        return "no reallocation after the failure"
    return None


def _partition(m):
    if m["duplicate_owner_seen"] < 1 or m["claim_conflicts_lost"] < 1:
        return "two owners were elected but the duplicate was not detected and resolved after the network healed"
    return None


EXPECT: dict[str, list[Check]] = {
    "default": [_completes(1)], "mission": [_completes(1)], "geofence_detour": [_completes(1)],
    "low_battery": [_completes(1)], "tracking_loss": [_completes(1)], "limited_comms": [lambda m: None if m["completed"] >= 1 else "never completed"],
    "duplicate_reports": [lambda m: None if m["completed"] >= 1 else "never completed"],
    "external_cue": [_completes(1)],  # the cue at an empty place must never be logged
    "false_observation": [_none_completed, lambda m: None if m["false_confirmations"] == 0 else "a spurious observation was confirmed"],
    "target_in_restricted": [_none_completed],
    "drone_failure": [_completes(1), _failure, _no_dup],
    "partition": [_completes(1), _partition, _no_dup],
}  # fmt: skip


def check(name: str, m: dict) -> list[str]:
    checks = [_all] + EXPECT.get(name, [])
    return [p for c in checks if (p := c(m))]


# ---------------------------------------------------------------------------------- running


def run_one(args: tuple[str, int]) -> dict[str, Any]:
    name, seed = args
    cfg = load_config(CONFIG_DIR / f"{name}.yaml")
    cfg = dataclasses.replace(cfg, sim=dataclasses.replace(cfg.sim, seed=seed, headless=True))
    w = World(cfg)
    w.run()
    m = measure(w)
    problems = check(name, m)
    return {"scenario": name, "seed": seed, **m, "ok": not problems, "problems": "; ".join(problems)}


def evaluate(names: list[str], seeds: int, workers: int = 1) -> list[dict]:
    jobs = [(n, s) for n in names for s in range(1, seeds + 1)]
    if workers > 1:
        with ProcessPoolExecutor(workers) as ex:
            return list(ex.map(run_one, jobs))
    return [run_one(j) for j in jobs]


NUMERIC = ["t_detect", "t_confirm", "t_allocate", "t_first_track", "t_log", "msgs_per_allocation", "baseline_agreement_pct",
           "t_realloc_after_failure", "track_lost", "reacquired", "false_confirmations", "geofence_violations",
           "separation_violations", "duplicate_inspections", "completed"]  # fmt: skip


def summarize(runs: list[dict]) -> list[dict]:
    out = []
    for name in dict.fromkeys(r["scenario"] for r in runs):
        rs = [r for r in runs if r["scenario"] == name]
        row: dict[str, Any] = {"scenario": name, "runs": len(rs), "passed": sum(r["ok"] for r in rs)}
        for k in NUMERIC:
            vals = [r[k] for r in rs if r[k] is not None]
            row[k] = round(statistics.fmean(vals), 2) if vals else None
        lost = sum(r["track_lost"] for r in rs)
        row["reacquire_rate_pct"] = round(100 * sum(r["reacquired"] for r in rs) / lost, 1) if lost else None
        conf = sum(r["confirmed_targets"] for r in rs)
        row["false_confirm_rate_pct"] = round(100 * sum(r["false_confirmations"] for r in rs) / conf, 1) if conf else None
        out.append(row)
    return out


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with path.open("w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=keys)
        wr.writeheader()
        wr.writerows(rows)


def markdown(summary: list[dict]) -> str:
    cols = [("scenario", "scenario"), ("passed", "pass"), ("t_detect", "detect s"), ("t_confirm", "confirm s"),
            ("t_allocate", "allocate s"), ("t_first_track", "track s"), ("t_log", "log s"),
            ("msgs_per_allocation", "msgs/alloc"), ("baseline_agreement_pct", "agree %"),
            ("t_realloc_after_failure", "realloc s"), ("reacquire_rate_pct", "reacq %"),
            ("false_confirm_rate_pct", "false conf %"), ("geofence_violations", "geofence viol."),
            ("separation_violations", "sep. viol."), ("duplicate_inspections", "dup. insp.")]  # fmt: skip
    head = "| " + " | ".join(h for _, h in cols) + " |\n|" + "---|" * len(cols) + "\n"
    body = ""
    for r in summary:
        cells = []
        for k, _ in cols:
            v = r.get(k)
            cells.append(f"{r['passed']}/{r['runs']}" if k == "passed" else ("-" if v is None else str(v)))
        body += "| " + " | ".join(cells) + " |\n"
    return head + body


# ---------------------------------------------------------------------------------- plots


def plots(summary: list[dict], runs: list[dict], out: Path) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out.mkdir(parents=True, exist_ok=True)
    paths = []
    names = [r["scenario"] for r in summary]
    c = {"detect": "#9aa5b8", "confirm": "#46a0ff", "allocate": "#ffb347", "track": "#3cb371", "log": "#8e6bd8"}

    def style(ax, title, xlabel=None, ylabel=None):
        ax.set_title(title, fontsize=12, loc="left")
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="x" if ylabel is None else "y", alpha=0.25)
        if xlabel: ax.set_xlabel(xlabel)
        if ylabel: ax.set_ylabel(ylabel)

    # 1. timeline: where the time goes from detection to the log entry
    rows = [r for r in summary if r["t_log"] is not None]
    fig, ax = plt.subplots(figsize=(9, 0.5 * len(rows) + 1.6))
    for i, r in enumerate(rows):
        marks = [("detect", r["t_detect"]), ("confirm", r["t_confirm"]), ("allocate", r["t_allocate"]),
                 ("track", r["t_first_track"]), ("log", r["t_log"])]  # fmt: skip
        prev = 0.0
        for key, t in marks:
            if t is None:
                continue
            ax.barh(i, max(t - prev, 0), left=prev, color=c[key], label=key if i == 0 else None, height=0.6)
            prev = max(prev, t)
    ax.set_yticks(range(len(rows)), [r["scenario"] for r in rows])
    ax.invert_yaxis()
    style(ax, "Time from start to each milestone (mean over seeds)", "simulated seconds")
    ax.legend(ncols=5, fontsize=8, frameon=False, loc="lower right")
    fig.tight_layout(); p = out / "timeline.png"; fig.savefig(p, dpi=140); plt.close(fig); paths.append(p)

    # 2. messages per allocation
    rows = [r for r in summary if r["msgs_per_allocation"] is not None]
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.bar([r["scenario"] for r in rows], [r["msgs_per_allocation"] for r in rows], color="#46a0ff")
    ax.tick_params(axis="x", rotation=35)
    style(ax, "Radio messages sent per allocation round", ylabel="messages")
    fig.tight_layout(); p = out / "messages.png"; fig.savefig(p, dpi=140); plt.close(fig); paths.append(p)

    # 3. agreement with the centralized optimum
    rows = [r for r in summary if r["baseline_agreement_pct"] is not None]
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.bar([r["scenario"] for r in rows], [r["baseline_agreement_pct"] for r in rows], color="#3cb371")
    ax.set_ylim(0, 105); ax.tick_params(axis="x", rotation=35)
    style(ax, "Decentralized winner = centralized optimum (evaluation only)", ylabel="% of allocations")
    fig.tight_layout(); p = out / "agreement.png"; fig.savefig(p, dpi=140); plt.close(fig); paths.append(p)

    # 4. robustness: reallocation time and safety counters
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4), gridspec_kw={"width_ratios": [1, 2]})
    fr = [r for r in summary if r["t_realloc_after_failure"] is not None]
    a1.bar([r["scenario"] for r in fr] or ["-"], [r["t_realloc_after_failure"] for r in fr] or [0], color="#ffb347")
    style(a1, "Reallocation after owner failure", ylabel="seconds")
    x = range(len(names)); w = 0.27
    for k, (key, lab, col) in enumerate([("geofence_violations", "geofence violations", "#ff5a5a"),
                                         ("separation_violations", "separation violations", "#ffb347"),
                                         ("duplicate_inspections", "duplicate inspections", "#8e6bd8")]):  # fmt: skip
        a2.bar([i + (k - 1) * w for i in x], [r[key] or 0 for r in summary], w, label=lab, color=col)
    a2.set_xticks(list(x), names, rotation=35, ha="right"); a2.legend(fontsize=8, frameon=False)
    style(a2, "Safety counters (violations must be 0; duplicates are marked)", ylabel="count (mean per run)")
    fig.tight_layout(); p = out / "robustness.png"; fig.savefig(p, dpi=140); plt.close(fig); paths.append(p)
    return paths


# ---------------------------------------------------------------------------------- main


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Run every scenario over N seeds; metrics table and plots")
    ap.add_argument("--seeds", type=int, default=10, help="seeds 1..N per scenario (default 10)")
    ap.add_argument("--scenarios", nargs="*", help="scenario names (default: all in config/)")
    ap.add_argument("--output", default="output/eval")
    ap.add_argument("--workers", type=int, default=1, help="parallel processes (default 1)")
    ap.add_argument("--no-plots", action="store_true")
    a = ap.parse_args(argv)
    names = a.scenarios or sorted(p.stem for p in CONFIG_DIR.glob("*.yaml"))
    unknown = [n for n in names if not (CONFIG_DIR / f"{n}.yaml").is_file()]
    if unknown:
        print(f"unknown scenario(s): {', '.join(unknown)}", file=sys.stderr)
        return 2
    runs = evaluate(names, a.seeds, a.workers)
    summary = summarize(runs)
    out = Path(a.output)
    write_csv(out / "metrics.csv", runs)
    write_csv(out / "summary.csv", summary)
    table = markdown(summary)
    (out / "summary.md").write_text(table, encoding="utf-8")
    print(table)
    if not a.no_plots:
        for p in plots(summary, runs, out):
            print("wrote", p)
    bad = [r for r in runs if not r["ok"]]
    for r in bad:
        print(f"FAIL {r['scenario']} seed {r['seed']}: {r['problems']}", file=sys.stderr)
    print(f"\n{len(runs) - len(bad)}/{len(runs)} runs met their expectations ({len(names)} scenarios x {a.seeds} seeds)")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
