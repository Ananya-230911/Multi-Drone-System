"""Compare the drones' decentralized auction with other ways of choosing the drone: python -m src.compare

Every allocation in every scenario is scored with the SAME cost function (travel time, energy,
remaining battery; lower is better). For the same situation the script asks what each method would
have picked and reports the cost of that pick, how far it is from the best possible pick (regret),
and how often the pick is a drone that cannot finish the job (battery below the reserve or no route).

    ours         the drones' own auction (what actually happened in the run)
    optimum      centralized solver that sees everything (upper bound; evaluation only)
    nearest      the geometrically closest drone, ignoring battery and altitude
    battery      the drone with the most battery, ignoring distance
    random       a random drone (mean over 200 draws)

Outputs `output/compare/`: compare.csv (every allocation), summary.md and plots. Simulation only.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import random
import statistics
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from src.config import load_config
from src.evaluate import CONFIG_DIR, write_csv
from src.sim.world import World

METHODS = ["ours", "optimum", "nearest", "battery", "random"]
LABEL = {"ours": "Ours (decentralized auction)", "optimum": "Centralized optimum", "nearest": "Nearest drone",
         "battery": "Most battery", "random": "Random drone"}  # fmt: skip
COLOUR = {"ours": "#46a0ff", "optimum": "#3cb371", "nearest": "#ffb347", "battery": "#8e6bd8", "random": "#9aa5b8"}


def allocations(name: str, seed: int) -> list[dict]:
    cfg = load_config(CONFIG_DIR / f"{name}.yaml")
    cfg = dataclasses.replace(cfg, sim=dataclasses.replace(cfg.sim, seed=seed, headless=True))
    w = World(cfg)
    w.run()
    starts = {(s["t"], k[1]): s for k, s in w.allocation_starts.items()}
    out = []
    for r in w.baseline_results():
        st = starts.get((r["t"], r["epoch"]))
        bids = r["baseline_bids"]
        feasible = {d: -b[1] for d, b in bids.items() if b[0] and b[1] is not None}
        if st is None or not feasible or not r["committed"]:
            continue
        live = [d for d in st["drones"] if d.mode != "FAILED"]
        tx, ty = st["target_est"]
        picks = {
            "ours": sorted(r["committed"].values())[0],
            "optimum": r["baseline_winner"],
            "nearest": min(live, key=lambda d: (math.hypot(d.x - tx, d.y - ty), d.id)).id,
            "battery": max(live, key=lambda d: (d.battery_wh, d.id)).id,
        }  # fmt: skip
        best = min(feasible.values())
        row = {"scenario": name, "seed": seed, "epoch": r["epoch"], "drones": len(live), "feasible_drones": len(feasible),
               "best_cost": round(best, 2)}  # fmt: skip
        for m, d in picks.items():
            row[f"{m}_pick"] = d
            row[f"{m}_cost"] = round(feasible[d], 2) if d in feasible else None
        rng = random.Random(f"{name}{seed}{r['epoch']}")
        draws = [rng.choice(live).id for _ in range(200)]
        ok = [feasible[d] for d in draws if d in feasible]
        row["random_pick"] = "-"
        row["random_cost"] = round(statistics.fmean(ok), 2) if ok else None
        row["random_feasible_rate"] = round(len(ok) / len(draws), 3)
        out.append(row)
    return out


def run_job(job: tuple[str, int]) -> list[dict]:
    return allocations(*job)


def summarize(rows: list[dict]) -> list[dict]:
    out = []
    for m in METHODS:
        feas = [r[f"{m}_cost"] is not None for r in rows]
        rate = (statistics.fmean(r["random_feasible_rate"] for r in rows) if m == "random"
                else statistics.fmean(feas))  # fmt: skip
        regrets = [r[f"{m}_cost"] - r["best_cost"] for r in rows if r[f"{m}_cost"] is not None]
        costs = [r[f"{m}_cost"] for r in rows if r[f"{m}_cost"] is not None]
        optimal = statistics.fmean(
            (r[f"{m}_cost"] is not None and r[f"{m}_cost"] - r["best_cost"] < 1e-6) for r in rows) if m != "random" else None
        out.append({"method": m, "label": LABEL[m], "allocations": len(rows), "feasible_pick_pct": round(100 * rate, 1),
                    "optimal_pick_pct": None if optimal is None else round(100 * optimal, 1),
                    "mean_cost_when_feasible": round(statistics.fmean(costs), 2) if costs else None,
                    "mean_regret": round(statistics.fmean(regrets), 2) if regrets else None})  # fmt: skip
    return out


def markdown(summary: list[dict]) -> str:
    h = "| method | allocations | picks a drone that can finish (%) | picks the best drone (%) | mean extra cost vs best (feasible picks only) |\n|---|---|---|---|---|\n"
    return h + "".join(f"| {r['label']} | {r['allocations']} | {r['feasible_pick_pct']} | {r['optimal_pick_pct'] if r['optimal_pick_pct'] is not None else '-'} "
                       f"| {r['mean_regret']} |\n" for r in summary)  # fmt: skip


def plots(summary: list[dict], rows: list[dict], out: Path) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out.mkdir(parents=True, exist_ok=True)
    paths = []

    def style(ax, title, ylabel):
        ax.set_title(title, fontsize=12, loc="left")
        ax.set_ylabel(ylabel)
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="y", alpha=0.25)

    names = [LABEL[r["method"]].replace(" (", "\n(").replace("Centralized optimum", "Centralized\noptimum") for r in summary]
    cols = [COLOUR[r["method"]] for r in summary]
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.4))
    for ax, key, title, yl in [(axs[0], "feasible_pick_pct", "Picks a drone that can finish the job", "% of allocations"),
                               (axs[1], "optimal_pick_pct", "Picks the best drone", "% of allocations"),
                               (axs[2], "mean_regret", "Extra cost vs the best drone (lower is better)", "cost units")]:  # fmt: skip
        vals = [r[key] or 0 for r in summary]
        bars = ax.bar(range(len(vals)), vals, color=cols)
        ax.set_xticks(range(len(vals)), names, fontsize=7.5)
        for b, v in zip(bars, vals):
            na = summary[len(ax.patches) - 1 - list(bars).index(b) if False else list(bars).index(b)][key] is None
            ax.text(b.get_x() + b.get_width() / 2, v, "n/a" if na else f"{v:g}", ha="center", va="bottom", fontsize=8)
        style(ax, title, yl)
    fig.tight_layout(); p = out / "methods.png"; fig.savefig(p, dpi=140); plt.close(fig); paths.append(p)

    scen = sorted({r["scenario"] for r in rows})
    fig, ax = plt.subplots(figsize=(12, 4.6))
    w = 0.16
    for k, m in enumerate(METHODS):
        ys = []
        for s in scen:
            c = [r[f"{m}_cost"] for r in rows if r["scenario"] == s and r[f"{m}_cost"] is not None]
            ys.append(statistics.fmean(c) if c else 0)
        ax.bar([i + (k - 2) * w for i in range(len(scen))], ys, w, label=LABEL[m], color=COLOUR[m])
    ax.set_xticks(range(len(scen)), scen, rotation=30, ha="right")
    ax.legend(fontsize=8, frameon=False, ncols=3)
    style(ax, "Cost of the chosen drone per scenario (lower is better; infeasible picks are not drawn)", "cost units")
    fig.tight_layout(); p = out / "cost_per_scenario.png"; fig.savefig(p, dpi=140); plt.close(fig); paths.append(p)
    return paths


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Compare allocation methods on the same scenarios")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--output", default="output/compare")
    a = ap.parse_args(argv)
    names = sorted(p.stem for p in CONFIG_DIR.glob("*.yaml") if p.stem not in {"false_observation", "target_in_restricted", "partition"})
    jobs = [(n, s) for n in names for s in range(1, a.seeds + 1)]
    with ProcessPoolExecutor(a.workers) as ex:
        rows = [r for part in ex.map(run_job, jobs) for r in part]
    if not rows:
        print("no allocations found", file=sys.stderr)
        return 1
    summary = summarize(rows)
    out = Path(a.output)
    write_csv(out / "compare.csv", rows)
    write_csv(out / "summary.csv", summary)
    table = markdown(summary)
    (out / "summary.md").write_text(table, encoding="utf-8")
    print(table)
    for p in plots(summary, rows, out):
        print("wrote", p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
