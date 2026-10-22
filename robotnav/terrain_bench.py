"""Measure planners on the off-road course as conditions get harder.

Three experiments, each varying one factor and holding the rest at the
defaults (hilliness 10 m, grade limit 0.6, normal obstacle density):

  hilliness  — taller hills mean steeper slopes and more detours
  max_grade  — a weaker robot can climb less, so more terrain is off-limits
  obstacles  — more rocks and trees
"""
from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from .algorithms import ALGORITHMS
from .terrain import course_solvable, offroad_course, run_course

DEFAULTS = dict(size=64, hilliness=10.0, max_grade=0.6, rock_density=0.035, tree_density=0.03)
EXPERIMENTS = {
    "hilliness": [("4 m", dict(hilliness=4)), ("8 m", dict(hilliness=8)),
                  ("12 m", dict(hilliness=12)), ("16 m", dict(hilliness=16)),
                  ("20 m", dict(hilliness=20))],
    "max_grade": [("100%", dict(max_grade=1.0)), ("80%", dict(max_grade=0.8)),
                  ("65%", dict(max_grade=0.65)), ("55%", dict(max_grade=0.55)),
                  ("45%", dict(max_grade=0.45))],
    "obstacles": [("sparse", dict(rock_density=0.01, tree_density=0.01)),
                  ("normal", dict(rock_density=0.035, tree_density=0.03)),
                  ("dense", dict(rock_density=0.07, tree_density=0.06)),
                  ("very dense", dict(rock_density=0.11, tree_density=0.09))],
}


def run_terrain_benchmark(trials: int = 8, algorithms=None) -> pd.DataFrame:
    algorithms = algorithms or ALGORITHMS
    rows = []
    for factor, levels in EXPERIMENTS.items():
        for order, (level, override) in enumerate(levels):
            for trial in range(trials):
                t = offroad_course(seed=500 + trial, **{**DEFAULTS, **override})
                drivable = course_solvable(t)
                results = {n: run_course(t, fn) for n, fn in algorithms.items()} if drivable else {}
                best = results["Dijkstra"].cost if drivable else math.nan
                for name in algorithms:
                    r = results.get(name)
                    st = t.path_stats(r.path) if r and r.found else {}
                    rows.append(dict(
                        factor=factor, level=level, order=order, trial=trial, algorithm=name,
                        success=bool(r and r.found),
                        time=r.cost if r and r.found else math.nan,
                        time_ratio=r.cost / best if r and r.found else math.nan,
                        distance_m=st.get("distance_m", math.nan),
                        climb_m=st.get("climb_m", math.nan),
                        rough_m=st.get("rough_m", math.nan),
                        nodes=r.nodes_expanded if r else math.nan,
                        ms=r.runtime_ms if r else math.nan,
                    ))
    return pd.DataFrame(rows)


_COLORS = {"BFS": "#7a869a", "Dijkstra": "#2f6fb0", "A*": "#e0921f",
           "Weighted A*": "#b64fae", "Greedy": "#d6453d"}
_TITLES = {"hilliness": "Hill height", "max_grade": "Steepest drivable slope",
           "obstacles": "Rocks & trees"}


def plot_terrain_benchmark(df: pd.DataFrame, outdir: str | Path) -> list[Path]:
    """One row per experiment: drivable rate, drive time vs optimal, cells searched."""
    outdir = Path(outdir)
    fig, axes = plt.subplots(3, 3, figsize=(14, 11))
    for row, factor in enumerate(EXPERIMENTS):
        part = df[df.factor == factor]
        levels = part.drop_duplicates("level").sort_values("order")["level"].tolist()
        x = range(len(levels))

        sr = part[part.algorithm == "Dijkstra"].groupby("level", sort=False).success.mean().reindex(levels)
        ax = axes[row, 0]
        ax.bar(x, sr.values * 100, color="#8a9a6a")
        ax.set_ylim(0, 105); ax.set_ylabel("% of courses drivable")

        for col, (metric, label, logy) in enumerate([("time_ratio", "Drive time ÷ optimal", False),
                                                     ("nodes", "Cells searched", True)], start=1):
            ax2 = axes[row, col]
            for alg, g in part[part.success].groupby("algorithm"):
                s = g.groupby("level")[metric].mean().reindex(levels)
                ax2.plot(list(x), s.values, marker="o", lw=2, color=_COLORS.get(alg), label=alg)
            if logy:
                ax2.set_yscale("log")
            ax2.set_ylabel(label)
        for col in range(3):
            a = axes[row, col]
            a.set_xticks(list(x)); a.set_xticklabels(levels)
            a.set_title(f"{_TITLES[factor]}", fontsize=10, loc="left")
            a.grid(alpha=0.3, axis="y")
    handles, labels = axes[0, 1].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=len(labels), frameon=False, fontsize=10)
    fig.tight_layout(rect=(0, 0.035, 1, 1))
    p = outdir / "terrain_benchmark.png"
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return [p]
