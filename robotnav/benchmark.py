"""Run every algorithm across many maps and conditions; collect metrics."""
from __future__ import annotations

import math
from itertools import product
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from .algorithms import ALGORITHMS
from .grid import maze_grid, random_grid, rooms_grid

MAP_TYPES = {"random": random_grid, "maze": maze_grid, "rooms": rooms_grid}


def run_benchmark(
    sizes=(20, 40, 80),
    densities=(0.1, 0.2, 0.3, 0.4),
    diagonals=(False, True),
    trials: int = 10,
    algorithms: dict | None = None,
) -> pd.DataFrame:
    """Random maps swept over size × obstacle density × movement model.

    Each (condition, trial) uses one seed, so every algorithm sees the
    exact same map — that's what makes the comparison fair.
    """
    algorithms = algorithms or ALGORITHMS
    rows = []
    for size, dens, diag, t in product(sizes, densities, diagonals, range(trials)):
        seed = hash((size, dens, t)) & 0xFFFF_FFFF
        grid = random_grid(size, size, dens, diagonal=diag, seed=seed)
        results = {name: fn(grid) for name, fn in algorithms.items()}
        optimal = results["Dijkstra"].cost if "Dijkstra" in results else math.nan
        for name, r in results.items():
            rows.append(dict(
                map="random", size=size, density=dens,
                moves="8-way" if diag else "4-way", trial=t, algorithm=name,
                found=r.found, cost=r.cost if r.found else math.nan,
                optimality=(r.cost / optimal) if r.found and optimal > 0 else math.nan,
                nodes_expanded=r.nodes_expanded, max_frontier=r.max_frontier,
                runtime_ms=r.runtime_ms, path_len=len(r.path),
            ))
    return pd.DataFrame(rows)


def run_map_type_benchmark(size: int = 41, trials: int = 10,
                           diagonal: bool = False) -> pd.DataFrame:
    rows = []
    for mtype, gen in MAP_TYPES.items():
        for t in range(trials):
            kw = dict(diagonal=diagonal, seed=1000 + t)
            grid = gen(size, size, 0.25, **kw) if mtype == "random" else gen(size, size, **kw)
            results = {n: fn(grid) for n, fn in ALGORITHMS.items()}
            opt = results["Dijkstra"].cost
            for n, r in results.items():
                rows.append(dict(map=mtype, trial=t, algorithm=n, found=r.found,
                                 nodes_expanded=r.nodes_expanded, runtime_ms=r.runtime_ms,
                                 optimality=r.cost / opt if r.found else math.nan))
    return pd.DataFrame(rows)


def summarize(df: pd.DataFrame, by=("algorithm",)) -> pd.DataFrame:
    return (df.groupby(list(by))
              .agg(success_rate=("found", "mean"),
                   avg_cost=("cost", "mean"),
                   optimality=("optimality", "mean"),
                   nodes_expanded=("nodes_expanded", "mean"),
                   runtime_ms=("runtime_ms", "mean"))
              .round(3))


# ----------------------------------------------------------------- charts

_COLORS = {"BFS": "#7a869a", "Dijkstra": "#2f6fb0", "A*": "#d6453d",
           "Weighted A*": "#e08a2e", "Greedy": "#3a9a6a"}


def plot_benchmark(df: pd.DataFrame, outdir: str | Path) -> list[Path]:
    """Line charts of each metric vs obstacle density and vs grid size."""
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    solved = df[df.found]
    saved = []
    metrics = [("nodes_expanded", "Nodes expanded", True),
               ("runtime_ms", "Runtime (ms)", True),
               ("optimality", "Path cost ÷ optimal", False)]

    for xvar, xlabel, subset in [
        ("density", "Obstacle density", solved[solved["size"] == solved["size"].max()]),
        ("size", "Grid size (N × N)", solved[solved.density == 0.2]),
    ]:
        fig, axes = plt.subplots(2, 3, figsize=(14, 7.5), sharex=True)
        for row, moves in enumerate(["4-way", "8-way"]):
            part = subset[subset.moves == moves]
            for ax, (m, label, logy) in zip(axes[row], metrics):
                for alg, g in part.groupby("algorithm"):
                    s = g.groupby(xvar)[m].mean()
                    ax.plot(s.index, s.values, marker="o", lw=2,
                            color=_COLORS.get(alg), label=alg)
                if logy:
                    ax.set_yscale("log")
                ax.set_title(f"{label} — {moves}", fontsize=10)
                ax.grid(alpha=0.3)
                if row == 1:
                    ax.set_xlabel(xlabel)
        axes[0, 0].legend(fontsize=8, frameon=False)
        fig.tight_layout()
        p = outdir / f"benchmark_vs_{xvar}.png"
        fig.savefig(p, dpi=130)
        plt.close(fig)
        saved.append(p)

    # success rate vs density (all algorithms agree — it's a property of the map)
    fig, ax = plt.subplots(figsize=(6, 4))
    sr = df[df.algorithm == "Dijkstra"].groupby(["density", "size"]).found.mean().unstack()
    for size in sr.columns:
        ax.plot(sr.index, sr[size], marker="o", lw=2, label=f"{size}×{size}")
    ax.set_xlabel("Obstacle density"); ax.set_ylabel("Fraction of maps with a route")
    ax.set_title("Reachability vs obstacle density"); ax.grid(alpha=0.3)
    ax.legend(frameon=False)
    fig.tight_layout()
    p = outdir / "reachability.png"
    fig.savefig(p, dpi=130); plt.close(fig); saved.append(p)
    return saved
