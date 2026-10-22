"""Matplotlib visualizations: static route maps and animated robot runs."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless-safe; plt.show() still works if you switch backends
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.colors import ListedColormap

from .algorithms import SearchResult
from .grid import Grid

# 0 free, 1 explored, 2 obstacle, 3 rough terrain
_CMAP = ListedColormap(["#f7f7f5", "#b9d4ee", "#2b2f36", "#e3d3b0"])
PATH_COLOR = "#d6453d"


def _base_image(grid: Grid) -> np.ndarray:
    img = np.zeros(grid.blocked.shape, dtype=int)
    if grid.weights is not None:
        img[grid.weights > 1] = 3
    img[grid.blocked] = 2
    return img


def _draw(ax, grid: Grid, result: SearchResult, show_explored: bool = True) -> None:
    img = _base_image(grid)
    if show_explored:
        for cell in result.explored_order:
            if not grid.blocked[cell]:
                img[cell] = 1
    ax.imshow(img, cmap=_CMAP, vmin=0, vmax=3, interpolation="nearest")
    if result.path:
        ys, xs = zip(*result.path)
        ax.plot(xs, ys, color=PATH_COLOR, lw=2.2, solid_capstyle="round")
    ax.scatter(*grid.start[::-1], s=90, c="#2e9d5b", marker="o", zorder=3, label="start")
    ax.scatter(*grid.goal[::-1], s=140, c="#e0a100", marker="*", zorder=3, label="goal")
    cost = f"{result.cost:.1f}" if result.found else "no path"
    ax.set_title(f"{result.algorithm}\ncost {cost} · expanded {result.nodes_expanded} · "
                 f"{result.runtime_ms:.1f} ms", fontsize=9)
    ax.set_xticks([])
    ax.set_yticks([])


def plot_result(grid: Grid, result: SearchResult, out: str | Path | None = None):
    fig, ax = plt.subplots(figsize=(6, 6))
    _draw(ax, grid, result)
    fig.tight_layout()
    if out:
        fig.savefig(out, dpi=130)
    return fig


def plot_comparison(grid: Grid, results: list[SearchResult],
                    out: str | Path | None = None, title: str = ""):
    """Side-by-side route maps, one panel per algorithm."""
    n = len(results)
    fig, axes = plt.subplots(1, n, figsize=(3.6 * n, 4.1))
    for ax, res in zip(np.atleast_1d(axes), results):
        _draw(ax, grid, res)
    if title:
        fig.suptitle(title, fontsize=12, y=0.99)
        fig.tight_layout(rect=(0, 0, 1, 0.93))
    else:
        fig.tight_layout()
    if out:
        fig.savefig(out, dpi=130)
    return fig


def animate_search(grid: Grid, result: SearchResult, out: str | Path,
                   max_frames: int = 160, fps: int = 20) -> Path:
    """GIF: the search frontier spreads, then the robot drives the route."""
    out = Path(out)
    img = _base_image(grid)
    explored = result.explored_order
    step = max(1, len(explored) // (max_frames * 2 // 3))
    search_frames = list(range(0, len(explored), step)) + [len(explored)]
    path_step = max(1, len(result.path) // (max_frames // 3)) if result.path else 1
    drive_frames = list(range(0, len(result.path), path_step)) + [len(result.path) - 1] \
        if result.path else []

    fig, ax = plt.subplots(figsize=(5, 5))
    ax.set_xticks([]); ax.set_yticks([])
    im = ax.imshow(img, cmap=_CMAP, vmin=0, vmax=3, interpolation="nearest")
    ax.scatter(*grid.goal[::-1], s=140, c="#e0a100", marker="*", zorder=3)
    line, = ax.plot([], [], color=PATH_COLOR, lw=2.2)
    robot = ax.scatter([grid.start[1]], [grid.start[0]], s=110, c="#2e9d5b",
                       edgecolors="white", linewidths=1.5, zorder=4)
    title = ax.set_title(result.algorithm, fontsize=10)

    def update(i):
        if i < len(search_frames):
            k = search_frames[i]
            frame = img.copy()
            for cell in explored[:k]:
                if not grid.blocked[cell]:
                    frame[cell] = 1
            im.set_data(frame)
            title.set_text(f"{result.algorithm}: searching… {k} expanded")
        else:
            j = drive_frames[i - len(search_frames)]
            ys, xs = zip(*result.path[: j + 1])
            line.set_data(xs, ys)
            robot.set_offsets([[xs[-1], ys[-1]]])
            title.set_text(f"{result.algorithm}: driving · cost {result.cost:.1f}")
        return im, line, robot, title

    total = len(search_frames) + len(drive_frames)
    anim = animation.FuncAnimation(fig, update, frames=total, blit=False)
    anim.save(out, writer=animation.PillowWriter(fps=fps))
    plt.close(fig)
    return out
