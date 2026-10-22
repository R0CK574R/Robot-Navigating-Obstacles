"""Grid world the robot navigates: free cells, obstacles, start and goal."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterator

import numpy as np

Cell = tuple[int, int]  # (row, col)

SQRT2 = math.sqrt(2)

FOUR = [(-1, 0), (1, 0), (0, -1), (0, 1)]
EIGHT = FOUR + [(-1, -1), (-1, 1), (1, -1), (1, 1)]


@dataclass
class Grid:
    """2-D occupancy grid. True in `blocked` means an obstacle.

    `weights` (optional) gives a per-cell traversal cost >= 1, used to model
    terrain like mud or sand. Moving into a cell costs step_length * weight.
    """

    blocked: np.ndarray
    start: Cell
    goal: Cell
    diagonal: bool = False
    weights: np.ndarray | None = None
    _moves: list = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._moves = EIGHT if self.diagonal else FOUR
        for name, c in (("start", self.start), ("goal", self.goal)):
            if not self.in_bounds(c):
                raise ValueError(f"{name} {c} is outside the grid")
            if self.blocked[c]:
                raise ValueError(f"{name} {c} is on an obstacle")

    @property
    def rows(self) -> int:
        return self.blocked.shape[0]

    @property
    def cols(self) -> int:
        return self.blocked.shape[1]

    def in_bounds(self, c: Cell) -> bool:
        return 0 <= c[0] < self.rows and 0 <= c[1] < self.cols

    def passable(self, c: Cell) -> bool:
        return self.in_bounds(c) and not self.blocked[c]

    def neighbors(self, c: Cell) -> Iterator[tuple[Cell, float]]:
        """Yield (neighbor, move_cost). Diagonals can't cut obstacle corners."""
        r, col = c
        for dr, dc in self._moves:
            n = (r + dr, col + dc)
            if not self.passable(n):
                continue
            if dr and dc and (self.blocked[r + dr, col] or self.blocked[r, col + dc]):
                continue  # no squeezing between two diagonal obstacles
            step = SQRT2 if (dr and dc) else 1.0
            w = 1.0 if self.weights is None else float(self.weights[n])
            yield n, step * w

    def heuristic(self, a: Cell, b: Cell) -> float:
        """Admissible distance estimate matching the movement model."""
        dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
        if self.diagonal:  # octile distance
            return (dr + dc) + (SQRT2 - 2) * min(dr, dc)
        return dr + dc  # Manhattan

    def path_cost(self, path: list[Cell]) -> float:
        total = 0.0
        for a, b in zip(path, path[1:]):
            step = SQRT2 if (a[0] != b[0] and a[1] != b[1]) else 1.0
            w = 1.0 if self.weights is None else float(self.weights[b])
            total += step * w
        return total


# ---------------------------------------------------------------- generators

def random_grid(
    rows: int = 40,
    cols: int = 40,
    density: float = 0.25,
    diagonal: bool = False,
    seed: int | None = None,
    terrain: bool = False,
) -> Grid:
    """Uniform random obstacles. Start = top-left, goal = bottom-right."""
    rng = np.random.default_rng(seed)
    blocked = rng.random((rows, cols)) < density
    start, goal = (0, 0), (rows - 1, cols - 1)
    blocked[start] = blocked[goal] = False
    weights = None
    if terrain:
        weights = np.ones((rows, cols))
        weights[rng.random((rows, cols)) < 0.2] = 5.0  # 20% "rough" cells
    return Grid(blocked, start, goal, diagonal, weights)


def maze_grid(rows: int = 41, cols: int = 41, diagonal: bool = False,
              seed: int | None = None) -> Grid:
    """Perfect maze via randomized DFS. Odd dimensions work best."""
    rng = np.random.default_rng(seed)
    rows, cols = rows | 1, cols | 1
    blocked = np.ones((rows, cols), dtype=bool)
    stack = [(1, 1)]
    blocked[1, 1] = False
    while stack:
        r, c = stack[-1]
        options = [(r + dr, c + dc, dr // 2, dc // 2)
                   for dr, dc in ((-2, 0), (2, 0), (0, -2), (0, 2))
                   if 0 < r + dr < rows - 1 and 0 < c + dc < cols - 1
                   and blocked[r + dr, c + dc]]
        if not options:
            stack.pop()
            continue
        nr, nc, wr, wc = options[rng.integers(len(options))]
        blocked[r + wr, c + wc] = blocked[nr, nc] = False
        stack.append((nr, nc))
    return Grid(blocked, (1, 1), (rows - 2, cols - 2), diagonal)


def rooms_grid(rows: int = 40, cols: int = 40, diagonal: bool = False,
               seed: int | None = None) -> Grid:
    """Warehouse-like layout: walls with doorways, plus scattered crates."""
    rng = np.random.default_rng(seed)
    blocked = rng.random((rows, cols)) < 0.05          # scattered crates
    row_walls = list(range(rows // 4, rows - 1, rows // 4))
    col_walls = list(range(cols // 4, cols - 1, cols // 4))
    for r in row_walls:
        blocked[r, :] = True
    for c in col_walls:
        blocked[:, c] = True

    def segments(walls, n):
        edges = [-1] + walls + [n]
        return [(a + 1, b) for a, b in zip(edges, edges[1:]) if b - a > 2]

    # one 2-wide doorway in every wall segment, carved after walls and crates,
    # so every room connects to its neighbours
    for r in row_walls:
        for lo, hi in segments(col_walls, cols):
            d = int(rng.integers(lo, hi - 1))
            blocked[r, d:d + 2] = False
            blocked[r - 1, d:d + 2] = blocked[r + 1, d:d + 2] = False  # clear approach
    for c in col_walls:
        for lo, hi in segments(row_walls, rows):
            d = int(rng.integers(lo, hi - 1))
            blocked[d:d + 2, c] = False
            blocked[d:d + 2, c - 1] = blocked[d:d + 2, c + 1] = False
    start, goal = (0, 0), (rows - 1, cols - 1)
    blocked[:2, :2] = blocked[-2:, -2:] = False  # docking pads stay clear
    return Grid(blocked, start, goal, diagonal)


def is_reachable(grid: Grid) -> bool:
    """Flood fill from start; cheap check before visualizing a map."""
    seen, stack = {grid.start}, [grid.start]
    while stack:
        cur = stack.pop()
        if cur == grid.goal:
            return True
        for n, _ in grid.neighbors(cur):
            if n not in seen:
                seen.add(n)
                stack.append(n)
    return False


def solvable(generator, *args, seed: int = 0, max_tries: int = 200, **kwargs) -> Grid:
    """Call generator with seed, seed+1, ... until the goal is reachable."""
    for s in range(seed, seed + max_tries):
        g = generator(*args, seed=s, **kwargs)
        if is_reachable(g):
            return g
    raise RuntimeError("no solvable map found; lower the obstacle density")


def from_ascii(text: str, diagonal: bool = False) -> Grid:
    """Build a grid from a picture: '#' wall, '.' free, 'S' start, 'G' goal."""
    lines = [ln for ln in text.strip("\n").splitlines()]
    blocked = np.array([[ch == "#" for ch in ln] for ln in lines])
    start = goal = None
    for r, ln in enumerate(lines):
        for c, ch in enumerate(ln):
            if ch == "S":
                start = (r, c)
            elif ch == "G":
                goal = (r, c)
    if start is None or goal is None:
        raise ValueError("map needs one 'S' and one 'G'")
    return Grid(blocked, start, goal, diagonal)
