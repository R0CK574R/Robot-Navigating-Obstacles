"""Pathfinding algorithms. Each returns a SearchResult with path + metrics."""
from __future__ import annotations

import heapq
import time
from collections import deque
from dataclasses import dataclass, field
from itertools import count
from typing import Callable

from .grid import Cell, Grid


@dataclass
class SearchResult:
    algorithm: str
    path: list[Cell]               # empty if no route exists
    cost: float                    # total path cost (inf if not found)
    nodes_expanded: int            # cells popped from the frontier
    max_frontier: int              # peak frontier size (memory proxy)
    runtime_ms: float
    explored_order: list[Cell] = field(default_factory=list, repr=False)

    @property
    def found(self) -> bool:
        return bool(self.path)


def _reconstruct(came_from: dict[Cell, Cell | None], goal: Cell) -> list[Cell]:
    path, node = [], goal
    while node is not None:
        path.append(node)
        node = came_from[node]
    return path[::-1]


def bfs(grid: Grid) -> SearchResult:
    """Breadth-first search. Optimal only when every move costs the same."""
    t0 = time.perf_counter()
    frontier = deque([grid.start])
    came_from: dict[Cell, Cell | None] = {grid.start: None}
    explored, peak = [], 1
    while frontier:
        cur = frontier.popleft()
        explored.append(cur)
        if cur == grid.goal:
            break
        for nxt, _ in grid.neighbors(cur):
            if nxt not in came_from:
                came_from[nxt] = cur
                frontier.append(nxt)
        peak = max(peak, len(frontier))
    path = _reconstruct(came_from, grid.goal) if grid.goal in came_from else []
    return SearchResult("BFS", path, grid.path_cost(path) if path else float("inf"),
                        len(explored), peak, (time.perf_counter() - t0) * 1e3, explored)


def _best_first(grid: Grid, name: str,
                priority: Callable[[float, float], float]) -> SearchResult:
    """Shared engine for Dijkstra / A* / Greedy.

    priority(g, h) -> f decides the algorithm:
      Dijkstra: f = g      A*: f = g + h      Greedy: f = h
    """
    t0 = time.perf_counter()
    tie = count()  # breaks ties FIFO so heap never compares cells
    h0 = grid.heuristic(grid.start, grid.goal)
    frontier = [(priority(0.0, h0), next(tie), grid.start)]
    g = {grid.start: 0.0}
    came_from: dict[Cell, Cell | None] = {grid.start: None}
    closed: set[Cell] = set()
    explored, peak = [], 1
    while frontier:
        _, _, cur = heapq.heappop(frontier)
        if cur in closed:
            continue  # stale duplicate entry
        closed.add(cur)
        explored.append(cur)
        if cur == grid.goal:
            break
        for nxt, step in grid.neighbors(cur):
            new_g = g[cur] + step
            if nxt not in g or new_g < g[nxt]:
                g[nxt] = new_g
                came_from[nxt] = cur
                f = priority(new_g, grid.heuristic(nxt, grid.goal))
                heapq.heappush(frontier, (f, next(tie), nxt))
        peak = max(peak, len(frontier))
    found = grid.goal in closed
    path = _reconstruct(came_from, grid.goal) if found else []
    return SearchResult(name, path, grid.path_cost(path) if path else float("inf"),
                        len(explored), peak, (time.perf_counter() - t0) * 1e3, explored)


def dijkstra(grid: Grid) -> SearchResult:
    return _best_first(grid, "Dijkstra", lambda g, h: g)


def astar(grid: Grid) -> SearchResult:
    return _best_first(grid, "A*", lambda g, h: g + h)


def weighted_astar(grid: Grid, w: float = 2.0) -> SearchResult:
    """Inflated heuristic: faster, path cost at most w × optimal."""
    return _best_first(grid, f"Weighted A* (w={w:g})", lambda g, h: g + w * h)


def greedy(grid: Grid) -> SearchResult:
    """Greedy best-first: chases the goal, fast but not optimal."""
    return _best_first(grid, "Greedy", lambda g, h: h)


ALGORITHMS: dict[str, Callable[[Grid], SearchResult]] = {
    "BFS": bfs,
    "Dijkstra": dijkstra,
    "A*": astar,
    "Weighted A*": weighted_astar,
    "Greedy": greedy,
}
