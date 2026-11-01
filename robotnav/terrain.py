"""Off-road 3D terrain: hills, surfaces, rocks, trees, water and themed maps.

The robot drives on a height map. Each cell has an elevation (meters), a
surface type, and maybe an obstacle. Every move costs *drive time* in seconds:
the rover cruises at 1 m/s on the fastest surface and slows down on rough
surfaces and climbs, so

    time = 3-D distance × surface factor × (1 + uphill_penalty × grade⁺)

(the 3-D distance is sqrt(run² + rise²), not hypot, so Python, the C++ core and the browser round it alike)

and any move steeper than `max_grade` is impossible (the treads would slip).
Every surface factor is at least 1, so a move never takes less time than its
flat distance; the octile distance stays an admissible heuristic and A* is
still optimal (it finds the quickest route).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace

import numpy as np
from scipy.ndimage import zoom

from .algorithms import SearchResult
from .grid import EIGHT, FOUR, Cell, Grid, is_reachable

SQRT2 = math.sqrt(2)


def add_up(values) -> float:
    """Add drive times left to right, the way the C++ core and the browser add them, so all three agree to the
    last bit (Python's own sum() has compensated for rounding since 3.12, so its last digit can differ)."""
    total = 0.0
    for v in values:
        total += v
    return total


# surface types
GRASS, MUD, GRAVEL = 0, 1, 2
SURFACE_COST = {GRASS: 1.0, MUD: 3.0, GRAVEL: 1.4}
# obstacle types
NONE, ROCK, TREE, WATER = 0, 1, 2, 3


@dataclass
class Terrain(Grid):
    """A Grid whose moves are priced by slope, distance and surface."""

    height: np.ndarray = None          # meters, shape (rows, cols)
    surface: np.ndarray = None         # GRASS / MUD / GRAVEL
    obstacle: np.ndarray = None        # NONE / ROCK / TREE
    cell_size: float = 1.0             # meters per cell
    max_grade: float = 0.6             # rise/run limit (~31°)
    uphill_penalty: float = 3.0        # extra time per unit of uphill grade
    surface_costs: tuple = (SURFACE_COST[GRASS], SURFACE_COST[MUD], SURFACE_COST[GRAVEL])
    surface_names: tuple = ("Grass", "Mud", "Gravel")
    sea_level: float | None = None     # island maps: cells below this are water
    ponds: list = field(default_factory=list)  # desert oases: [{"level": m, "cells": [(r, c), ...]}], all WATER
    checkpoints: list = field(default_factory=list)
    meta: dict = field(default_factory=dict)   # generation parameters, enough to rebuild the course

    def edge_cost(self, a: Cell, b: Cell) -> float | None:
        """Seconds to drive a -> b, or None if the slope is too steep."""
        diag = a[0] != b[0] and a[1] != b[1]
        run = (SQRT2 if diag else 1.0) * self.cell_size
        rise = float(self.height[b] - self.height[a])
        grade = rise / run
        if abs(grade) > self.max_grade:
            return None
        dist3d = math.sqrt(run * run + rise * rise)    # not hypot: rounds the same in Python, C++ and JavaScript
        surf = self.surface_costs[int(self.surface[b])]
        return dist3d * surf * (1.0 + self.uphill_penalty * max(grade, 0.0))

    def neighbors(self, c: Cell):
        r, col = c
        for dr, dc in self._moves:
            n = (r + dr, col + dc)
            if not self.passable(n):
                continue
            if dr and dc and (self.blocked[r + dr, col] or self.blocked[r, col + dc]):
                continue
            cost = self.edge_cost(c, n)
            if cost is not None:
                yield n, cost

    def heuristic(self, a: Cell, b: Cell) -> float:
        return super().heuristic(a, b) * self.cell_size

    def path_cost(self, path: list[Cell]) -> float:
        return add_up(self.edge_cost(a, b) for a, b in zip(path, path[1:]))

    def path_stats(self, path: list[Cell]) -> dict:
        """Physical stats a robotics engineer would care about."""
        if len(path) < 2:
            return dict(distance_m=0.0, climb_m=0.0, descent_m=0.0, max_grade_pct=0.0,
                        rough_m=0.0)
        dist = climb = descent = rough = 0.0
        max_grade = 0.0
        for a, b in zip(path, path[1:]):
            run = (SQRT2 if a[0] != b[0] and a[1] != b[1] else 1.0) * self.cell_size
            rise = float(self.height[b] - self.height[a])
            seg = math.sqrt(run * run + rise * rise)
            dist += seg
            climb += max(rise, 0.0)
            descent += max(-rise, 0.0)
            max_grade = max(max_grade, abs(rise) / run)
            if self.surface_costs[int(self.surface[b])] > 1.0:    # any surface slower than the best
                rough += seg
        return dict(distance_m=round(dist, 1), climb_m=round(climb, 1),
                    descent_m=round(descent, 1), max_grade_pct=round(100 * max_grade, 1),
                    rough_m=round(rough, 1))


# ------------------------------------------------------------------ generation

def _fractal_noise(rows: int, cols: int, rng, octaves: int = 5,
                   base: int = 4, persistence: float = 0.5) -> np.ndarray:
    """Sum of smoothly upsampled random grids (value noise), in [0, 1]."""
    out = np.zeros((rows, cols))
    amp, total = 1.0, 0.0
    for o in range(octaves):
        n = base * 2 ** o
        coarse = rng.random((n + 1, n + 1))
        layer = zoom(coarse, ((rows) / (n + 1), (cols) / (n + 1)), order=3)
        out += amp * layer[:rows, :cols]
        total += amp
        amp *= persistence
    out /= total
    return (out - out.min()) / (np.ptp(out) + 1e-9)


def offroad_course(
    size: int = 96,
    hilliness: float = 14.0,
    rock_density: float = 0.035,
    tree_density: float = 0.03,
    max_grade: float = 0.6,
    n_checkpoints: int = 2,
    seed: int | None = None,
) -> Terrain:
    """Generate one hilly off-road course with start, checkpoints and finish.

    hilliness  — elevation range in meters (higher = steeper hills)
    max_grade  — steepest slope the robot can drive (0.6 = 60% ≈ 31°)
    """
    rng = np.random.default_rng(seed)
    h = _fractal_noise(size, size, rng)
    h = h ** 1.4 * hilliness                       # flatter valleys, sharper peaks

    # mud collects in the low basins
    surface = np.full((size, size), GRASS, dtype=np.int8)
    low = h < np.quantile(h, 0.14)
    surface[low] = MUD
    gravel = (_fractal_noise(size, size, rng, octaves=3, base=6) > 0.72) & ~low
    surface[gravel] = GRAVEL

    # rocks in clusters, trees scattered on gentler ground
    obstacle = np.zeros((size, size), dtype=np.int8)
    n_clusters = int(rock_density * size * size / 6)
    for _ in range(n_clusters):
        r, c = rng.integers(0, size, 2)
        for _ in range(rng.integers(2, 9)):
            rr, cc = r + rng.integers(-2, 3), c + rng.integers(-2, 3)
            if 0 <= rr < size and 0 <= cc < size:
                obstacle[rr, cc] = ROCK
    trees = (rng.random((size, size)) < tree_density) & (obstacle == NONE) & (surface != MUD)
    obstacle[trees] = TREE

    # course waypoints run diagonally across the map with some wobble
    pad = max(3, size // 16)
    raw = [(size - 1 - pad, pad)]
    for i in range(1, n_checkpoints + 1):
        t = i / (n_checkpoints + 1)
        # wobble sideways (perpendicular to the diagonal) so legs stay spread out
        j = int(rng.integers(-size // 4, size // 4 + 1))
        raw.append((int(np.clip(size - 1 - pad - t * (size - 2 * pad) - j, pad, size - pad - 1)),
                    int(np.clip(pad + t * (size - 2 * pad) - j, pad, size - pad - 1))))
    raw.append((pad, size - 1 - pad))

    # clear a small pad around each waypoint so flags aren't inside a rock
    for r, c in raw:
        obstacle[max(0, r - 2):r + 3, max(0, c - 2):c + 3] = NONE

    blocked = obstacle != NONE
    meta = dict(size=size, hilliness=hilliness, rock_density=rock_density, tree_density=tree_density,
                max_grade=max_grade, n_checkpoints=n_checkpoints, seed=seed)
    return Terrain(blocked=blocked, start=raw[0], goal=raw[-1], diagonal=True,
                   height=h, surface=surface, obstacle=obstacle,
                   max_grade=max_grade, checkpoints=raw[1:-1], meta=meta)


def waypoints(t: Terrain) -> list[Cell]:
    return [t.start, *t.checkpoints, t.goal]


def course_solvable(t: Terrain) -> bool:
    pts = waypoints(t)
    return all(is_reachable(replace(t, start=a, goal=b)) for a, b in zip(pts, pts[1:]))


def solvable_course(seed: int = 0, max_tries: int = 100, **kw) -> Terrain:
    for s in range(seed, seed + max_tries):
        t = offroad_course(seed=s, **kw)
        if course_solvable(t):
            return t
    raise RuntimeError("no drivable course found — lower hilliness or obstacle density")


def evaluate_route(t: Terrain, path) -> dict:
    """Check and score a hand-made route with exactly the planners' rules.

    A valid route begins at the start, moves one cell at a time (8 directions),
    never drives into a rock or tree, never squeezes diagonally between two
    obstacles, never exceeds the grade limit, and passes every checkpoint in
    order before ending at the finish.

    Returns {"valid", "reason", "time", **path_stats}.
    """
    path = [(int(r), int(c)) for r, c in path]
    out = dict(valid=False, reason=None, time=math.inf)
    if len(path) < 2 or path[0] != t.start:
        out["reason"] = "the route must begin at the start"
        return out
    for i, (a, b) in enumerate(zip(path, path[1:]), start=1):
        dr, dc = b[0] - a[0], b[1] - a[1]
        if max(abs(dr), abs(dc)) != 1:
            out["reason"] = f"step {i} jumps from {a} to {b}"
        elif not t.passable(b):
            out["reason"] = f"step {i} drives into an obstacle at {b}"
        elif dr and dc and (t.blocked[a[0] + dr, a[1]] or t.blocked[a[0], a[1] + dc]):
            out["reason"] = f"step {i} squeezes between two obstacles at {a}"
        elif t.edge_cost(a, b) is None:
            out["reason"] = f"step {i} is steeper than the {t.max_grade:.0%} grade limit"
        if out["reason"]:
            return out
    wps = waypoints(t)
    nxt = 1                                   # index of the next waypoint to reach
    for cell in path:
        if nxt < len(wps) and cell == wps[nxt]:
            nxt += 1
    if nxt < len(wps):
        out["reason"] = "the route never reaches " + ("the finish" if nxt == len(wps) - 1 else f"checkpoint {nxt}")
        return out
    if path[-1] != t.goal:
        out["reason"] = "the route must end at the finish"
        return out
    out.update(valid=True, time=t.path_cost(path), **t.path_stats(path))
    return out


def run_course(t: Terrain, algorithm) -> SearchResult:
    """Plan start → checkpoint(s) → finish, one leg at a time."""
    pts = waypoints(t)
    path, explored = [t.start], []
    nodes = peak = 0
    t0 = time.perf_counter()
    name = None
    for a, b in zip(pts, pts[1:]):
        leg = algorithm(replace(t, start=a, goal=b))
        name = leg.algorithm
        nodes += leg.nodes_expanded
        peak = max(peak, leg.max_frontier)
        explored += leg.explored_order
        if not leg.found:
            return SearchResult(name, [], math.inf, nodes, peak,
                                (time.perf_counter() - t0) * 1e3, explored)
        path += leg.path[1:]
    return SearchResult(name, path, t.path_cost(path), nodes, peak,
                        (time.perf_counter() - t0) * 1e3, explored)


# ------------------------------------------------------------------ themed maps for the 3D viewer

THEMES = ("grassland", "desert", "island")


def _scatter_rocks(obstacle, rng, density, ok=None):
    """Rock clusters of 2-8 stones; `ok` limits where stones may land."""
    size = obstacle.shape[0]
    for _ in range(int(density * size * size / 6)):
        r, c = rng.integers(0, size, 2)
        for _ in range(rng.integers(2, 9)):
            rr, cc = r + rng.integers(-2, 3), c + rng.integers(-2, 3)
            if 0 <= rr < size and 0 <= cc < size and obstacle[rr, cc] == NONE and (ok is None or ok[rr, cc]):
                obstacle[rr, cc] = ROCK


def _grassland(size, rng):
    """Rolling green hills: short grass, patches of slower tall grass, trees and rocks."""
    h = _fractal_noise(size, size, rng) ** 1.3 * 12.0
    surface = (_fractal_noise(size, size, rng, octaves=3, base=5) > 0.62).astype(np.int8)
    obstacle = np.zeros((size, size), dtype=np.int8)
    _scatter_rocks(obstacle, rng, 0.02)
    obstacle[(rng.random((size, size)) < 0.035) & (obstacle == NONE)] = TREE
    return h, surface, obstacle, (1.0, 1.8), ("Grass", "Tall grass"), None, []


def _oases(h, surface, obstacle, rng, wind):
    """Desert oases: two or three ponds in the lowest broad ground between the dunes.

    Each pond is a basin stretched along the dune troughs (like a real interdune lake), carved into
    a gently levelled apron so its shores stay drivable, and filled to just below its rim: the pond
    is every cell below the water level that connects to the basin floor, so it never spills. Round
    it lie firm wet sand, a patchy ring of oasis grass, and palms. Changes the arrays in place and
    returns the ponds as [{"level": metres, "cells": [(r, c), ...]}].
    """
    from scipy.ndimage import distance_transform_edt, label, uniform_filter
    size = h.shape[0]
    k = size / 96                                                     # distances scale with the map
    yy, xx = np.mgrid[0:size, 0:size].astype(float)
    ok = np.zeros((size, size), bool)
    m = int(13 * k)
    ok[m:size - m, m:size - m] = True                                 # away from the map edges
    for r, c in _endpoints("desert", h, obstacle, None):              # and from the default flags
        ok &= np.hypot(yy - r, xx - c) > 24 * k
    broad = uniform_filter(h, int(9 * k) | 1, mode="nearest")        # broad low ground
    cand = np.argwhere(ok)
    sites = []
    for j in np.argsort(broad[ok] + rng.uniform(0, 0.6, len(cand)), kind="stable"):
        cy, cx = (int(v) for v in cand[j])
        if all(math.hypot(cy - p[0], cx - p[1]) >= 30 * k for p in sites):
            sites.append((cy, cx))
        if len(sites) == (3 if size >= 64 else 2):
            break

    trough = (-math.sin(wind), math.cos(wind))                        # (x, y) along the dune troughs
    ponds, water = [], np.zeros((size, size), bool)
    for cy, cx in sites:
        a = rng.uniform(9.5, 12.5) * k                                # half-length and half-width
        b = a * rng.uniform(0.5, 0.65)
        tilt = rng.uniform(-0.35, 0.35)
        ux = trough[0] * math.cos(tilt) - trough[1] * math.sin(tilt)
        uy = trough[0] * math.sin(tilt) + trough[1] * math.cos(tilt)
        along = (xx - cx) * ux + (yy - cy) * uy
        across = -(xx - cx) * uy + (yy - cy) * ux
        ang = np.arctan2(across / b, along / a)
        wobble = sum(rng.uniform(0.04, 0.1) * np.sin(f * ang + rng.uniform(0, 2 * np.pi)) for f in (2, 3, 5))
        d = np.hypot(along / a, across / b) / (1 + wobble)            # 1 on the basin's rim
        hc = h.copy()
        floor = np.percentile(hc[d < 1.2], 30)
        s = np.clip(1 - (d - 1) * b / 8.0, 0, 1)                     # level an apron that fades out over ~8 m
        w = 0.9 * s * s * (3 - 2 * s)
        hc = hc * (1 - w) + np.minimum(hc, floor + 0.6) * w
        hc -= rng.uniform(1.4, 1.9) * np.clip(1 - d * d, 0, None) ** 1.5    # the basin
        bottom = np.unravel_index(np.argmin(np.where(d < 0.5, hc, np.inf)), hc.shape)
        level = hc[(d > 0.95) & (d < 1.3)].min() - 0.1               # just below the lowest point of the rim
        pond = None
        while level > hc[bottom] + 0.6:                               # at least 0.6 m deep
            lab, _ = label(hc < level, structure=np.ones((3, 3)))
            region = lab == lab[bottom]
            if lab[bottom] and not (region & (d > 1.25)).any():      # stays inside its basin
                pond = region
                break
            level -= 0.05
        if pond is None or pond.sum() < 12 * k * k:
            continue
        h[:] = hc
        water |= pond
        ponds.append(dict(level=float(level), cells=[(int(r), int(c)) for r, c in np.argwhere(pond)]))

    if ponds:
        dist = distance_transform_edt(~water)                         # metres to the nearest pond
        patchy = _fractal_noise(size, size, rng, octaves=3, base=6)
        grass = ~water & (dist > 1.5) & (dist <= 2.0 + 2.5 * patchy)
        obstacle[water] = WATER
        obstacle[~water & (dist <= 1.5)] = NONE                      # a clean shoreline
        obstacle[grass] = NONE
        surface[water | (dist <= 1.5)] = 0                            # firm wet sand (and the pond beds)
        surface[grass] = 3                                            # oasis grass
        gy, gx = np.gradient(h)
        spots = np.argwhere(grass & (dist >= 2.0) & (dist <= 3.6) & (np.hypot(gx, gy) < 0.35))
        palms = []
        for j in rng.permutation(len(spots))[: max(6, len(spots) // 4)]:
            r, c = (int(v) for v in spots[j])
            if all(math.hypot(r - p[0], c - p[1]) >= 2.3 for p in palms):
                palms.append((r, c))
                obstacle[r, c] = TREE                                 # palms (drawn as palms near the water)
    return ponds


def _desert(size, rng):
    """Dune field: gentle windward slopes, steep slip faces, soft sand on the crests, rocky flats, oasis ponds."""
    yy, xx = np.mgrid[0:size, 0:size].astype(float)
    swell = _fractal_noise(size, size, rng, octaves=4, base=3) * 4.0
    wind = rng.uniform(0, np.pi)
    u = (xx * np.cos(wind) + yy * np.sin(wind)) / 18.0 + _fractal_noise(size, size, rng, octaves=3, base=4) * 1.5
    phase = u - np.floor(u)
    dune = np.where(phase < 0.75, phase / 0.75, (1.0 - phase) / 0.25) ** 1.5
    amp = 4.5 * np.clip(_fractal_noise(size, size, rng, octaves=3, base=3) * 1.4 - 0.2, 0.05, 1.0)
    h = swell + dune * amp
    rocky = swell / 4.0 > 0.68
    surface = np.zeros((size, size), dtype=np.int8)               # 0 firm sand
    surface[(dune > 0.45) & (amp > 1.5) & ~rocky] = 1              # 1 soft sand
    surface[rocky] = 2                                             # 2 rocky ground
    obstacle = np.zeros((size, size), dtype=np.int8)
    _scatter_rocks(obstacle, rng, 0.035, ok=rocky | (rng.random((size, size)) < 0.25))
    obstacle[(rng.random((size, size)) < 0.012) & (obstacle == NONE) & (surface == 0)] = TREE   # cacti
    # oases last, from their own random stream, so the dunes, rocks and cacti stay as they were
    ponds = _oases(h, surface, obstacle, np.random.default_rng(int(rng.integers(1 << 62))), wind)
    return (h, surface, obstacle, (1.0, 2.4, 1.3, 1.0), ("Firm sand", "Soft sand", "Rocky ground", "Oasis grass"),
            None, ponds)


def _island(size, rng):
    """Tropical island: ragged coast, sandy beach, grass and jungle, a central peak; the sea is impassable."""
    yy, xx = np.mgrid[0:size, 0:size].astype(float)
    cy, cx = size / 2 + rng.uniform(-3, 3, 2)
    rad = np.hypot((xx - cx) / (size * 0.5), (yy - cy) / (size * 0.44))
    rad += (_fractal_noise(size, size, rng, octaves=4, base=4) - 0.5) * 0.45
    m = np.clip((1.0 - rad) / 0.3, 0.0, 1.0)
    m = m * m * (3 - 2 * m)                                         # 0 at sea → 1 inland
    peak = 6.0 * np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * (size * 0.12) ** 2))
    h = m * (_fractal_noise(size, size, rng) ** 1.3 * 8.0 + peak + 1.6) - (1 - m) * 2.5
    sea = 0.6
    water = h < sea
    beach = ~water & (h < sea + 0.9)
    jungle = ~water & ~beach & (_fractal_noise(size, size, rng, octaves=3, base=5) > 0.6)
    surface = np.zeros((size, size), dtype=np.int8)               # 0 grass
    surface[beach | water] = 1                                    # 1 sand (also the sea floor)
    surface[jungle] = 2                                           # 2 jungle
    obstacle = np.zeros((size, size), dtype=np.int8)
    _scatter_rocks(obstacle, rng, 0.015, ok=~water)
    obstacle[(rng.random((size, size)) < 0.03) & (obstacle == NONE) & ~water] = TREE           # palms
    obstacle[water] = WATER
    return h, surface, obstacle, (1.0, 1.6, 2.0), ("Grass", "Sand", "Jungle"), sea, []


def _endpoints(theme, h, obstacle, sea):
    """Default start and destination: opposite corners, or the island's west and east ends."""
    size = h.shape[0]
    pad = max(3, size // 16)
    if theme != "island":
        return (size - 1 - pad, pad), (pad, size - 1 - pad)
    rr, cc = np.nonzero((obstacle != WATER) & (h > sea + 1.0))
    off = 0.5 * np.abs(rr - size / 2)
    i, j = int(np.argmin(cc + off)), int(np.argmax(cc - off))
    return (int(rr[i]), int(cc[i])), (int(rr[j]), int(cc[j]))


def make_map(theme: str = "grassland", size: int = 96, seed: int | None = None,
             max_grade: float = 0.6) -> Terrain:
    """One of the viewer's three maps: "grassland", "desert" or "island"."""
    if theme not in THEMES:
        raise ValueError(f"theme must be one of {THEMES}")
    rng = np.random.default_rng(seed)
    gen = {"grassland": _grassland, "desert": _desert, "island": _island}[theme]
    h, surface, obstacle, costs, names, sea, ponds = gen(size, rng)
    start, goal = _endpoints(theme, h, obstacle, sea)
    for r, c in (start, goal):                 # keep a clear pad around the default flags
        pad = obstacle[max(0, r - 2):r + 3, max(0, c - 2):c + 3]
        pad[pad != WATER] = NONE
    return Terrain(blocked=obstacle != NONE, start=start, goal=goal, diagonal=True,
                   height=h, surface=surface, obstacle=obstacle, max_grade=max_grade,
                   surface_costs=costs, surface_names=names, sea_level=sea, ponds=ponds,
                   meta=dict(theme=theme, size=size, seed=seed, max_grade=max_grade))


def solvable_map(theme: str, seed: int = 0, max_tries: int = 100, **kw) -> Terrain:
    """make_map with the first seed (from `seed` up) whose default start can reach its destination."""
    for s in range(seed, seed + max_tries):
        t = make_map(theme, seed=s, **kw)
        if is_reachable(t):
            return t
    raise RuntimeError(f"no drivable {theme} map found")


def plan_pair(t: Terrain, start: Cell, goal: Cell, algorithms=None) -> list[SearchResult]:
    """Run every planner from `start` to `goal` (what the viewer does after two clicks)."""
    from .algorithms import ALGORITHMS
    leg = replace(t, start=tuple(start), goal=tuple(goal), checkpoints=[])
    return [run_course(leg, fn) for fn in (algorithms or ALGORITHMS).values()]


def _region(cell, size) -> str:
    """Compass name of where a cell sits on the map: "NW", "E", "Center", ..."""
    r, c = cell
    ns = "N" if r < size / 3 else "S" if r >= 2 * size / 3 else ""
    we = "W" if c < size / 3 else "E" if c >= 2 * size / 3 else ""
    return ns + we or "Center"


# ------------------------------------------------------------------ the shortest route: what a robot that ignores hills drives

SHORTEST_TIE = 1e-6   # a hair of drive time added to each move, so among equally short routes it takes the quickest


class _HillBlind:
    """The terrain as a robot that ignores hills sees it: the same legal moves (obstacles, water and the
    grade limit still stop it), but each move is priced by its flat distance only."""

    def __init__(self, t: Terrain, start: Cell, goal: Cell):
        self.t, self.start, self.goal = t, start, goal

    def neighbors(self, c: Cell):
        for n, step in self.t.neighbors(c):
            run = (SQRT2 if n[0] != c[0] and n[1] != c[1] else 1.0) * self.t.cell_size
            yield n, run + SHORTEST_TIE * step

    def heuristic(self, a: Cell, b: Cell) -> float:
        return 0.0

    def path_cost(self, path: list[Cell]) -> float:         # what the route really takes: drive time
        return self.t.path_cost(path)


def shortest_route(t: Terrain, start: Cell | None = None, goal: Cell | None = None) -> SearchResult:
    """The shortest drivable route (Dijkstra on flat distance), as a robot that ignores hills would
    choose it. Its `cost` is the drive time that route really takes on this terrain, so it can be
    compared with the planners. The browser runs the same search, so both pick the same cells."""
    from .algorithms import _best_first
    s = tuple(t.start if start is None else start)
    g = tuple(t.goal if goal is None else goal)
    return _best_first(_HillBlind(t, s, g), "Shortest route", lambda gc, h: gc)


# ------------------------------------------------------------------ preset routes: long, crossing, shaped by the hills

def _move_graphs(t: Terrain):
    """Every legal move as two sparse graphs (drive time; flat distance + a hair of time) for fast screening."""
    from scipy.sparse import csr_matrix
    R, C = t.rows, t.cols
    H, B = t.height, t.blocked
    surf = np.asarray(t.surface_costs, dtype=float)[t.surface]
    rr, cc = np.mgrid[0:R, 0:C]
    src, dst, drive, flat = [], [], [], []
    for dr, dc in EIGHT:
        r2, c2 = rr + dr, cc + dc
        inside = (r2 >= 0) & (r2 < R) & (c2 >= 0) & (c2 < C)
        r2, c2 = np.clip(r2, 0, R - 1), np.clip(c2, 0, C - 1)
        ok = inside & ~B & ~B[r2, c2]
        if dr and dc:                                       # no squeezing diagonally between two obstacles
            ok &= ~B[np.clip(rr + dr, 0, R - 1), cc] & ~B[rr, np.clip(cc + dc, 0, C - 1)]
        run = (SQRT2 if dr and dc else 1.0) * t.cell_size
        rise = H[r2, c2] - H
        grade = rise / run
        ok &= np.abs(grade) <= t.max_grade
        secs = np.sqrt(run * run + rise * rise) * surf[r2, c2] * (1.0 + t.uphill_penalty * np.maximum(grade, 0.0))
        src.append((rr * C + cc)[ok]); dst.append((r2 * C + c2)[ok])
        drive.append(secs[ok]); flat.append(run + SHORTEST_TIE * secs[ok])
    src, dst, N = np.concatenate(src), np.concatenate(dst), R * C
    return (csr_matrix((np.concatenate(drive), (src, dst)), shape=(N, N)),
            csr_matrix((np.concatenate(flat), (src, dst)), shape=(N, N)))


def _tree_times(t: Terrain, pred: np.ndarray, root: int) -> np.ndarray:
    """Drive time from `root` to every cell along a shortest-path tree (pointer jumping, vectorised)."""
    C, N = t.cols, pred.size
    v = np.arange(N)
    has = pred >= 0
    u = np.where(has, pred, root)
    dr, dc = v // C - u // C, v % C - u % C
    run = np.where((dr != 0) & (dc != 0), SQRT2, 1.0) * t.cell_size
    Hf, S = t.height.ravel(), np.asarray(t.surface_costs, dtype=float)[t.surface.ravel()]
    rise = Hf[v] - Hf[u]
    secs = np.sqrt(run * run + rise * rise) * S[v] * (1.0 + t.uphill_penalty * np.maximum(rise / run, 0.0))
    val = np.where(has, secs, 0.0)
    val[root] = 0.0
    ptr = u.copy(); ptr[root] = root
    for _ in range(int(math.ceil(math.log2(N))) + 1):
        val, ptr = val + val[ptr], ptr[ptr]
    return val


def preset_routes(t: Terrain, n: int = 20) -> list[dict]:
    """`n` start → destination routes that cross the map and show off hill-aware planning, quickest first.

    Every route is long (at least about half the land's width), follows mostly new ground (it shares
    little with the routes picked before it), and its quickest path is clearly faster than the
    shortest route a hill-blind robot would drive and visibly splits from it (the two lines run
    metres apart), so the hills shape it. Candidates are screened on a lattice with whole-map
    Dijkstra, then every pick is re-planned exactly with A* and `shortest_route` (the same searches
    the browser runs). Deterministic: no randomness.

    Each item: {"name", "start", "goal", "time", "distance_m", "path", "short_time", "saving"}.
    """
    from scipy.ndimage import binary_dilation, distance_transform_edt
    from scipy.sparse.csgraph import dijkstra as all_dijkstra
    from .algorithms import astar

    R, C, margin = t.rows, t.cols, 3
    inner = np.zeros((R, C), bool)
    inner[margin:R - margin, margin:C - margin] = True
    open_inner = np.argwhere(~t.blocked & inner)
    if len(open_inner) < 2:
        return []
    extent = int(min(np.ptp(open_inner[:, 0]), np.ptp(open_inner[:, 1]))) + 1     # land width (island < map)
    step = max(2, int(round(math.sqrt(len(open_inner) / 500))))                     # ~500 lattice cells
    lattice = np.array([(r, c) for r in range(margin, R - margin, step) for c in range(margin, C - margin, step)
                        if not t.blocked[r, c]])
    li = lattice[:, 0] * C + lattice[:, 1]

    # screen every lattice pair at once: quickest time, and the time the shortest route really takes
    G_drive, G_flat = _move_graphs(t)
    quick, pred = all_dijkstra(G_drive, directed=True, indices=li, return_predecessors=True)
    flat_d, flat_pred = all_dijkstra(G_flat, directed=True, indices=li, return_predecessors=True)
    quick = quick[:, li]
    short = np.vstack([np.where(np.isfinite(flat_d[k]), _tree_times(t, flat_pred[k], s), np.inf)[li]
                       for k, s in enumerate(li)])
    with np.errstate(divide="ignore", invalid="ignore"):
        saving = np.where(np.isfinite(quick) & np.isfinite(short) & (short > 0), 1.0 - quick / short, -1.0)
    length = np.hypot(lattice[:, None, 0] - lattice[None, :, 0], lattice[:, None, 1] - lattice[None, :, 1])

    def cells_of(tree, k, g):
        out, i = [], li[g]
        while i >= 0 and i != li[k]:
            out.append(i); i = tree[k, i]
        return np.array([divmod(int(j), C) for j in [li[k]] + out[::-1]])

    def splits(path, other, min_sep, min_frac):
        """Does `path` visibly leave `other`: metres apart at its widest, and apart for much of the way?"""
        free = np.ones((R, C), bool)
        free[other[:, 0], other[:, 1]] = False
        d = distance_transform_edt(free)[path[:, 0], path[:, 1]] * t.cell_size
        return d.max() >= min_sep and (d > 2.5 * t.cell_size).mean() >= min_frac

    picked, used, ends, names = [], np.zeros((R, C), bool), [], set()
    scale = max(R, C) / 96                                                 # distances in cells scale with the map
    # strict first; loosen only if the land is too small to fit n such routes
    tiers = [(0.55, 0.12, 0.30, 7, True, 8, 0.5), (0.50, 0.10, 0.35, 6, True, 7, 0.45),
             (0.45, 0.08, 0.40, 5, False, 5, 0.35), (0.40, 0.06, 0.45, 4, False, 4, 0.25)]
    for min_len, min_save, max_overlap, gap, unique, min_sep, min_frac in tiers:
        gap, min_sep = gap * scale, min_sep * scale * t.cell_size
        ks, gs = np.nonzero((length >= min_len * extent) & (saving >= min_save))
        score = saving[ks, gs] * np.sqrt(length[ks, gs] / extent)          # big savings, favouring long crossings
        for j in np.lexsort((gs, ks, -score)):
            if len(picked) == n:
                break
            k, g = int(ks[j]), int(gs[j])
            a, b = (int(lattice[k][0]), int(lattice[k][1])), (int(lattice[g][0]), int(lattice[g][1]))
            if any(math.dist(a, e) < gap or math.dist(b, e) < gap for e in ends):
                continue
            name = f"{_region(a, R)} → {_region(b, R)}"
            if unique and name in names:
                continue
            rough = cells_of(pred, k, g)
            if used[rough[:, 0], rough[:, 1]].mean() > max_overlap:
                continue
            if not splits(rough, cells_of(flat_pred, k, g), min_sep, min_frac):
                continue
            leg = replace(t, start=a, goal=b, checkpoints=[])
            best, blind = astar(leg), shortest_route(t, a, b)                  # exact, as the browser plans it
            if not best.found or 1.0 - best.cost / blind.cost < min_save * 0.8:
                continue
            cells = np.array(best.path)
            overlap = float(used[cells[:, 0], cells[:, 1]].mean())
            if overlap > max_overlap or not splits(cells, np.array(blind.path), min_sep * 0.85, min_frac * 0.85):
                continue
            mask = np.zeros((R, C), bool); mask[cells[:, 0], cells[:, 1]] = True
            used |= binary_dilation(mask)
            ends += [a, b]; names.add(name)
            picked.append(dict(name=name, start=a, goal=b, time=best.cost,
                               distance_m=t.path_stats(best.path)["distance_m"], path=best.path,
                               short_time=blind.cost, saving=1.0 - best.cost / blind.cost))
        if len(picked) == n:
            break
    picked.sort(key=lambda d: d["time"])
    seen: dict[str, int] = {}
    for p in picked:                                                       # keep names unique
        seen[p["name"]] = seen.get(p["name"], 0) + 1
        if seen[p["name"]] > 1:
            p["name"] = f"{p['name']} ({seen[p['name']]})"
    return picked
