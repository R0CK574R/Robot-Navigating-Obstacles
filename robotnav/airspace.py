"""The air route: a drone flies a 3-D route over the map and keeps out of the enemy's sight.

The airspace is a stack of flight levels over the map's grid: cells 1 m across, levels `dz` metres
apart, the first a little above the lowest ground. A voxel is free if it clears the ground below it
(and the trees, rocks and water) by `clear` metres. Watchtowers stand on hilltops and see up to
`sight` metres, but only along a clear line of sight: the ground and trees between a tower and the
drone block the view, so a drone flying low behind a hill is hidden (terrain masking).

Moves go to any of the 26 neighbouring voxels, as long as every voxel the move passes beside is
free (no clipping a hillside or a treetop). A move costs its flight time at `speed`, plus a little
for every metre climbed; the stealth route also pays `exposure` seconds for every second the drone
is in sight, the direct route ignores the towers. Both are found with A* and a straight-line
heuristic, which never overestimates (every move costs at least its length at full speed), so both
are optimal for what they weigh.

The browser runs the same search (`viewer3d.html`, ported line for line, with the same voxel and
neighbour order, the same arithmetic for the sight lines and the same tie-breaking), and the tests
check both find identical routes.
"""
from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field
from itertools import count

import numpy as np

from .terrain import ROCK, TREE, WATER, Terrain

# what the drone and the towers are like (the viewer gets the same numbers)
AIR = dict(levels=8, dz=3.0, clear=2.0, tree=6.0, rock=1.2,     # flight levels; clearance; how tall trees and rocks stand
           towers=4, spacing=30.0, eye=6.0, sight=24.0,         # watchtowers: how many, how far apart, eye height, reach
           speed=5.0, climb=0.2, exposure=10.0)                 # m/s; seconds per metre climbed; seconds per second seen
SQRT2 = math.sqrt(2)
NB_R = (-1, 1, 0, 0, -1, -1, 1, 1)                              # the planners' neighbour order (grid.EIGHT)
NB_C = (0, 0, -1, 1, -1, 1, -1, 1)
# the 26 moves (dr, dc, dk): level first, then straight up and down, then climbing and descending
MOVES = ([(NB_R[k], NB_C[k], 0) for k in range(8)] + [(0, 0, 1), (0, 0, -1)]
         + [(NB_R[k], NB_C[k], 1) for k in range(8)] + [(NB_R[k], NB_C[k], -1) for k in range(8)])


def water_levels(t: Terrain) -> np.ndarray:
    """Water surface height per cell (NaN on land): the sea, then the ponds, as the viewer builds it."""
    wl = np.full(t.height.shape, np.nan)
    if t.sea_level is not None:
        wl[t.obstacle == WATER] = t.sea_level
    for p in t.ponds:
        for r, c in p["cells"]:
            wl[r, c] = p["level"]
    return wl


def ground_top(t: Terrain) -> np.ndarray:
    """What the drone must clear and what blocks a tower's view: the ground or water, plus any tree or rock."""
    wl = water_levels(t)
    top = np.where(np.isnan(wl), t.height, np.maximum(t.height, np.nan_to_num(wl, nan=-np.inf)))
    return top + np.where(t.obstacle == TREE, AIR["tree"], np.where(t.obstacle == ROCK, AIR["rock"], 0.0))


def place_towers(t: Terrain, n: int | None = None, spacing: float | None = None) -> list[tuple[int, int]]:
    """Watchtowers on the highest open ground, at least `spacing` metres apart, away from the map edges."""
    n = AIR["towers"] if n is None else n
    spacing = AIR["spacing"] if spacing is None else spacing
    R, C = t.rows, t.cols
    m = max(4, R // 12)
    ok = (t.obstacle == 0)
    ok[:m, :] = ok[-m:, :] = ok[:, :m] = ok[:, -m:] = False
    cand = np.argwhere(ok)
    order = np.lexsort((cand[:, 1], cand[:, 0], -t.height[cand[:, 0], cand[:, 1]]))   # highest first, then row, col
    picked: list[tuple[int, int]] = []
    for j in order:
        r, c = int(cand[j, 0]), int(cand[j, 1])
        if all((r - a) * (r - a) + (c - b) * (c - b) >= spacing * spacing for a, b in picked):   # exact in any language
            picked.append((r, c))
            if len(picked) == n:
                break
    return picked


@dataclass
class Airspace:
    """Flight levels over a map: which voxels are free, which a tower can see, and the towers."""
    t: Terrain
    towers: list
    alt: list                      # altitude of each level (metres)
    free: np.ndarray               # (levels, rows, cols) bool
    seen: np.ndarray               # (levels, rows, cols) bool: some tower sees a drone here
    eyes: list = field(default_factory=list)   # each tower's eye height (metres)

    @property
    def shape(self):
        return self.free.shape

    def voxel(self, cell) -> int:
        """The lowest free voxel over a cell (where the drone takes off or lands), as a flat index."""
        r, c = cell
        L, R, C = self.free.shape
        for k in range(L):
            if self.free[k, r, c]:
                return (k * R + r) * C + c
        raise ValueError(f"no free airspace over {cell}")


def make_airspace(t: Terrain, towers=None) -> Airspace:
    """The airspace over a map, with its watchtowers (placed on the hilltops unless given)."""
    towers = place_towers(t) if towers is None else [tuple(map(int, tw)) for tw in towers]
    R, C, L, dz = t.rows, t.cols, AIR["levels"], AIR["dz"]
    top = ground_top(t)
    wl = water_levels(t)
    lo = float(np.min(np.where(np.isnan(wl), t.height, np.maximum(t.height, np.nan_to_num(wl, nan=-np.inf)))))
    alt = [lo + (k + 1) * dz for k in range(L)]
    free = np.stack([a >= top + AIR["clear"] for a in alt])
    seen = np.zeros((L, R, C), bool)
    rr, cc = np.mgrid[0:R, 0:C]
    eyes = []
    sight2 = AIR["sight"] * AIR["sight"]
    for tr, tc in towers:
        E = float(t.height[tr, tc]) + AIR["eye"]
        eyes.append(E)
        for k in range(L):
            dr, dc = rr - tr, cc - tc
            dy = alt[k] - E
            near = ((dr * dr + dc * dc) + dy * dy <= sight2) & free[k]
            near[tr, tc] = False
            vr, vc = rr[near], cc[near]
            n = np.maximum(np.abs(vr - tr), np.abs(vc - tc))       # one sample per cell crossed
            hidden = np.zeros(len(vr), bool)
            for j in range(1, int(n.max(initial=1))):
                on = j < n
                f = j / n[on]
                pr = tr + f * (vr[on] - tr)
                pc = tc + f * (vc[on] - tc)
                line = E + f * (alt[k] - E)
                hidden[on] |= top[np.floor(pr + 0.5).astype(int), np.floor(pc + 0.5).astype(int)] > line
            seen[k, vr[~hidden], vc[~hidden]] = True
    return Airspace(t=t, towers=towers, alt=alt, free=free, seen=seen, eyes=eyes)


@dataclass
class Flight:
    """One route through the airspace. Voxels are flat indices ((level * rows) + row) * cols + col."""
    name: str
    voxels: list
    found: bool
    cost: float                    # what the planner minimised (seconds, exposure included)
    time: float                    # flight time (seconds)
    seen: float                    # seconds of it in some tower's sight
    nodes: int                     # voxels expanded
    runtime_ms: float
    distance: float = 0.0          # metres flown
    top: float = 0.0               # highest altitude above the ground below (metres)


def move_ok(air: Airspace, r: int, c: int, k: int, dr: int, dc: int, dk: int) -> bool:
    """A move is legal if every voxel in its little box (start, end, and the ones it passes beside) is free."""
    L, R, C = air.free.shape
    if not (0 <= r + dr < R and 0 <= c + dc < C and 0 <= k + dk < L):
        return False
    f = air.free
    for kk in ((k, k + dk) if dk else (k,)):
        for rr in ((r, r + dr) if dr else (r,)):
            for cc in ((c, c + dc) if dc else (c,)):
                if not f[kk, rr, cc]:
                    return False
    return True


def move_time(dr: int, dc: int, dk: int) -> float:
    """Seconds to fly one move: its length at cruise speed, plus a little per metre climbed."""
    h = SQRT2 if (dr and dc) else (1.0 if (dr or dc) else 0.0)
    dy = dk * AIR["dz"]
    return math.sqrt(h * h + dy * dy) / AIR["speed"] + AIR["climb"] * max(dy, 0.0)


def fly(air: Airspace, start, goal, stealth: bool = True, heuristic: bool = True) -> Flight:
    """The quickest route from the ground at `start` to the ground at `goal` (both (row, col) cells).

    stealth=True also pays `exposure` seconds for each second in sight, so it hides when it can.
    heuristic=False searches without the straight-line estimate (Dijkstra), for checking A*."""
    t0 = time.perf_counter()
    L, R, C = air.free.shape
    s, g = air.voxel(start), air.voxel(goal)
    gk, rem = divmod(g, R * C)
    gr, gc = divmod(rem, C)
    dz, speed, w = AIR["dz"], AIR["speed"], AIR["exposure"] if stealth else 0.0
    seen = air.seen.ravel()
    MT = [move_time(dr, dc, dk) for dr, dc, dk in MOVES]

    def h(v: int) -> float:
        if not heuristic:
            return 0.0
        k, rem = divmod(v, R * C)
        r, c = divmod(rem, C)
        y = (k - gk) * dz
        return math.sqrt((r - gr) * (r - gr) + (c - gc) * (c - gc) + y * y) / speed

    tie = count()
    gs = {s: 0.0}
    parent = {s: -1}
    closed = set()
    heap = [(h(s), next(tie), s)]
    nodes = 0
    while heap:
        _, _, v = heapq.heappop(heap)
        if v in closed:
            continue
        closed.add(v)
        nodes += 1
        if v == g:
            break
        k, rem = divmod(v, R * C)
        r, c = divmod(rem, C)
        for m, (dr, dc, dk) in enumerate(MOVES):
            if not move_ok(air, r, c, k, dr, dc, dk):
                continue
            u = v + (dk * R + dr) * C + dc
            step = MT[m] + (w * MT[m] if seen[u] else 0.0)
            ng = gs[v] + step
            if u not in gs or ng < gs[u]:
                gs[u] = ng
                parent[u] = v
                heapq.heappush(heap, (ng + h(u), next(tie), u))
    found = g in closed
    path = []
    if found:
        v = g
        while v != -1:
            path.append(v)
            v = parent[v]
        path.reverse()
    out = Flight(name="Stealth" if stealth else "Direct", voxels=path, found=found, cost=gs.get(g, math.inf) if found else math.inf,
                 time=0.0, seen=0.0, nodes=nodes, runtime_ms=(time.perf_counter() - t0) * 1e3)
    if found:
        out.time, out.seen, out.distance, out.top = flight_stats(air, path)
    else:
        out.time = out.seen = math.inf
    return out


def flight_stats(air: Airspace, path: list) -> tuple[float, float, float, float]:
    """Flight time, seconds in sight, metres flown and the highest height above the ground below."""
    L, R, C = air.free.shape
    top = ground_top(air.t)
    secs = seen = dist = 0.0
    hi = 0.0
    seen_flat = air.seen.ravel()
    for a, b in zip(path, path[1:]):
        ka, ra = divmod(a, R * C)
        kb, rb = divmod(b, R * C)
        (r0, c0), (r1, c1) = divmod(ra, C), divmod(rb, C)
        mt = move_time(r1 - r0, c1 - c0, kb - ka)
        secs += mt
        if seen_flat[b]:
            seen += mt
        hgt = SQRT2 if (r1 != r0 and c1 != c0) else (1.0 if (r1 != r0 or c1 != c0) else 0.0)
        dy = (kb - ka) * AIR["dz"]
        dist += math.sqrt(hgt * hgt + dy * dy)
        hi = max(hi, air.alt[kb] - float(top[r1, c1]))
    return secs, seen, dist, hi
