"""Driving with a limited sensor: D* Lite re-planning on the drive-time cost model.

The rover starts knowing almost nothing: beyond its sensor range the terrain is assumed flat and open,
the cheapest it could possibly be. Each time it moves one cell it senses every cell within
`sensor_range` metres (heights, surface, obstacles; a 360-degree range sensor, no occlusion), and every
surprise can only make moves dearer: a hill, soft ground, water, a rock, or a roadblock that was not
there before. Rather than plan again from scratch, it repairs its plan with D* Lite (Koenig &
Likhachev, 2002), which searches backwards from the destination and only re-examines the cells whose
cost-to-go the new information changed.

For comparison, every run also counts what A* would have searched had it re-planned from scratch
each time the plan broke (when a newly sensed cell made a move on the remaining plan dearer). As
costs only ever go up, a plan that no surprise touched is still optimal, so that is a fair baseline.

The browser runs the same algorithm, ported line for line (same neighbour order, keys and
tie-breaking), and the tests check both drive identical routes with identical work counts.
Costs use sqrt(run^2 + rise^2), like the terrain's own drive-time model, so Python, C++ and JavaScript round alike.
"""
from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field, replace

from .algorithms import _best_first
from .terrain import Terrain, add_up

INF = math.inf
SQRT2 = math.sqrt(2)
NB_R = (-1, 1, 0, 0, -1, -1, 1, 1)          # the planners' neighbour order (grid.EIGHT)
NB_C = (0, 0, -1, 1, -1, 1, -1, 1)
# what the rover knows before it sets off:
#   "none"       nothing: unsensed ground is assumed flat and open
#   "elevation"  the heights and surfaces (say, from satellite data), but none of the obstacles
#   "map"        the whole map, except roadblocks that are not on it
PRIORS = ("none", "elevation", "map")


@dataclass(frozen=True)
class Roadblock:
    """An obstacle that is not on the map: it appears at step `appears` (0 = from the start)."""
    cell: tuple[int, int]
    appears: int = 0


class Belief:
    """What the rover believes about the terrain: which cells it has sensed, and which it thinks are blocked.

    Unsensed cells are assumed open and flat. A move is priced exactly once both its cells have been
    sensed; until then it costs its flat length at the fastest surface, a lower bound on its real cost.
    """

    def __init__(self, t: Terrain, prior: str = "none"):
        if prior not in PRIORS:
            raise ValueError(f"prior must be one of {PRIORS}")
        self.R, self.C, self.N = t.rows, t.cols, t.rows * t.cols
        self.cell = float(t.cell_size)
        self.max_grade = float(t.max_grade)
        self.uphill = float(t.uphill_penalty)
        self.H = [float(v) for v in t.height.ravel()]
        self.S = [float(t.surface_costs[int(s)]) for s in t.surface.ravel()]
        self.known = [prior != "none"] * self.N            # heights and surfaces
        self.blk = [bool(v) for v in t.blocked.ravel()] if prior == "map" else [False] * self.N

    def cost(self, u: int, v: int, dr: int, dc: int) -> float:
        """Believed seconds to drive u -> v, where v = u + (dr, dc) is one of u's 8 neighbours."""
        blk, C = self.blk, self.C
        if blk[u] or blk[v]:
            return INF
        if dr and dc:
            if blk[u + dr * C] or blk[u + dc]:                     # no squeezing between two obstacles
                return INF
            run = SQRT2 * self.cell
        else:
            run = self.cell
        if not (self.known[u] and self.known[v]):
            return run                                              # unknown ground: flat, open, fastest surface
        rise = self.H[v] - self.H[u]
        grade = rise / run
        if abs(grade) > self.max_grade:
            return INF
        return math.sqrt(run * run + rise * rise) * self.S[v] * (1.0 + self.uphill * max(grade, 0.0))

    def h(self, a: int, b: int) -> float:
        """Octile distance in metres: never more than the real cost, sensed or not."""
        C = self.C
        dr, dc = abs(a // C - b // C), abs(a % C - b % C)
        return ((dr + dc) + (SQRT2 - 2) * min(dr, dc)) * self.cell


class DStarLite:
    """D* Lite, the optimised version (Koenig & Likhachev, Fig. 4), on a Belief.

    The queue is a binary heap with lazy deletion; ties on the key break first in, first out, so the
    order of expansions is fully determined (and matches the browser's port).
    """

    def __init__(self, belief: Belief, start: int, goal: int):
        self.b, self.start, self.goal, self.last = belief, start, goal, start
        N = belief.N
        self.g, self.rhs = [INF] * N, [INF] * N
        self.km = 0.0
        self.heap: list = []
        self.inq = [0] * N                     # stamp of a cell's live heap entry (0: not in the queue)
        self.stamp = 0
        self.expanded = 0                      # cells whose g changed (the work, like the planners' "searched")
        self.rhs[goal] = 0.0
        self._push(goal, (belief.h(start, goal), 0.0))

    # queue
    def _push(self, u: int, key: tuple) -> None:
        self.stamp += 1
        self.inq[u] = self.stamp
        heapq.heappush(self.heap, (key[0], key[1], self.stamp, u))

    def _top(self):
        heap, inq = self.heap, self.inq
        while heap and inq[heap[0][3]] != heap[0][2]:
            heapq.heappop(heap)                # stale entry
        return heap[0] if heap else None

    def key(self, u: int) -> tuple:
        m = min(self.g[u], self.rhs[u])
        return (m + self.b.h(self.start, u) + self.km, m)

    def update_vertex(self, u: int) -> None:
        if self.g[u] != self.rhs[u]:
            self._push(u, self.key(u))         # insert, or update (the old entry goes stale)
        elif self.inq[u]:
            self.inq[u] = 0                    # remove

    def best_succ(self, u: int) -> tuple[float, int]:
        """min over u's neighbours s of c(u, s) + g(s), and that neighbour (first in neighbour order on ties)."""
        b, C, R, g = self.b, self.b.C, self.b.R, self.g
        r, c = divmod(u, C)
        best, arg = INF, -1
        for k in range(8):
            dr, dc = NB_R[k], NB_C[k]
            nr, nc = r + dr, c + dc
            if nr < 0 or nc < 0 or nr >= R or nc >= C:
                continue
            s = u + dr * C + dc
            v = b.cost(u, s, dr, dc) + g[s]
            if v < best:
                best, arg = v, s
        return best, arg

    def compute(self) -> int:
        """ComputeShortestPath; returns how many cells it expanded."""
        b, C, R, g, rhs, goal = self.b, self.b.C, self.b.R, self.g, self.rhs, self.goal
        before = self.expanded
        while True:
            top = self._top()
            ks = self.key(self.start)
            k_old = (top[0], top[1]) if top else (INF, INF)
            if not (k_old < ks or rhs[self.start] > g[self.start]) or top is None:
                break
            u = top[3]
            k_new = self.key(u)
            if k_old < k_new:
                self._push(u, k_new)
                continue
            r, c = divmod(u, C)
            if g[u] > rhs[u]:
                g[u] = rhs[u]
                self.inq[u] = 0
                self.expanded += 1
                for k in range(8):             # predecessors p = u - (dr, dc); the move p -> u is (dr, dc)
                    dr, dc = NB_R[k], NB_C[k]
                    pr, pc = r - dr, c - dc
                    if pr < 0 or pc < 0 or pr >= R or pc >= C:
                        continue
                    p = u - dr * C - dc
                    if p != goal:
                        v = b.cost(p, u, dr, dc) + g[u]
                        if v < rhs[p]:
                            rhs[p] = v
                    self.update_vertex(p)
            else:
                g_old = g[u]
                g[u] = INF
                self.expanded += 1
                for k in range(8):
                    dr, dc = NB_R[k], NB_C[k]
                    pr, pc = r - dr, c - dc
                    if pr < 0 or pc < 0 or pr >= R or pc >= C:
                        continue
                    p = u - dr * C - dc
                    if p != goal and rhs[p] == b.cost(p, u, dr, dc) + g_old:
                        rhs[p] = self.best_succ(p)[0]
                    self.update_vertex(p)
                self.update_vertex(u)
        return self.expanded - before

    def moved_and_sensed(self, changed: list[int]) -> None:
        """The rover is at self.start and the belief changed at `changed`: fix the right-hand sides."""
        if not changed:
            return
        self.km += self.b.h(self.last, self.start)
        self.last = self.start
        C, R = self.b.C, self.b.R
        touched = set()
        for x in changed:                       # moves out of x, into x, and diagonals that squeeze past x
            r, c = divmod(x, C)
            touched.add(x)
            for k in range(8):
                nr, nc = r + NB_R[k], c + NB_C[k]
                if 0 <= nr < R and 0 <= nc < C:
                    touched.add(nr * C + nc)
        for u in sorted(touched):
            if u != self.goal:
                self.rhs[u] = self.best_succ(u)[0]
            self.update_vertex(u)

    def plan(self, limit: int | None = None) -> list[int]:
        """The current plan: from the rover, always the neighbour with the least cost-to-go."""
        out, u, limit = [self.start], self.start, limit or self.b.N
        while u != self.goal and len(out) <= limit:
            best, u = self.best_succ(u)
            if best == INF:
                return []
            out.append(u)
        return out if u == self.goal else []


class _BeliefGraph:
    """The belief as a graph for robotnav.algorithms' engine (the A*-from-scratch baseline)."""

    def __init__(self, b: Belief, start: int, goal: int):
        self.b, self.start, self.goal = b, divmod(start, b.C), divmod(goal, b.C)

    def neighbors(self, cell):
        b, C, R = self.b, self.b.C, self.b.R
        r, c = cell
        u = r * C + c
        for k in range(8):
            dr, dc = NB_R[k], NB_C[k]
            nr, nc = r + dr, c + dc
            if 0 <= nr < R and 0 <= nc < C:
                cost = b.cost(u, nr * C + nc, dr, dc)
                if cost < INF:
                    yield (nr, nc), cost

    def heuristic(self, a, z) -> float:
        return self.b.h(a[0] * self.b.C + a[1], z[0] * self.b.C + z[1])

    def path_cost(self, path) -> float:
        C = self.b.C
        return add_up(self.b.cost(a[0] * C + a[1], z[0] * C + z[1], z[0] - a[0], z[1] - a[1]) for a, z in zip(path, path[1:]))


def disk(sensor_range: float) -> list[tuple[int, int]]:
    """Cell offsets within `sensor_range` cells of the rover, row by row."""
    n = int(math.floor(sensor_range))
    return [(dr, dc) for dr in range(-n, n + 1) for dc in range(-n, n + 1) if dr * dr + dc * dc <= sensor_range * sensor_range]


@dataclass
class SensorRun:
    """One drive with a limited sensor. Cells are flat indices (row * cols + col)."""
    found: bool
    driven: list[int]                    # every cell the rover drove through, start to finish
    time: float                          # drive time of that route (seconds, the planners' clock)
    optimal: float                       # the quickest time with the whole map known (A*)
    steps: list[dict]                    # per cell driven: changed, work, broke, astar, plan (when it changed)
    dstar_work: int                      # cells D* Lite expanded, in total
    astar_work: int                      # cells A* would have searched, re-planning from scratch each time the plan broke
    replans: int                         # times the plan broke and had to change
    sensed: int                          # cells sensed by the end
    runtime_ms: float
    max_check_error: float = 0.0         # with check=True: largest |D* Lite plan - A* from scratch| seen (seconds)
    first_seen: list[int] = field(default_factory=list, repr=False)   # per cell: step it was first sensed + 1 (0: never)

    @property
    def extra(self) -> float:
        """How much longer than with a perfect map (0.25 = 25% longer)."""
        return self.time / self.optimal - 1.0 if self.found and self.optimal > 0 else math.inf


def sensor_drive(t: Terrain, start=None, goal=None, sensor_range: float = 10.0, prior: str = "none",
                 roadblocks=(), check: bool = False, max_steps: int | None = None) -> SensorRun:
    """Drive from `start` to `goal` seeing only `sensor_range` metres around the rover, re-planning with D* Lite.

    `prior` is what the rover knows before it sets off (see PRIORS): nothing, the elevation without
    the obstacles, or the whole map; it never knows the roadblocks (obstacles that are not on the map)
    until it senses them. `roadblocks` is a list of Roadblock (or
    (row, col) cells, which appear from the start). check=True also verifies, every time the plan
    breaks, that D* Lite's repaired plan is exactly as quick as A* planning from scratch on the same
    belief (the largest difference is reported as max_check_error).
    """
    from .algorithms import astar

    t0 = time.perf_counter()
    C, R = t.cols, t.rows
    s = tuple(t.start if start is None else start)
    gcell = tuple(t.goal if goal is None else goal)
    si, gi = s[0] * C + s[1], gcell[0] * C + gcell[1]
    appears: dict[int, int] = {}                 # roadblock cell -> the step it appears at
    for rb in roadblocks:
        rb = rb if isinstance(rb, Roadblock) else Roadblock(tuple(rb))
        x = rb.cell[0] * C + rb.cell[1]
        appears[x] = min(appears.get(x, rb.appears), rb.appears)
    world = [bool(v) for v in t.blocked.ravel()]
    b = Belief(t, prior)
    offsets = disk(max(sensor_range, 2.0))   # the cells next to the rover are always sensed: no blind moves
    first_seen = [0] * (R * C)

    def truth(x: int, step: int) -> bool:
        return world[x] or appears.get(x, step + 1) <= step

    def sense(at: int, step: int) -> list[int]:
        r, c = divmod(at, C)
        changed = []
        for dr, dc in offsets:
            nr, nc = r + dr, c + dc
            if nr < 0 or nc < 0 or nr >= R or nc >= C:
                continue
            x = nr * C + nc
            blk = truth(x, step)
            if not b.known[x] or b.blk[x] != blk:
                b.known[x] = True
                b.blk[x] = blk
                changed.append(x)
            if not first_seen[x]:
                first_seen[x] = step + 1
        changed.sort()
        return changed

    def edge_costs(path: list[int]) -> list[float]:
        return [b.cost(a, z, z // C - a // C, z % C - a % C) for a, z in zip(path, path[1:])]

    def astar_from_scratch(at: int) -> tuple[int, float]:
        res = _best_first(_BeliefGraph(b, at, gi), "A*", lambda g, h: g + h)
        return res.nodes_expanded, (res.cost if res.found else INF)

    optimal_res = astar(replace(t, start=s, goal=gcell, checkpoints=[]))
    optimal = optimal_res.cost if optimal_res.found else INF

    changed = sense(si, 0)
    ds = DStarLite(b, si, gi)
    work = ds.compute()
    plan = ds.plan()
    plan_costs = edge_costs(plan)
    astar_work, check_err = 0, 0.0
    n0, c0 = astar_from_scratch(si)
    astar_work += n0
    if check and plan:
        check_err = max(check_err, abs(add_up(plan_costs) - c0))
    steps = [dict(cell=si, changed=len(changed), work=work, broke=False, astar=n0, plan=plan)]
    driven, cur, replans = [si], si, 0
    limit = max_steps or 4 * R * C
    found = bool(plan)
    while found and cur != gi and len(driven) <= limit:
        nxt = plan[1]
        cur = nxt
        driven.append(cur)
        ds.start = cur
        step = len(driven) - 1
        changed = sense(cur, step)
        rest = plan[1:]
        broke = bool(changed) and edge_costs(rest) != plan_costs[1:]
        work = 0
        if changed:
            ds.moved_and_sensed(changed)
            work = ds.compute()
        rec = dict(cell=cur, changed=len(changed), work=work, broke=broke, astar=0)
        if broke:
            replans += 1
            n, cost = astar_from_scratch(cur)
            astar_work += n
            rec["astar"] = n
            plan = ds.plan()
            if check and plan:
                check_err = max(check_err, abs(add_up(edge_costs(plan)) - cost))
            rec["plan"] = plan
        elif changed:
            new = ds.plan()
            if new != rest:
                rec["plan"] = new                 # an equally quick alternative (rare): follow D* Lite's choice
            plan = new
        else:
            plan = rest
        if not plan:
            found = False
            steps.append(rec)
            break
        plan_costs = edge_costs(plan)
        steps.append(rec)
    found = found and cur == gi
    cells = [divmod(i, C) for i in driven]
    return SensorRun(found=found, driven=driven, time=t.path_cost(cells) if found else INF, optimal=optimal,
                     steps=steps, dstar_work=ds.expanded, astar_work=astar_work, replans=replans,
                     sensed=sum(1 for v in first_seen if v), runtime_ms=(time.perf_counter() - t0) * 1e3,
                     max_check_error=check_err, first_seen=first_seen)
