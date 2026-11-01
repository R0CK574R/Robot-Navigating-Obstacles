"""Hold the C++ core against the Python reference: the same routes, cell counts and times, and how much faster.

Everything is compared with ==, never a tolerance: the routes, the order the cells were searched in, the frontier
peaks, every drive time and edge cost, every step of every D* Lite drive (what changed, the work, the plans), the
airspace (free voxels and which ones a tower sees) and every flight.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from . import cpp
from .airspace import fly, make_airspace
from .grid import EIGHT
from .replan import Roadblock, sensor_drive
from .terrain import Terrain, plan_pair, shortest_route

PLANNER_FIELDS = ("algorithm", "path", "cost", "nodes_expanded", "max_frontier", "explored_order")
DRIVE_FIELDS = ("found", "driven", "time", "optimal", "steps", "dstar_work", "astar_work", "replans", "sensed",
                "max_check_error", "first_seen")
FLIGHT_FIELDS = ("name", "voxels", "found", "cost", "time", "seen", "nodes", "distance", "top")


@dataclass
class Tally:
    """How many results were compared, how many differed (with the first few), and the time each side took."""
    same: int = 0
    total: int = 0
    py_ms: float = 0.0
    cpp_ms: float = 0.0
    diffs: list = field(default_factory=list)

    def check(self, what: str, a, b, fields) -> None:
        self.total += 1
        bad = [f for f in fields if getattr(a, f) != getattr(b, f)]
        if bad:
            self.diffs.append(f"{what}: {', '.join(bad)}")
        else:
            self.same += 1

    def __str__(self) -> str:
        return f"{self.same}/{self.total}" + ("" if self.same == self.total else " DIFFERENT")


def compare_edges(t: Terrain, m) -> Tally:
    """Every move's drive time (or None where it is too steep), on both sides."""
    tally = Tally()
    C = t.cols
    for r in range(t.rows):
        for c in range(C):
            for dr, dc in EIGHT:
                if 0 <= r + dr < t.rows and 0 <= c + dc < C:
                    tally.total += 1
                    if t.edge_cost((r, c), (r + dr, c + dc)) == m.edge_cost(r * C + c, (r + dr) * C + c + dc):
                        tally.same += 1
                    elif len(tally.diffs) < 5:
                        tally.diffs.append(f"edge {(r, c)} -> {(r + dr, c + dc)}")
    return tally


def compare_map(t: Terrain, presets: list[dict], ranges=(5.0, 10.0, 20.0), log=None) -> dict[str, Tally]:
    """Compare everything on one map's preset trips."""
    m = cpp.to_cpp(t)
    rc = cpp.core()
    out = dict(edges=compare_edges(t, m), planners=Tally(), stats=Tally(), drives=Tally(), airspace=Tally(),
               flights=Tally())
    for k, p in enumerate(presets, start=1):
        a, b = tuple(p["start"]), tuple(p["goal"])
        py = plan_pair(t, a, b) + [shortest_route(t, a, b)]
        cc = cpp.race(t, a, b)
        for x, y in zip(py, cc):
            out["planners"].check(f"P{k} {x.algorithm}", x, y, PLANNER_FIELDS)
            out["planners"].py_ms += x.runtime_ms
            out["planners"].cpp_ms += y.runtime_ms
            out["stats"].total += 1
            if x.found and t.path_stats(x.path) == rc.path_stats(m, [r * t.cols + c for r, c in y.path]):
                out["stats"].same += 1
            elif x.found:
                out["stats"].diffs.append(f"P{k} {x.algorithm} path stats")
            else:
                out["stats"].same += 1
        # the drives: each sensor range knowing nothing; plus knowing the hills, and the map with a late roadblock
        drives = [dict(sensor_range=r, prior="none") for r in ranges]
        mid = py[2].path[len(py[2].path) // 2]                          # halfway along A*'s route
        drives += [dict(sensor_range=ranges[0], prior="elevation"),
                   dict(sensor_range=ranges[0], prior="map", roadblocks=[Roadblock(mid, appears=3)])]
        for kw in drives:
            x = sensor_drive(t, a, b, check=True, **kw)
            y = cpp.sensor_drive(t, a, b, check=True, **kw)
            out["drives"].check(f"P{k} {kw}", x, y, DRIVE_FIELDS)
            out["drives"].py_ms += x.runtime_ms
            out["drives"].cpp_ms += y.runtime_ms
        if log:
            log(f"  P{k}", end="", flush=True)
    t0 = time.perf_counter()
    pa = make_airspace(t)
    t1 = time.perf_counter()
    ca = cpp.make_airspace(t)
    t2 = time.perf_counter()
    air = out["airspace"]
    air.total, air.py_ms, air.cpp_ms = 1, (t1 - t0) * 1e3, (t2 - t1) * 1e3
    if pa.towers == ca.towers and pa.alt == ca.alt and pa.eyes == ca.eyes and np.array_equal(pa.free, ca.free) \
            and np.array_equal(pa.seen, ca.seen):
        air.same = 1
    else:
        air.diffs.append("the airspace")
    for k, p in enumerate(presets, start=1):
        a, b = tuple(p["start"]), tuple(p["goal"])
        for stealth in (True, False):
            x, y = fly(pa, a, b, stealth=stealth), cpp.fly(ca, a, b, stealth=stealth)
            out["flights"].check(f"P{k} {x.name}", x, y, FLIGHT_FIELDS)
            out["flights"].py_ms += x.runtime_ms
            out["flights"].cpp_ms += y.runtime_ms
    if log:
        log("")
    return out
