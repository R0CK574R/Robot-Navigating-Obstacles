"""The C++ core from Python: give it a Terrain, get the Python reference's own result types back.

The route-deciding code (the five planners, the hill-blind shortest route, D* Lite with a limited sensor and the
air route) also exists in C++ (cpp/), computing with the same operations in the same order, so it finds the same
routes with the same times, to the last bit. This module runs it through its pybind11 module, robotnav_cpp, so the
tests and `python main.py parity` can hold the two side by side, and the CLI can use it (--engine cpp).

Build the module first (it lands in cpp/build, where this module looks for it):

    cmake -S cpp -B cpp/build
    cmake --build cpp/build
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from .airspace import Flight
from .algorithms import SearchResult
from .replan import Roadblock, SensorRun
from .terrain import NONE, SHORTEST_TIE, Terrain

BUILD = Path(__file__).resolve().parent.parent / "cpp" / "build"
HOW_TO_BUILD = "Build the C++ core first:\n  cmake -S cpp -B cpp/build\n  cmake --build cpp/build"
_core = None


def core():
    """The compiled robotnav_cpp module; ImportError (with the build commands) if it has not been built."""
    global _core
    if _core is None:
        try:
            import robotnav_cpp
        except ImportError:
            for d in (BUILD, *sorted(BUILD.glob("*"))):        # single-config builds, and Release/ Debug/ ones
                if d.is_dir() and str(d) not in sys.path:
                    sys.path.append(str(d))
            try:
                import robotnav_cpp
            except ImportError as e:
                raise ImportError(f"The C++ core (robotnav_cpp) is not built. {HOW_TO_BUILD}") from e
        _core = robotnav_cpp
    return _core


def available() -> bool:
    try:
        core()
        return True
    except ImportError:
        return False


def to_cpp(t: Terrain, presets=()):
    """The C++ Map of a Terrain: the same heights, surfaces, obstacles, water and cost model."""
    if not np.array_equal(t.blocked, t.obstacle != NONE):
        raise ValueError("the C++ core reads obstacles, so the map's blocked cells must be exactly its obstacles")
    rc = core()
    m = rc.Map()
    m.set_grids(np.asarray(t.height, dtype=float), np.asarray(t.surface, dtype=np.int32),
                np.asarray(t.obstacle, dtype=np.int32))
    m.theme = str(t.meta.get("theme", "course"))
    m.seed = int(t.meta.get("seed") or 0)
    m.cell, m.max_grade, m.uphill, m.short_tie = float(t.cell_size), float(t.max_grade), float(t.uphill_penalty), SHORTEST_TIE
    m.surface_cost = [float(c) for c in t.surface_costs]
    m.surface_names = list(t.surface_names)
    m.sea = None if t.sea_level is None else float(t.sea_level)
    C = t.cols
    m.ponds = [rc.Pond(float(p["level"]), [r * C + c for r, c in p["cells"]]) for p in t.ponds]
    m.start, m.goal = t.start[0] * C + t.start[1], t.goal[0] * C + t.goal[1]
    m.presets = [rc.Preset(p["start"][0] * C + p["start"][1], p["goal"][0] * C + p["goal"][1], p.get("name", ""))
                 for p in presets]
    return m


def _flat(t: Terrain, cell) -> int:
    return int(cell[0]) * t.cols + int(cell[1])


def _cells(C: int, flat) -> list:
    return [divmod(i, C) for i in flat]


def _result(r, C: int) -> SearchResult:
    return SearchResult(r.algorithm, _cells(C, r.path), r.cost, r.nodes_expanded, r.max_frontier, r.runtime_ms,
                        _cells(C, r.explored))


def plan_pair(t: Terrain, start, goal) -> list[SearchResult]:
    """BFS, Dijkstra, A*, Weighted A* and Greedy from start to goal: robotnav.terrain.plan_pair, in C++."""
    m = to_cpp(t)
    return [_result(r, t.cols) for r in core().plan_pair(m, _flat(t, start), _flat(t, goal))]


def shortest_route(t: Terrain, start=None, goal=None) -> SearchResult:
    """The hill-blind shortest route: robotnav.terrain.shortest_route, in C++."""
    s, g = (t.start if start is None else start), (t.goal if goal is None else goal)
    return _result(core().shortest_route(to_cpp(t), _flat(t, s), _flat(t, g)), t.cols)


def race(t: Terrain, start, goal) -> list[SearchResult]:
    """The five planners plus the hill-blind shortest route, as the viewer's board shows them."""
    m, rc = to_cpp(t), core()
    s, g = _flat(t, start), _flat(t, goal)
    return [_result(r, t.cols) for r in [*rc.plan_pair(m, s, g), rc.shortest_route(m, s, g)]]


def sensor_drive(t: Terrain, start=None, goal=None, sensor_range: float = 10.0, prior: str = "none",
                 roadblocks=(), check: bool = False, max_steps: int | None = None) -> SensorRun:
    """robotnav.replan.sensor_drive, in C++: the same drive, step for step."""
    s, g = (t.start if start is None else start), (t.goal if goal is None else goal)
    blocks = []
    for rb in roadblocks:
        rb = rb if isinstance(rb, Roadblock) else Roadblock(tuple(rb))
        blocks.append((_flat(t, rb.cell), int(rb.appears)))
    run = core().sensor_drive(to_cpp(t), _flat(t, s), _flat(t, g), float(sensor_range), prior, blocks, bool(check),
                              int(max_steps or 0))
    steps = []
    for st in run.steps:
        rec = dict(cell=st.cell, changed=st.changed, work=st.work, broke=st.broke, astar=st.astar)
        if st.has_plan:
            rec["plan"] = list(st.plan)
        steps.append(rec)
    return SensorRun(found=run.found, driven=list(run.driven), time=run.time, optimal=run.optimal, steps=steps,
                     dstar_work=run.dstar_work, astar_work=run.astar_work, replans=run.replans, sensed=run.sensed,
                     runtime_ms=run.runtime_ms, max_check_error=run.max_check_error, first_seen=list(run.first_seen))


class Airspace:
    """The C++ airspace, shaped like robotnav.airspace.Airspace (towers, alt, free, seen, eyes, voxel)."""

    def __init__(self, t: Terrain, air):
        self.t, self._air = t, air
        self.towers = [tuple(tw) for tw in air.towers]
        self.alt, self.eyes = list(air.alt), list(air.eyes)
        self.free, self.seen = air.free, air.seen

    @property
    def shape(self):
        return self.free.shape

    def voxel(self, cell) -> int:
        return self._air.voxel(_flat(self.t, cell))


def make_airspace(t: Terrain, towers=None) -> Airspace:
    """robotnav.airspace.make_airspace, in C++."""
    tw = None if towers is None else [(int(r), int(c)) for r, c in towers]
    return Airspace(t, core().make_airspace(to_cpp(t), tw))


def place_towers(t: Terrain, n: int | None = None, spacing: float | None = None) -> list[tuple[int, int]]:
    rc = core()
    return [tuple(tw) for tw in rc.place_towers(to_cpp(t), rc.AIR["towers"] if n is None else n,
                                                 rc.AIR["spacing"] if spacing is None else spacing)]


def fly(air: Airspace, start, goal, stealth: bool = True, heuristic: bool = True) -> Flight:
    """robotnav.airspace.fly, in C++."""
    f = core().fly(air._air, _flat(air.t, start), _flat(air.t, goal), stealth, heuristic)
    return Flight(name=f.name, voxels=list(f.voxels), found=f.found, cost=f.cost, time=f.time, seen=f.seen,
                  nodes=f.nodes, runtime_ms=f.runtime_ms, distance=f.distance, top=f.top)
