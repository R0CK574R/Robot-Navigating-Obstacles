import math

import numpy as np
import pytest
from scipy.ndimage import binary_dilation

from robotnav import astar, dijkstra
from robotnav.terrain import (THEMES, WATER, evaluate_route, is_reachable, make_map, plan_pair, preset_routes,
                              shortest_route, solvable_map)


@pytest.fixture(scope="module", params=THEMES)
def themed(request):
    return solvable_map(request.param, seed=3, size=48)


def test_default_flags_are_reachable(themed):
    assert is_reachable(themed)
    assert not themed.blocked[themed.start] and not themed.blocked[themed.goal]


def test_surfaces_never_faster_than_cruise(themed):
    # keeps the octile heuristic admissible, so A* stays optimal
    assert min(themed.surface_costs) >= 1.0
    assert len(themed.surface_costs) == len(themed.surface_names) == int(themed.surface.max()) + 1


def test_island_sea_is_impassable():
    t = solvable_map("island", seed=3, size=48)
    water = t.obstacle == WATER
    assert water.any() and t.blocked[water].all()
    assert (t.height[water] < t.sea_level).all()


@pytest.mark.parametrize("size", [96, 48])
def test_desert_oases_are_closed_ponds_the_rover_can_reach(size):
    from dataclasses import replace
    t = solvable_map("desert", seed=3, size=size)
    assert len(t.ponds) >= 2
    water = np.zeros(t.blocked.shape, bool)
    for p in t.ponds:
        cells = np.array(p["cells"])
        assert (t.obstacle[cells[:, 0], cells[:, 1]] == WATER).all()
        assert (t.height[cells[:, 0], cells[:, 1]] < p["level"]).all()              # under water
        assert p["level"] - t.height[cells[:, 0], cells[:, 1]].min() >= 0.6         # and deep enough to read as a pond
        pond = np.zeros(t.blocked.shape, bool); pond[cells[:, 0], cells[:, 1]] = True
        rim = binary_dilation(pond, np.ones((3, 3), bool)) & ~pond
        assert (t.height[rim] >= p["level"]).all()                                  # a closed basin: it can't spill
        assert any(is_reachable(replace(t, goal=tuple(map(int, c)))) for c in np.argwhere(rim & ~t.blocked))
        water |= pond
    assert (water == (t.obstacle == WATER)).all()                                   # all the desert's water is ponds
    assert not water[t.start] and not water[t.goal]


def test_astar_is_quickest_on_every_map(themed):
    rng = np.random.default_rng(0)
    open_cells = np.argwhere(~themed.blocked)
    for _ in range(4):
        a, b = (tuple(map(int, open_cells[i])) for i in rng.choice(len(open_cells), 2, replace=False))
        res = {r.algorithm.split(" (")[0]: r for r in plan_pair(themed, a, b)}
        assert res["A*"].found == res["Dijkstra"].found
        if res["A*"].found:
            assert res["A*"].cost == pytest.approx(res["Dijkstra"].cost)
            for r in res.values():
                assert r.cost >= res["Dijkstra"].cost - 1e-9


def test_planned_route_passes_route_check(themed):
    r = plan_pair(themed, themed.start, themed.goal)[2]          # A*
    res = evaluate_route(themed, r.path)
    assert res["valid"] and res["time"] == pytest.approx(r.cost)


def test_meta_rebuilds_the_map(themed):
    again = make_map(**themed.meta)
    assert np.array_equal(again.height, themed.height) and np.array_equal(again.obstacle, themed.obstacle)


# ---------------------------------------------------------------- the hill-blind shortest route

def _flat_length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def test_shortest_route_is_shortest_drivable_and_never_quicker(themed):
    rng = np.random.default_rng(1)
    open_cells = np.argwhere(~themed.blocked)
    for _ in range(4):
        a, b = (tuple(map(int, open_cells[i])) for i in rng.choice(len(open_cells), 2, replace=False))
        blind = shortest_route(themed, a, b)
        best = plan_pair(themed, a, b)[2]                            # A*
        assert blind.found == best.found
        if not blind.found:
            continue
        assert evaluate_route(_leg(themed, dict(start=a, goal=b)), blind.path)["valid"]
        assert blind.cost == pytest.approx(themed.path_cost(blind.path))      # cost is real drive time
        assert best.cost <= blind.cost + 1e-9                                  # reading the hills never loses
        # no drivable route is shorter: compare with every planner's route
        for r in plan_pair(themed, a, b):
            assert _flat_length(blind.path) <= _flat_length(r.path) + 1e-9


# ---------------------------------------------------------------- the 20 preset paths (on the real 96 x 96 maps)

@pytest.fixture(scope="module", params=THEMES)
def full_presets(request):
    t = solvable_map(request.param, seed=3)
    return t, preset_routes(t)


def test_twenty_presets_drivable_distinct_and_sorted(full_presets):
    t, presets = full_presets
    assert len(presets) == 20
    assert len({(p["start"], p["goal"]) for p in presets}) == 20
    assert len({p["name"] for p in presets}) == 20
    for p in presets:
        assert not t.blocked[p["start"]] and not t.blocked[p["goal"]]
        assert p["path"][0] == p["start"] and p["path"][-1] == p["goal"]
        res = evaluate_route(_leg(t, p), p["path"])
        assert res["valid"] and res["time"] == pytest.approx(p["time"])
    times = [p["time"] for p in presets]
    assert times == sorted(times)


def test_presets_cross_the_map_and_beat_the_shortest_route(full_presets):
    t, presets = full_presets
    land = np.argwhere(t.obstacle != WATER)
    extent = min(np.ptp(land[:, 0]), np.ptp(land[:, 1]))
    for p in presets:
        assert math.dist(p["start"], p["goal"]) >= 0.35 * extent          # long: across the land
        blind = shortest_route(t, p["start"], p["goal"])
        assert p["short_time"] == pytest.approx(blind.cost)
        assert p["time"] <= 0.95 * blind.cost                              # the hills change the answer
        # and visibly: somewhere the two routes run at least 4 m apart
        gap = max(min(math.dist(c, d) for d in blind.path) for c in p["path"])
        assert gap >= 4


def test_presets_take_mostly_new_ground(full_presets):
    t, presets = full_presets
    seen = np.zeros(t.blocked.shape, bool)
    shared = []
    for p in sorted(presets, key=lambda q: -q["saving"]):                  # roughly the order they were picked
        cells = np.array(p["path"])
        shared.append(seen[cells[:, 0], cells[:, 1]].mean())
        seen[cells[:, 0], cells[:, 1]] = True
    assert np.median(shared) <= 0.35


def test_presets_are_repeatable():
    t = solvable_map("desert", seed=3, size=48)
    first = [(p["start"], p["goal"], p["time"]) for p in preset_routes(t)]
    assert first and first == [(p["start"], p["goal"], p["time"]) for p in preset_routes(t)]


def _leg(t, p):
    from dataclasses import replace
    return replace(t, start=p["start"], goal=p["goal"], checkpoints=[])
