import math

import numpy as np
import pytest

from robotnav import ALGORITHMS, astar, bfs, dijkstra, from_ascii, greedy, random_grid

SIMPLE = """
S....
.###.
...#.
.#...
....G
"""

WALLED = """
S.#..
..#..
..#.G
"""


def test_all_find_path_on_simple_map():
    g = from_ascii(SIMPLE)
    for fn in ALGORITHMS.values():
        r = fn(g)
        assert r.found and r.path[0] == g.start and r.path[-1] == g.goal


def test_path_is_contiguous_and_avoids_obstacles():
    g = random_grid(30, 30, 0.25, diagonal=True, seed=3)
    r = astar(g)
    if not r.found:
        pytest.skip("random map unsolvable")
    for a, b in zip(r.path, r.path[1:]):
        assert max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 1
        assert not g.blocked[b]


def test_no_path_reported():
    g = from_ascii(WALLED)
    for fn in ALGORITHMS.values():
        r = fn(g)
        assert not r.found and math.isinf(r.cost)


@pytest.mark.parametrize("diag", [False, True])
@pytest.mark.parametrize("seed", range(15))
def test_astar_matches_dijkstra_cost(seed, diag):
    g = random_grid(25, 25, 0.25, diagonal=diag, seed=seed)
    a, d = astar(g), dijkstra(g)
    assert a.found == d.found
    if a.found:
        assert a.cost == pytest.approx(d.cost)
        assert a.nodes_expanded <= d.nodes_expanded


@pytest.mark.parametrize("seed", range(10))
def test_bfs_optimal_on_uniform_4way(seed):
    g = random_grid(25, 25, 0.25, seed=seed)
    b, d = bfs(g), dijkstra(g)
    if d.found:
        assert b.cost == pytest.approx(d.cost)


@pytest.mark.parametrize("seed", range(10))
def test_greedy_never_beats_optimal(seed):
    g = random_grid(25, 25, 0.25, diagonal=True, seed=seed)
    gr, d = greedy(g), dijkstra(g)
    if d.found:
        assert gr.cost >= d.cost - 1e-9


def test_terrain_weights_respected():
    g = random_grid(25, 25, 0.1, seed=1, terrain=True)
    a, d = astar(g), dijkstra(g)
    assert a.cost == pytest.approx(d.cost)


def test_no_corner_cutting():
    g = from_ascii("S#\n#G", diagonal=True)
    assert not astar(g).found


@pytest.mark.parametrize("seed", range(20))
def test_rooms_always_connected(seed):
    from robotnav.grid import is_reachable, rooms_grid
    assert is_reachable(rooms_grid(40, 40, seed=seed))
