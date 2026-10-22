import math

import pytest

from robotnav import ALGORITHMS, astar, dijkstra
from dataclasses import replace

import numpy as np

from robotnav.terrain import (MUD, course_solvable, evaluate_route, offroad_course, run_course,
                              solvable_course, waypoints)


@pytest.fixture(scope="module")
def course():
    return solvable_course(seed=3, size=48)


@pytest.mark.parametrize("seed", range(8))
def test_astar_optimal_on_hills(seed):
    t = offroad_course(size=40, seed=seed)
    if not course_solvable(t):
        pytest.skip("undrivable course")
    a, d = run_course(t, astar), run_course(t, dijkstra)
    assert a.cost == pytest.approx(d.cost)
    assert a.nodes_expanded <= d.nodes_expanded


def test_route_respects_grade_limit_and_obstacles(course):
    r = run_course(course, astar)
    for a, b in zip(r.path, r.path[1:]):
        assert course.edge_cost(a, b) is not None        # never steeper than max_grade
        assert not course.blocked[b]


def test_route_visits_every_checkpoint_in_order(course):
    r = run_course(course, astar)
    idx = [r.path.index(w) for w in waypoints(course)]
    assert idx == sorted(idx) and idx[0] == 0 and idx[-1] == len(r.path) - 1


def test_uphill_costs_more_than_downhill(course):
    h = course.height
    a, b = (10, 10), (10, 11)
    if h[a] == h[b]:
        pytest.skip("flat pair")
    lo, hi = (a, b) if h[a] < h[b] else (b, a)
    up, down = course.edge_cost(lo, hi), course.edge_cost(hi, lo)
    if up is not None:
        assert up > down


def test_mud_is_expensive(course):
    flat = course.edge_cost  # compare one mud move vs same geometry on grass
    import numpy as np
    r, c = map(int, np.argwhere(course.surface == MUD)[0])
    nb = (r, c + 1) if c + 1 < course.cols else (r, c - 1)
    saved = course.surface[r, c]
    cost_mud = flat(nb, (r, c))
    course.surface[r, c] = 0
    cost_grass = flat(nb, (r, c))
    course.surface[r, c] = saved
    if cost_mud is not None:
        assert cost_mud == pytest.approx(3 * cost_grass)


def test_too_steep_is_impossible():
    t = offroad_course(size=32, hilliness=200, seed=1)
    assert not course_solvable(t)


def test_all_planners_finish(course):
    for fn in ALGORITHMS.values():
        r = run_course(course, fn)
        assert r.found and not math.isinf(r.cost)


# ---------------------------------------------------------------- hand-drawn routes

def test_evaluate_route_matches_planner(course):
    r = run_course(course, astar)
    res = evaluate_route(course, r.path)
    assert res["valid"] and res["time"] == pytest.approx(r.cost)


def test_hand_route_never_beats_optimal(course):
    best = run_course(course, astar).cost
    for fn in ALGORITHMS.values():
        res = evaluate_route(course, run_course(course, fn).path)
        assert res["valid"] and res["time"] >= best - 1e-9


def test_evaluate_rejects_obstacle(course):
    path = run_course(course, astar).path
    blocked = course.blocked.copy()
    blocked[path[len(path) // 2]] = True
    res = evaluate_route(replace(course, blocked=blocked), path)
    assert not res["valid"] and "obstacle" in res["reason"]


def test_evaluate_rejects_jump(course):
    path = run_course(course, astar).path
    a = path[4]
    res = evaluate_route(course, path[:5] + [(a[0] + 3, a[1])] + path[5:])
    assert not res["valid"] and "jumps" in res["reason"]


def test_evaluate_rejects_too_steep(course):
    path = run_course(course, astar).path
    h = course.height.copy()
    h[path[len(path) // 2]] += 10
    res = evaluate_route(replace(course, height=h), path)
    assert not res["valid"] and "grade" in res["reason"]


def test_evaluate_requires_checkpoints(course):
    direct = astar(replace(course, start=course.start, goal=course.goal)).path
    res = evaluate_route(course, direct)
    assert not res["valid"] and "checkpoint" in res["reason"]


def test_evaluate_must_start_at_start(course):
    path = run_course(course, astar).path
    assert "start" in evaluate_route(course, path[1:])["reason"]


def test_meta_rebuilds_the_same_course(course):
    again = offroad_course(**course.meta)
    assert np.array_equal(again.height, course.height) and np.array_equal(again.blocked, course.blocked)
    assert waypoints(again) == waypoints(course)
