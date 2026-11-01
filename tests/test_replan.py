"""D* Lite with a limited sensor: legal routes, optimal repairs, roadblocks, and the work it saves."""
import math
import statistics
from dataclasses import replace

import pytest

from robotnav import astar
from robotnav.replan import INF, PRIORS, Belief, DStarLite, Roadblock, disk, sensor_drive
from robotnav.terrain import evaluate_route, solvable_map


@pytest.fixture(scope="module")
def desert():
    return solvable_map("desert", seed=3, size=48)


@pytest.fixture(scope="module")
def grass():
    return solvable_map("grassland", seed=3, size=48)


def pairs(t, n=4):
    """A few long trips across the map: the default flags, and corners to corners."""
    R, C = t.rows, t.cols
    out = [(t.start, t.goal)]
    for a, b in [((R - 4, C - 4), (3, 3)), ((3, C // 2), (R - 4, C // 2)), ((R // 2, 3), (R // 2, C - 4))]:
        a, b = nearest_open(t, a), nearest_open(t, b)
        if astar(replace(t, start=a, goal=b, checkpoints=[])).found:
            out.append((a, b))
    return out[:n]


def nearest_open(t, cell):
    r0, c0 = cell
    best = min(((r, c) for r in range(t.rows) for c in range(t.cols) if not t.blocked[r, c]),
               key=lambda rc: (rc[0] - r0) ** 2 + (rc[1] - c0) ** 2)
    return best


def cells(t, run):
    return [divmod(i, t.cols) for i in run.driven]


def test_disk_is_round():
    assert len(disk(1.5)) == 9
    assert len(disk(2)) == 13
    assert all(dr * dr + dc * dc <= 100 for dr, dc in disk(10))


def test_unknown_ground_is_never_dearer_than_the_truth(desert):
    """The belief prices unsensed moves at their flat length, a lower bound; sensed moves exactly."""
    unknown, known = Belief(desert, "none"), Belief(desert, "map")
    C = desert.cols
    for r in range(1, desert.rows - 1, 3):
        for c in range(1, C - 1, 3):
            u = r * C + c
            for dr, dc in [(-1, 0), (0, 1), (1, 1), (-1, -1)]:
                v = u + dr * C + dc
                truth = desert.edge_cost((r, c), (r + dr, c + dc))
                blocked = desert.blocked[r + dr, c + dc] or desert.blocked[r, c] or (
                    dr and dc and (desert.blocked[r + dr, c] or desert.blocked[r, c + dc]))
                exact = known.cost(u, v, dr, dc)
                if blocked or truth is None:
                    assert exact == INF
                else:
                    assert exact == pytest.approx(truth, rel=1e-12)
                    assert unknown.cost(u, v, dr, dc) <= exact


def test_with_the_whole_map_it_drives_the_optimal_route(desert):
    for a, b in pairs(desert):
        best = astar(replace(desert, start=a, goal=b, checkpoints=[]))
        run = sensor_drive(desert, a, b, sensor_range=6, prior="map", check=True)
        assert run.found and run.replans == 0
        assert run.time == pytest.approx(best.cost, abs=1e-9)
        assert run.max_check_error < 1e-9


@pytest.mark.parametrize("prior", PRIORS)
@pytest.mark.parametrize("sensor", [3, 6, 12])
def test_routes_are_legal_and_every_repair_is_optimal(desert, prior, sensor):
    for a, b in pairs(desert, 3):
        run = sensor_drive(desert, a, b, sensor_range=sensor, prior=prior, check=True)
        assert run.found
        route = cells(desert, run)
        verdict = evaluate_route(replace(desert, start=a, goal=b, checkpoints=[]), route)
        assert verdict["valid"], verdict["reason"]
        assert run.time >= run.optimal - 1e-9                     # it can't beat a perfect map
        assert run.max_check_error < 1e-9                         # each repaired plan == A* from scratch


def test_less_knowledge_costs_time(grass):
    """Seeing less makes the route slower on the whole (never quicker than the perfect map)."""
    short = [sensor_drive(grass, a, b, sensor_range=3).extra for a, b in pairs(grass)]
    full = [sensor_drive(grass, a, b, sensor_range=3, prior="map").extra for a, b in pairs(grass)]
    assert all(e >= -1e-12 for e in short) and all(abs(e) < 1e-12 for e in full)
    assert statistics.mean(short) > 0.02


def test_a_roadblock_on_the_route_forces_a_detour(desert):
    a, b = desert.start, desert.goal
    best = astar(replace(desert, start=a, goal=b, checkpoints=[]))
    block = best.path[len(best.path) // 2]
    run = sensor_drive(desert, a, b, sensor_range=6, prior="map", roadblocks=[block], check=True)
    assert run.found and run.replans >= 1
    assert block not in cells(desert, run)
    assert run.time > best.cost and run.max_check_error < 1e-9


def test_a_roadblock_can_appear_after_the_rover_has_seen_that_ground(desert):
    a, b = desert.start, desert.goal
    first = sensor_drive(desert, a, b, sensor_range=6, prior="map")
    route = cells(desert, first)
    k = len(route) * 2 // 3
    late = Roadblock(route[k], appears=k - 2)                 # ahead of the rover, on ground it saw long ago
    run = sensor_drive(desert, a, b, sensor_range=6, prior="map", roadblocks=[late], check=True)
    assert run.found and run.replans >= 1 and route[k] not in cells(desert, run)
    assert cells(desert, run)[:k - 2] == route[:k - 2]        # the same route until it appeared


def test_a_walled_off_finish_is_reported_not_looped(desert):
    r, c = desert.goal
    ring = [(r + dr, c + dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if (dr, dc) != (0, 0)
            and 0 <= r + dr < desert.rows and 0 <= c + dc < desert.cols]
    run = sensor_drive(desert, desert.start, desert.goal, sensor_range=6, prior="map", roadblocks=ring)
    assert not run.found and math.isinf(run.time)


def test_same_inputs_same_drive(grass):
    a, b = pairs(grass)[1]
    one = sensor_drive(grass, a, b, sensor_range=5, roadblocks=[Roadblock((20, 20), 3)])
    two = sensor_drive(grass, a, b, sensor_range=5, roadblocks=[Roadblock((20, 20), 3)])
    assert one.driven == two.driven and one.steps == two.steps and one.dstar_work == two.dstar_work


def test_d_star_lite_repairs_more_cheaply_than_replanning_with_a_short_sensor(desert):
    ratios = [run.astar_work / run.dstar_work for run in
              (sensor_drive(desert, a, b, sensor_range=3) for a, b in pairs(desert))]
    assert statistics.median(ratios) > 1.2


def test_initial_plan_cost_equals_a_star(grass):
    a, b = grass.start, grass.goal
    belief = Belief(grass, "map")
    C = grass.cols
    ds = DStarLite(belief, a[0] * C + a[1], b[0] * C + b[1])
    ds.compute()
    plan = [divmod(i, C) for i in ds.plan()]
    assert plan[0] == a and plan[-1] == b
    assert grass.path_cost(plan) == pytest.approx(astar(grass).cost, abs=1e-9)
    # the optimised D* Lite may stop before expanding the start itself: its rhs holds the cost to go
    assert min(ds.g[ds.start], ds.rhs[ds.start]) == pytest.approx(astar(grass).cost, abs=1e-9)


def test_unknown_prior_is_rejected(grass):
    with pytest.raises(ValueError, match="prior"):
        sensor_drive(grass, prior="psychic")
