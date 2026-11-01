"""The air route: free airspace clears the ground, hills hide the drone, routes are legal and optimal, stealth hides."""
import math

import numpy as np
import pytest

from robotnav.airspace import AIR, MOVES, fly, ground_top, make_airspace, move_ok, place_towers
from robotnav.scene3d import map_scene
from robotnav.terrain import Terrain, solvable_map


@pytest.fixture(scope="module")
def desert():
    return solvable_map("desert", seed=3, size=48)


@pytest.fixture(scope="module")
def air(desert):
    return make_airspace(desert, towers=place_towers(desert, n=3, spacing=18))


def pairs(t):
    return [(t.start, t.goal), ((t.rows - 6, 6), (6, t.cols // 2)), ((t.rows // 2, 4), (t.rows // 2, t.cols - 5))]


def unpack(air, v):
    L, R, C = air.free.shape
    k, rem = divmod(v, R * C)
    return k, *divmod(rem, C)


def test_free_air_clears_the_ground_and_the_top_level_is_open(desert, air):
    top = ground_top(desert)
    for k, a in enumerate(air.alt):
        assert (a >= top[air.free[k]] + AIR["clear"]).all()
        assert (a < top[~air.free[k]] + AIR["clear"]).all()
    assert air.free[-1].all()


def test_towers_stand_on_open_high_ground_far_apart():
    t = solvable_map("grassland", seed=3)
    tw = place_towers(t)
    assert len(tw) == AIR["towers"]
    assert all(t.obstacle[r, c] == 0 for r, c in tw)
    assert all(math.dist(a, b) >= AIR["spacing"] for i, a in enumerate(tw) for b in tw[i + 1:])
    assert t.height[tw[0]] == max(t.height[r, c] for r in range(8, 88) for c in range(8, 88) if t.obstacle[r, c] == 0)


def flat_world(ridge=12.0):
    """A flat 40 x 40 field with a north-south ridge at columns 15-16."""
    h = np.zeros((40, 40))
    h[:, 15:17] = ridge
    z = np.zeros((40, 40), dtype=np.int8)
    return Terrain(blocked=np.zeros((40, 40), bool), start=(20, 2), goal=(20, 37), diagonal=True, height=h,
                   surface=z, obstacle=z, surface_costs=(1.0,), surface_names=("Grass",))


def test_a_ridge_hides_a_low_drone_but_not_a_high_one():
    air = make_airspace(flat_world(), towers=[(20, 8)])
    low = next(k for k in range(len(air.alt)) if air.free[k, 20, 24])
    assert not air.seen[low, 20, 24]                          # behind the ridge, low down: hidden
    assert any(air.seen[k, 20, 24] for k in range(low + 1, len(air.alt)))   # high above it: in sight
    assert air.seen[low, 20, 12]                              # in front of the ridge: in sight
    no_ridge = make_airspace(flat_world(ridge=0.0), towers=[(20, 8)])
    assert no_ridge.seen[low, 20, 24]


def test_out_of_reach_is_out_of_sight():
    air = make_airspace(flat_world(ridge=0.0), towers=[(20, 8)])
    far = [(k, r, c) for k in range(len(air.alt)) for r in range(40) for c in range(40)
           if (r - 20) ** 2 + (c - 8) ** 2 + (air.alt[k] - air.eyes[0]) ** 2 > AIR["sight"] ** 2]
    assert far and not any(air.seen[v] for v in far)


@pytest.mark.parametrize("stealth", [True, False])
def test_routes_take_off_and_land_and_every_move_is_legal(desert, air, stealth):
    for a, b in pairs(desert):
        f = fly(air, a, b, stealth)
        assert f.found and f.voxels[0] == air.voxel(a) and f.voxels[-1] == air.voxel(b)
        for u, v in zip(f.voxels, f.voxels[1:]):
            (k0, r0, c0), (k1, r1, c1) = unpack(air, u), unpack(air, v)
            assert (r1 - r0, c1 - c0, k1 - k0) in MOVES
            assert move_ok(air, r0, c0, k0, r1 - r0, c1 - c0, k1 - k0)


def test_stealth_hides_and_direct_is_quicker(desert, air):
    for a, b in pairs(desert):
        st, di = fly(air, a, b, True), fly(air, a, b, False)
        assert st.seen <= di.seen + 1e-9 and di.time <= st.time + 1e-9
        assert st.cost <= di.time + AIR["exposure"] * di.seen + 1e-9   # stealth is optimal for what it weighs
        assert di.cost == pytest.approx(di.time, abs=1e-9)


def test_a_star_matches_dijkstra(desert, air):
    for stealth in (True, False):
        a, b = pairs(desert)[0]
        astar, dij = fly(air, a, b, stealth), fly(air, a, b, stealth, heuristic=False)
        assert astar.cost == pytest.approx(dij.cost, abs=1e-9)
        assert astar.nodes <= dij.nodes


def test_the_viewer_gets_the_towers(desert):
    sc = map_scene(desert)
    assert sc["towers"] and all(len(c) == 2 for c in sc["towers"])
