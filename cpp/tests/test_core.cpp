// Unit tests for the C++ core on small hand-made maps (no test framework needed: ctest runs this executable).
// The number-for-number comparison with the Python reference on the real maps is tests/test_cpp.py.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "robotnav/airspace.hpp"
#include "robotnav/mapio.hpp"
#include "robotnav/planners.hpp"
#include "robotnav/replan.hpp"

using namespace robotnav;

namespace {

int g_checks = 0, g_failures = 0;

#define CHECK(cond)                                                                       \
    do {                                                                                  \
        ++g_checks;                                                                       \
        if (!(cond)) {                                                                    \
            ++g_failures;                                                                 \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                                 \
    } while (0)

#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

// A flat, open map: every move costs its length (1 m/s on the only surface).
Map flat_map(int rows, int cols) {
    Map m;
    m.theme = "test";
    m.rows = rows;
    m.cols = cols;
    m.height.assign(static_cast<std::size_t>(rows) * cols, 0.0);
    m.surface.assign(static_cast<std::size_t>(rows) * cols, 0);
    m.obstacle.assign(static_cast<std::size_t>(rows) * cols, OPEN);
    m.surface_cost = {1.0, 3.0};
    m.surface_names = {"Grass", "Mud"};
    m.start = 0;
    m.goal = rows * cols - 1;
    return m;
}

bool legal_route(const Map& m, const std::vector<int>& path) {
    for (std::size_t k = 1; k < path.size(); ++k) {
        bool ok = false;
        m.neighbours(path[k - 1], [&](int v, double) { ok = ok || v == path[k]; });
        if (!ok) return false;
    }
    return true;
}

void test_open_ground() {
    const Map m = flat_map(6, 6);
    const int s = m.at(5, 0), g = m.at(0, 5);
    const SearchResult a = astar(m, s, g), d = dijkstra(m, s, g), b = bfs(m, s, g);
    CHECK(a.found() && d.found() && b.found());
    CHECK(a.path.size() == 6 && b.path.size() == 6);          // five diagonal moves
    CHECK_NEAR(a.cost, 5 * kSqrt2, 1e-12);                      // each step is sqrt(2) m at 1 m/s
    CHECK(a.cost == d.cost);
    CHECK(a.nodes_expanded <= d.nodes_expanded);                // the estimate saves work
    CHECK(a.path.front() == s && a.path.back() == g);
    CHECK_NEAR(m.heuristic(s, g), a.cost, 1e-12);               // octile distance is exact on open flat ground
}

void test_straight_up() {
    // From the bottom-left corner straight up the side: the quickest route, and the first cell expanded is the start.
    const Map m = flat_map(3, 3);
    const SearchResult a = astar(m, m.at(2, 0), m.at(0, 0));
    CHECK((a.path == std::vector<int>{m.at(2, 0), m.at(1, 0), m.at(0, 0)}));
    CHECK(a.explored.front() == m.at(2, 0));
}

void test_wall_with_a_gap() {
    Map m = flat_map(7, 7);
    for (int r = 0; r < 6; ++r) m.obstacle[m.at(r, 3)] = ROCK;    // a wall down column 3, open only at the bottom
    const SearchResult a = astar(m, m.at(0, 0), m.at(0, 6)), d = dijkstra(m, m.at(0, 0), m.at(0, 6));
    CHECK(a.found());
    CHECK(std::find(a.path.begin(), a.path.end(), m.at(6, 3)) != a.path.end());
    CHECK(a.cost == d.cost);
    CHECK(legal_route(m, a.path));
    for (const SearchResult& r : plan_pair(m, m.at(0, 0), m.at(0, 6))) {
        CHECK(r.found());
        CHECK(legal_route(m, r.path));
        CHECK(r.cost >= a.cost - 1e-9);                             // nothing beats A*
    }
}

void test_no_squeezing_between_obstacles() {
    Map m = flat_map(3, 3);
    m.obstacle[m.at(0, 1)] = ROCK;
    m.obstacle[m.at(1, 0)] = TREE;
    CHECK(!astar(m, m.at(0, 0), m.at(1, 1)).found());               // the only way out is the diagonal squeeze
    CHECK(!bfs(m, m.at(0, 0), m.at(2, 2)).found());
}

void test_grade_limit_and_uphill_time() {
    Map m = flat_map(1, 3);
    m.height = {0.0, 1.0, 2.0};                                     // 100% grade: too steep
    CHECK(m.edge_cost(0, 1) < 0);
    CHECK(!astar(m, 0, 2).found());
    m.height = {0.0, 0.5, 1.0};                                     // 50% grade: drivable, slow uphill
    const double up = std::sqrt(1.0 * 1.0 + 0.5 * 0.5) * 1.0 * (1.0 + 3.0 * 0.5);
    CHECK(m.edge_cost(0, 1) == up);
    CHECK(m.edge_cost(1, 0) == std::sqrt(1.0 * 1.0 + 0.5 * 0.5));  // downhill: no penalty
    const SearchResult a = astar(m, 0, 2);
    CHECK(a.found() && a.cost == up + up);
}

void test_hills_and_mud_shape_the_quickest_route() {
    // A mud strip across the map with one firm crossing just beside the straight line: A* steps across on the firm
    // ground, the hill-blind route (shortest distance) wades straight through the mud, and takes longer.
    Map m = flat_map(9, 9);
    for (int c = 0; c < 9; ++c) m.surface[m.at(4, c)] = 1;
    m.surface[m.at(4, 1)] = 0;
    const int s = m.at(8, 0), g = m.at(0, 0);
    const SearchResult quick = astar(m, s, g), blind = shortest_route(m, s, g);
    CHECK(quick.found() && blind.found());
    CHECK(blind.path.size() == 9);                                  // straight up through the mud
    CHECK(quick.cost < blind.cost);
    CHECK_NEAR(quick.cost, 6 + 2 * kSqrt2, 1e-12);
    CHECK(blind.cost == 7.0 + 3.0);
    CHECK(blind.cost == m.path_cost(blind.path));                   // its cost is the time it really takes
    const PathStats st = path_stats(m, blind.path);
    CHECK(st.distance_m == 8.0 && st.rough_m == 1.0 && st.climb_m == 0.0);
}

void test_python_rounding() {
    CHECK(py_round(2.25, 1) == 2.2);                                // an exact tie goes to the even digit
    CHECK(py_round(0.125, 2) == 0.12);
    CHECK(py_round(2.675, 2) == 2.67);                              // 2.675 is really 2.67499999...
    CHECK(py_round(76.04, 1) == 76.0);
}

void test_sensor_disk() {
    CHECK(disk(1.5).size() == 9);
    CHECK(disk(2.0).size() == 13);
    CHECK(disk(5.0).size() == 81);
}

void test_dstar_lite_drives() {
    Map m = flat_map(15, 15);
    for (int r = 2; r < 15; ++r) m.obstacle[m.at(r, 7)] = ROCK;    // a wall the rover only finds when it gets close
    for (int k = 0; k < m.size(); ++k) m.height[k] = 0.05 * (k % 15);
    const int s = m.at(14, 0), g = m.at(14, 14);
    const SensorRun known = sensor_drive(m, s, g, 3.0, Prior::Map);
    CHECK(known.found && known.replans == 0);                   // a full map: the quickest route, no surprises
    CHECK_NEAR(known.time, known.optimal, 1e-9);
    for (Prior p : {Prior::None, Prior::Elevation}) {
        const SensorRun run = sensor_drive(m, s, g, 3.0, p, {}, true);
        CHECK(run.found);
        CHECK(run.driven.front() == s && run.driven.back() == g);
        CHECK(legal_route(m, run.driven));
        CHECK(run.time >= run.optimal - 1e-9);
        CHECK(run.replans > 0);
        CHECK(run.max_check_error < 1e-9);                          // every repaired plan was as quick as A*
        CHECK(run.steps.size() == run.driven.size());
    }
    // A roadblock on the quickest route, known to nobody: the rover finds it and drives round.
    const int block = known.driven[known.driven.size() / 2];
    const SensorRun around = sensor_drive(m, s, g, 3.0, Prior::Map, {{block, 0}}, true);
    CHECK(around.found && around.replans >= 1 && around.time >= known.time - 1e-9);
    CHECK(std::find(around.driven.begin(), around.driven.end(), block) == around.driven.end());
    // Walled off: no way through.
    Map shut = flat_map(5, 5);
    for (int r = 0; r < 5; ++r) shut.obstacle[shut.at(r, 2)] = WATER;
    CHECK(!sensor_drive(shut, shut.at(0, 0), shut.at(0, 4), 3.0).found);
}

void test_airspace_and_terrain_masking() {
    Map m = flat_map(40, 40);
    for (int r = 0; r < 40; ++r) m.height[m.at(r, 20)] = m.height[m.at(r, 21)] = 9.0;   // a ridge down the middle
    const std::vector<std::pair<int, int>> towers{{20, 10}};
    const Airspace air = make_airspace(m, towers);
    CHECK(air.L == AIR.levels && air.alt.front() == 0.0 + 1 * AIR.dz);
    CHECK(air.eyes.front() == 0.0 + AIR.eye);
    const auto voxel = [&](int k, int r, int c) { return (static_cast<std::size_t>(k) * 40 + r) * 40 + c; };
    CHECK(air.seen[voxel(0, 20, 15)]);                              // open ground in front of the tower
    CHECK(!air.seen[voxel(0, 20, 24)]);                             // low behind the ridge: hidden
    CHECK(!air.free[voxel(0, 20, 20)]);                             // the ridge itself is solid at the lowest level
    const Flight st = fly(air, m.at(5, 2), m.at(35, 2), true), di = fly(air, m.at(5, 2), m.at(35, 2), false);
    CHECK(st.found && di.found);
    CHECK(st.seen <= di.seen + 1e-9 && st.time >= di.time - 1e-9);
    CHECK(di.seen > 0.0);                                           // the direct route flies past the tower
    const Flight dj = fly(air, m.at(5, 2), m.at(35, 2), true, false);
    CHECK_NEAR(dj.cost, st.cost, 1e-9);                             // A* agrees with Dijkstra, with less work
    CHECK(dj.nodes >= st.nodes);
    const auto picked = place_towers(m, 2, 30.0);
    CHECK(picked.size() == 2);
    CHECK((picked[0].first - picked[1].first) * (picked[0].first - picked[1].first) +
              (picked[0].second - picked[1].second) * (picked[0].second - picked[1].second) >= 900);
}

void test_map_file_round_trip() {
    Map m = flat_map(4, 5);
    m.theme = "island";
    m.seed = 7;
    m.height = {0.1, 0.2, 1.0 / 3.0, 2.5, -1.25, 6.02e23, 1e-300, 0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    m.obstacle[m.at(3, 4)] = WATER;
    m.sea = 0.6;
    m.ponds.push_back({0.25, {m.at(2, 2), m.at(2, 3)}});
    m.presets.push_back({m.at(0, 0), m.at(3, 3), "NW → SE"});
    const std::string path = "robotnav_test_roundtrip.map";
    save_map(m, path);
    const Map r = load_map(path);
    std::remove(path.c_str());
    CHECK(r.theme == m.theme && r.seed == m.seed && r.rows == m.rows && r.cols == m.cols);
    CHECK(r.height == m.height);                                    // every double back exactly
    CHECK(r.surface == m.surface && r.obstacle == m.obstacle);
    CHECK(r.surface_cost == m.surface_cost && r.surface_names == m.surface_names);
    CHECK(r.sea && *r.sea == 0.6);
    CHECK(r.ponds.size() == 1 && r.ponds[0].level == 0.25 && r.ponds[0].cells == m.ponds[0].cells);
    CHECK(r.presets.size() == 1 && r.presets[0].name == "NW → SE" && r.presets[0].goal == m.at(3, 3));
    CHECK(r.start == m.start && r.goal == m.goal);
}

}  // namespace

int main() {
    const std::vector<std::pair<const char*, std::function<void()>>> tests = {
        {"open ground", test_open_ground},
        {"straight up", test_straight_up},
        {"wall with a gap", test_wall_with_a_gap},
        {"no squeezing between obstacles", test_no_squeezing_between_obstacles},
        {"grade limit and uphill time", test_grade_limit_and_uphill_time},
        {"hills and mud shape the quickest route", test_hills_and_mud_shape_the_quickest_route},
        {"Python rounding", test_python_rounding},
        {"sensor disk", test_sensor_disk},
        {"D* Lite drives", test_dstar_lite_drives},
        {"airspace and terrain masking", test_airspace_and_terrain_masking},
        {"map file round trip", test_map_file_round_trip},
    };
    for (const auto& [name, run] : tests) {
        const int before = g_failures;
        run();
        std::printf("%-42s %s\n", name, g_failures == before ? "ok" : "FAILED");
    }
    std::printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
