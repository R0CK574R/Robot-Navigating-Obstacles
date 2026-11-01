// The map the rover drives on, and its drive-time model: a C++ port of robotnav.terrain.Terrain.
//
// Every number here is computed with the same operations, in the same order, as the Python reference, and the
// build turns off fused multiply-add (-ffp-contract=off), so results agree with Python to the last bit.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace robotnav {

inline const double kSqrt2 = std::sqrt(2.0);                  // == Python's math.sqrt(2) (IEEE sqrt is exact)
inline constexpr double kInf = std::numeric_limits<double>::infinity();

// The eight moves in the planners' order (robotnav.grid.EIGHT): up, down, left, right, then the diagonals.
inline constexpr int NB_R[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
inline constexpr int NB_C[8] = {0, 0, -1, 1, -1, 1, -1, 1};

enum Obstacle : int { OPEN = 0, ROCK = 1, TREE = 2, WATER = 3 };

struct Pond {
    double level = 0.0;                                       // water surface (metres)
    std::vector<int> cells;                                   // flat indices
};

struct Preset {
    int start = -1, goal = -1;                                // flat indices
    std::string name;                                         // "NW → SE"
};

struct Map {
    std::string theme;                                        // "grassland", "desert" or "island"
    long seed = 0;
    int rows = 0, cols = 0;
    std::vector<double> height;                               // metres, row-major (rows * cols)
    std::vector<int> surface;                                 // index into surface_cost
    std::vector<double> surface_cost;                         // time factor of each surface, never below 1
    std::vector<std::string> surface_names;
    std::vector<int> obstacle;                                // Obstacle per cell
    double cell = 1.0;                                        // metres per cell
    double max_grade = 0.6;                                   // steepest drivable rise / run
    double uphill = 3.0;                                      // extra time per unit of uphill grade
    double short_tie = 1e-6;                                  // the hill-blind route's hair of drive time
    std::optional<double> sea;                                // island: the sea's surface
    std::vector<Pond> ponds;                                  // desert: the oases
    int start = -1, goal = -1;                                // the default flags (flat indices)
    std::vector<Preset> presets;                              // the 20 preset trips, quickest first

    int size() const { return rows * cols; }
    int at(int r, int c) const { return r * cols + c; }
    bool inside(int r, int c) const { return r >= 0 && c >= 0 && r < rows && c < cols; }
    bool blocked(int i) const { return obstacle[i] != OPEN; }

    // Seconds to drive a -> b (b one of a's eight neighbours), or -1 if the slope is steeper than max_grade.
    // time = 3-D distance x surface factor x (1 + uphill x uphill grade), the distance as sqrt(run^2 + rise^2).
    double edge_cost(int a, int b) const {
        const int ar = a / cols, ac = a % cols, br = b / cols, bc = b % cols;
        const double run = ((ar != br && ac != bc) ? kSqrt2 : 1.0) * cell;
        const double rise = height[b] - height[a];
        const double grade = rise / run;
        if (std::fabs(grade) > max_grade) return -1.0;
        return std::sqrt(run * run + rise * rise) * surface_cost[surface[b]] * (1.0 + uphill * std::max(grade, 0.0));
    }

    // Octile distance in metres: every move takes at least its flat length at 1 m/s, so it never overestimates.
    double heuristic(int a, int b) const {
        const int dr = std::abs(a / cols - b / cols), dc = std::abs(a % cols - b % cols);
        return ((dr + dc) + (kSqrt2 - 2.0) * std::min(dr, dc)) * cell;
    }

    // The legal moves out of cell a, in the planners' order: f(neighbour, seconds). No driving into an obstacle,
    // no squeezing diagonally between two obstacles, nothing steeper than the grade limit.
    template <class F>
    void neighbours(int a, F&& f) const {
        const int r = a / cols, c = a % cols;
        for (int k = 0; k < 8; ++k) {
            const int nr = r + NB_R[k], nc = c + NB_C[k];
            if (!inside(nr, nc)) continue;
            const int n = nr * cols + nc;
            if (blocked(n)) continue;
            if (NB_R[k] && NB_C[k] && (blocked((r + NB_R[k]) * cols + c) || blocked(r * cols + c + NB_C[k]))) continue;
            const double cost = edge_cost(a, n);
            if (cost < 0.0) continue;
            f(n, cost);
        }
    }

    // Drive time of a route, added up left to right (robotnav.terrain.add_up, and the browser, add the same way).
    double path_cost(const std::vector<int>& path) const {
        double total = 0.0;
        for (std::size_t k = 1; k < path.size(); ++k) total += edge_cost(path[k - 1], path[k]);
        return total;
    }

    // Water surface per cell (NaN on land): the sea, then the ponds, as the Python and the viewer build it.
    std::vector<double> water_levels() const;
};

// Distance, climb and descent of a route, and how much of it is on slow ground (robotnav.terrain.path_stats).
struct PathStats {
    double distance_m = 0.0, climb_m = 0.0, descent_m = 0.0, max_grade_pct = 0.0, rough_m = 0.0;
};
PathStats path_stats(const Map& m, const std::vector<int>& path);

// Python's round(x, digits): the nearest double to x rounded to `digits` decimals (ties to even).
double py_round(double x, int digits);

}  // namespace robotnav
