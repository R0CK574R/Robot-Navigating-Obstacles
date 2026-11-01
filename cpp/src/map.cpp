#include "robotnav/map.hpp"

#include <cstdio>
#include <cstdlib>

namespace robotnav {

std::vector<double> Map::water_levels() const {
    std::vector<double> wl(static_cast<std::size_t>(size()), std::nan(""));
    if (sea)
        for (int i = 0; i < size(); ++i)
            if (obstacle[i] == WATER) wl[i] = *sea;
    for (const Pond& p : ponds)
        for (int i : p.cells) wl[i] = p.level;
    return wl;
}

double py_round(double x, int digits) {
    // Python's round(x, n) is correctly rounded (ties to even on the exact binary value), and so is printf's %.nf
    // (glibc, and the BSD libc on macOS), so printing and reading back gives the very same double.
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", digits, x);
    return std::strtod(buf, nullptr);
}

PathStats path_stats(const Map& m, const std::vector<int>& path) {
    PathStats out;
    if (path.size() < 2) return out;
    double dist = 0.0, climb = 0.0, descent = 0.0, rough = 0.0, max_grade = 0.0;
    for (std::size_t k = 1; k < path.size(); ++k) {
        const int a = path[k - 1], b = path[k];
        const bool diag = a / m.cols != b / m.cols && a % m.cols != b % m.cols;
        const double run = (diag ? kSqrt2 : 1.0) * m.cell;
        const double rise = m.height[b] - m.height[a];
        const double seg = std::sqrt(run * run + rise * rise);
        dist += seg;
        climb += std::max(rise, 0.0);
        descent += std::max(-rise, 0.0);
        max_grade = std::max(max_grade, std::fabs(rise) / run);
        if (m.surface_cost[m.surface[b]] > 1.0) rough += seg;          // any surface slower than the best
    }
    out.distance_m = py_round(dist, 1);
    out.climb_m = py_round(climb, 1);
    out.descent_m = py_round(descent, 1);
    out.max_grade_pct = py_round(100.0 * max_grade, 1);
    out.rough_m = py_round(rough, 1);
    return out;
}

}  // namespace robotnav
