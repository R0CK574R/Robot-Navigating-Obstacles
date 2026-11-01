// Driving with a limited sensor: D* Lite re-planning on the drive-time model. A C++ port of robotnav.replan.
//
// The rover believes unsensed ground is flat and open (the cheapest it could be). Every cell it moves to, it senses
// everything within the sensor range, and repairs its plan with D* Lite (Koenig & Likhachev, 2002), which searches
// backwards from the destination and only re-examines the cells whose cost-to-go the news changed. Every run also
// counts what A* would have searched re-planning from scratch each time the plan broke, the baseline.
#pragma once

#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "robotnav/map.hpp"

namespace robotnav {

// What the rover knows before it sets off: nothing (unsensed ground is flat and open), the heights and surfaces
// but none of the obstacles, or the whole map except roadblocks that are not on it.
enum class Prior { None, Elevation, Map };
Prior parse_prior(const std::string& name);         // "none", "elevation" or "map"
const char* prior_name(Prior p);

// An obstacle that is not on the map: it appears at step `appears` (0 = from the start).
struct Roadblock {
    int cell = -1;                                   // flat index
    int appears = 0;
};

// What the rover believes about the terrain: which cells it has sensed and which it thinks are blocked.
struct Belief {
    int R = 0, C = 0, N = 0;
    double cell = 1.0, max_grade = 0.6, uphill = 3.0;
    std::vector<double> H, S;                        // heights, and the surface factor of each cell
    std::vector<char> known, blk;

    Belief(const Map& m, Prior prior);

    // Believed seconds to drive u -> v = u + (dr, dc). A move is priced exactly once both cells are sensed; until
    // then it costs its flat length at the fastest surface, a lower bound on its real cost.
    double cost(int u, int v, int dr, int dc) const {
        if (blk[u] || blk[v]) return kInf;
        double run;
        if (dr && dc) {
            if (blk[u + dr * C] || blk[u + dc]) return kInf;          // no squeezing between two obstacles
            run = kSqrt2 * cell;
        } else {
            run = cell;
        }
        if (!(known[u] && known[v])) return run;                       // unknown ground: flat, open, fastest
        const double rise = H[v] - H[u];
        const double grade = rise / run;
        if (std::fabs(grade) > max_grade) return kInf;
        return std::sqrt(run * run + rise * rise) * S[v] * (1.0 + uphill * std::max(grade, 0.0));
    }

    double h(int a, int b) const {                                     // octile distance in metres
        const int dr = std::abs(a / C - b / C), dc = std::abs(a % C - b % C);
        return ((dr + dc) + (kSqrt2 - 2.0) * std::min(dr, dc)) * cell;
    }
};

// D* Lite, the optimised version (Koenig & Likhachev, Fig. 4). The queue is a binary heap with lazy deletion; ties
// on the key break first in, first out, so the order of expansions is fully determined.
class DStarLite {
public:
    DStarLite(const Belief& b, int start, int goal);

    long compute();                                  // ComputeShortestPath; returns the cells it expanded
    void moved_and_sensed(const std::vector<int>& changed);
    std::vector<int> plan(long limit = 0) const;     // from the rover, always the neighbour with the least cost-to-go
    std::pair<double, int> best_succ(int u) const;   // min over neighbours s of c(u, s) + g(s), first on ties

    int start, goal, last;
    long expanded = 0;                               // cells whose g changed, in total
    double km = 0.0;
    std::vector<double> g, rhs;

private:
    struct Item {
        double k1, k2;
        long stamp;
        int u;
    };
    struct Later {
        bool operator()(const Item& a, const Item& b) const {
            return std::tie(a.k1, a.k2, a.stamp) > std::tie(b.k1, b.k2, b.stamp);
        }
    };
    std::pair<double, double> key(int u) const;
    void push(int u, std::pair<double, double> k);
    const Item* top();
    void pop();
    void update_vertex(int u);

    const Belief& b_;
    std::vector<Item> heap_;                         // a min-heap kept with std::push_heap / std::pop_heap
    std::vector<long> inq_;                          // stamp of a cell's live entry (0: not in the queue)
    long stamp_ = 0;
};

// One step of a drive: the cell reached and what happened there.
struct SensorStep {
    int cell = -1;
    int changed = 0;                                 // cells whose belief changed when sensed
    long work = 0;                                   // cells D* Lite expanded repairing the plan
    bool broke = false;                              // the news made a move on the remaining plan dearer
    long astar = 0;                                  // cells A* from scratch would have searched (when it broke)
    bool has_plan = false;                           // the plan changed here: the new plan
    std::vector<int> plan;
};

struct SensorRun {
    bool found = false;
    std::vector<int> driven;                         // every cell the rover drove through, start to finish
    double time = kInf;                              // drive time of that route (seconds)
    double optimal = kInf;                           // the quickest time with the whole map known (A*)
    std::vector<SensorStep> steps;
    long dstar_work = 0, astar_work = 0;
    int replans = 0, sensed = 0;
    double runtime_ms = 0.0;
    double max_check_error = 0.0;                    // with check: largest |D* Lite plan - A* from scratch| seen
    std::vector<int> first_seen;                     // per cell: the step it was first sensed + 1 (0: never)
    double extra() const { return found && optimal > 0 ? time / optimal - 1.0 : kInf; }
};

// Cell offsets within `range` cells of the rover, row by row.
std::vector<std::pair<int, int>> disk(double range);

// Drive from start to goal seeing only `sensor_range` metres around the rover, re-planning with D* Lite.
// check = true also verifies, each time the plan breaks, that the repaired plan is exactly as quick as A* from
// scratch on the same belief. max_steps = 0 means the default limit (4 x the cells on the map).
SensorRun sensor_drive(const Map& m, int start, int goal, double sensor_range = 10.0, Prior prior = Prior::None,
                       const std::vector<Roadblock>& roadblocks = {}, bool check = false, long max_steps = 0);

}  // namespace robotnav
