#include "robotnav/planners.hpp"

#include <cstdio>

namespace robotnav {

SearchResult bfs(const Map& m, int start, int goal) {
    // Breadth-first search: fewest moves, blind to drive time (robotnav.algorithms.bfs).
    const auto t0 = std::chrono::steady_clock::now();
    const int n = m.size();
    std::vector<int> parent(static_cast<std::size_t>(n), -1), queue;
    std::vector<char> seen(static_cast<std::size_t>(n), 0);
    queue.reserve(static_cast<std::size_t>(n));
    SearchResult out;
    out.algorithm = "BFS";
    out.max_frontier = 1;
    queue.push_back(start);
    seen[start] = 1;
    std::size_t head = 0;
    while (head < queue.size()) {
        const int cur = queue[head++];
        out.explored.push_back(cur);
        if (cur == goal) break;
        m.neighbours(cur, [&](int nxt, double) {
            if (!seen[nxt]) {
                seen[nxt] = 1;
                parent[nxt] = cur;
                queue.push_back(nxt);
            }
        });
        out.max_frontier = std::max(out.max_frontier, static_cast<long>(queue.size() - head));
    }
    out.nodes_expanded = static_cast<long>(out.explored.size());
    if (seen[goal]) {
        out.path = detail::walk_back(parent, goal);
        out.cost = m.path_cost(out.path);
    }
    out.runtime_ms = detail::ms_since(t0);
    return out;
}

SearchResult dijkstra(const Map& m, int start, int goal) {
    return best_first(TerrainGraph{m}, m.size(), start, goal, "Dijkstra", [](double g, double) { return g; });
}

SearchResult astar(const Map& m, int start, int goal) {
    return best_first(TerrainGraph{m}, m.size(), start, goal, "A*", [](double g, double h) { return g + h; });
}

SearchResult weighted_astar(const Map& m, int start, int goal, double w) {
    // Inflated heuristic: faster, and the route takes at most w times the quickest time. Named like Python's
    // f"Weighted A* (w={w:g})".
    char name[48];
    std::snprintf(name, sizeof name, "Weighted A* (w=%g)", w);
    return best_first(TerrainGraph{m}, m.size(), start, goal, name, [w](double g, double h) { return g + w * h; });
}

SearchResult greedy(const Map& m, int start, int goal) {
    return best_first(TerrainGraph{m}, m.size(), start, goal, "Greedy", [](double, double h) { return h; });
}

namespace {

// The terrain as a robot that ignores hills sees it: the same legal moves (obstacles, water and the grade limit
// still stop it), priced by flat distance plus a hair of drive time (robotnav.terrain._HillBlind).
struct HillBlind {
    const Map& m;
    template <class F>
    void neighbours(int u, F&& f) const {
        m.neighbours(u, [&](int v, double step) {
            const bool diag = u / m.cols != v / m.cols && u % m.cols != v % m.cols;
            const double run = (diag ? kSqrt2 : 1.0) * m.cell;
            f(v, run + m.short_tie * step);
        });
    }
    double heuristic(int, int) const { return 0.0; }
    double path_cost(const std::vector<int>& p) const { return m.path_cost(p); }   // what the route really takes
};

}  // namespace

SearchResult shortest_route(const Map& m, int start, int goal) {
    return best_first(HillBlind{m}, m.size(), start, goal, "Shortest route", [](double g, double) { return g; });
}

std::vector<SearchResult> plan_pair(const Map& m, int start, int goal) {
    return {bfs(m, start, goal), dijkstra(m, start, goal), astar(m, start, goal), weighted_astar(m, start, goal),
            greedy(m, start, goal)};
}

}  // namespace robotnav
