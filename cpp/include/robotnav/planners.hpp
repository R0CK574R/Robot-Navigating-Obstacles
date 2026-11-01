// The five planners and the hill-blind shortest route: a C++ port of robotnav.algorithms and robotnav.terrain.
//
// Same neighbour order, same priorities, and the same tie-breaking (first in, first out), so every planner
// expands the same cells in the same order as the Python reference and finds the same route.
#pragma once

#include <chrono>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "robotnav/map.hpp"

namespace robotnav {

struct SearchResult {
    std::string algorithm;
    std::vector<int> path;              // flat indices, start to goal; empty if there is no route
    double cost = kInf;                 // drive time of the route (seconds)
    long nodes_expanded = 0;            // cells taken off the frontier ("searched")
    long max_frontier = 0;              // peak frontier size, stale entries included (a memory proxy)
    double runtime_ms = 0.0;
    std::vector<int> explored;          // the cells in the order they were expanded
    bool found() const { return !path.empty(); }
};

namespace detail {

inline double ms_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

inline std::vector<int> walk_back(const std::vector<int>& parent, int goal) {
    std::vector<int> path;
    for (int v = goal; v != -1; v = parent[v]) path.push_back(v);
    return {path.rbegin(), path.rend()};
}

// One frontier entry. Python's heapq compares (priority, tie, cell) tuples and every tie number is unique, so the
// order entries come off is fixed by (priority, tie) alone, whatever the heap's internal layout.
struct Entry {
    double f;
    long tie;
    int v;
};
struct Later {
    bool operator()(const Entry& a, const Entry& b) const { return a.f > b.f || (a.f == b.f && a.tie > b.tie); }
};

}  // namespace detail

// The engine behind Dijkstra, A*, Weighted A*, Greedy and the shortest route (robotnav.algorithms._best_first).
//   graph.neighbours(u, f)  calls f(v, step) for each legal move, in neighbour order
//   graph.heuristic(a, b)   the estimate from a to b
//   graph.path_cost(path)   what the route really costs
//   priority(g, h)          the frontier order: g (Dijkstra), g + h (A*), g + w*h (Weighted A*), h (Greedy)
template <class Graph, class Priority>
SearchResult best_first(const Graph& graph, int n_cells, int start, int goal, std::string name, Priority priority) {
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<double> g(static_cast<std::size_t>(n_cells), kInf);
    std::vector<int> parent(static_cast<std::size_t>(n_cells), -1);
    std::vector<char> closed(static_cast<std::size_t>(n_cells), 0);
    std::priority_queue<detail::Entry, std::vector<detail::Entry>, detail::Later> frontier;
    long tie = 0;
    SearchResult out;
    out.algorithm = std::move(name);
    out.max_frontier = 1;
    g[start] = 0.0;
    frontier.push({priority(0.0, graph.heuristic(start, goal)), tie++, start});
    while (!frontier.empty()) {
        const int cur = frontier.top().v;
        frontier.pop();
        if (closed[cur]) continue;                                  // stale duplicate entry
        closed[cur] = 1;
        out.explored.push_back(cur);
        if (cur == goal) break;
        graph.neighbours(cur, [&](int nxt, double step) {
            const double new_g = g[cur] + step;
            if (new_g < g[nxt]) {                                   // also "not seen yet": g starts at infinity
                g[nxt] = new_g;
                parent[nxt] = cur;
                frontier.push({priority(new_g, graph.heuristic(nxt, goal)), tie++, nxt});
            }
        });
        out.max_frontier = std::max(out.max_frontier, static_cast<long>(frontier.size()));
    }
    out.nodes_expanded = static_cast<long>(out.explored.size());
    if (closed[goal]) {
        out.path = detail::walk_back(parent, goal);
        out.cost = graph.path_cost(out.path);
    }
    out.runtime_ms = detail::ms_since(t0);
    return out;
}

// The terrain as the planners see it: drive time per move, octile distance as the estimate.
struct TerrainGraph {
    const Map& m;
    template <class F>
    void neighbours(int u, F&& f) const { m.neighbours(u, f); }
    double heuristic(int a, int b) const { return m.heuristic(a, b); }
    double path_cost(const std::vector<int>& p) const { return m.path_cost(p); }
};

SearchResult bfs(const Map& m, int start, int goal);
SearchResult dijkstra(const Map& m, int start, int goal);
SearchResult astar(const Map& m, int start, int goal);
SearchResult weighted_astar(const Map& m, int start, int goal, double w = 2.0);
SearchResult greedy(const Map& m, int start, int goal);

// The shortest drivable route, as a robot that ignores hills picks it: Dijkstra on flat distance plus a hair
// (short_tie) of drive time per move. Its cost is the drive time that route really takes.
SearchResult shortest_route(const Map& m, int start, int goal);

// Every planner from start to goal, in the order the viewer's board lists them: BFS, Dijkstra, A*, Weighted A*,
// Greedy (robotnav.terrain.plan_pair).
std::vector<SearchResult> plan_pair(const Map& m, int start, int goal);

}  // namespace robotnav
