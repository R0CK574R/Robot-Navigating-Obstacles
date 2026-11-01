#include "robotnav/replan.hpp"

#include <algorithm>
#include <climits>
#include <stdexcept>

#include "robotnav/planners.hpp"

namespace robotnav {

Prior parse_prior(const std::string& name) {
    if (name == "none") return Prior::None;
    if (name == "elevation") return Prior::Elevation;
    if (name == "map") return Prior::Map;
    throw std::invalid_argument("prior must be one of none, elevation, map (not '" + name + "')");
}

const char* prior_name(Prior p) {
    switch (p) {
        case Prior::None: return "none";
        case Prior::Elevation: return "elevation";
        default: return "map";
    }
}

Belief::Belief(const Map& m, Prior prior)
    : R(m.rows), C(m.cols), N(m.size()), cell(m.cell), max_grade(m.max_grade), uphill(m.uphill), H(m.height),
      S(static_cast<std::size_t>(N)), known(static_cast<std::size_t>(N), prior != Prior::None),
      blk(static_cast<std::size_t>(N), 0) {
    for (int i = 0; i < N; ++i) S[i] = m.surface_cost[m.surface[i]];
    if (prior == Prior::Map)
        for (int i = 0; i < N; ++i) blk[i] = m.blocked(i);
}

// ------------------------------------------------------------------------------------------------ D* Lite

namespace {
// Python compares the keys as tuples: the first entries, and the second ones on a tie.
inline bool key_less(std::pair<double, double> a, std::pair<double, double> b) {
    return a.first < b.first || (a.first == b.first && a.second < b.second);
}
}  // namespace

DStarLite::DStarLite(const Belief& b, int start_, int goal_)
    : start(start_), goal(goal_), last(start_), g(static_cast<std::size_t>(b.N), kInf),
      rhs(static_cast<std::size_t>(b.N), kInf), b_(b), inq_(static_cast<std::size_t>(b.N), 0) {
    rhs[goal] = 0.0;
    push(goal, {b.h(start, goal), 0.0});
}

void DStarLite::push(int u, std::pair<double, double> k) {
    inq_[u] = ++stamp_;
    heap_.push_back({k.first, k.second, stamp_, u});
    std::push_heap(heap_.begin(), heap_.end(), Later{});
}

void DStarLite::pop() {
    std::pop_heap(heap_.begin(), heap_.end(), Later{});
    heap_.pop_back();
}

const DStarLite::Item* DStarLite::top() {
    while (!heap_.empty() && inq_[heap_.front().u] != heap_.front().stamp) pop();   // stale entry
    return heap_.empty() ? nullptr : &heap_.front();
}

std::pair<double, double> DStarLite::key(int u) const {
    const double m = std::min(g[u], rhs[u]);
    return {m + b_.h(start, u) + km, m};
}

void DStarLite::update_vertex(int u) {
    if (g[u] != rhs[u])
        push(u, key(u));                     // insert, or update (the old entry goes stale)
    else if (inq_[u])
        inq_[u] = 0;                         // remove
}

std::pair<double, int> DStarLite::best_succ(int u) const {
    const int C = b_.C, R = b_.R, r = u / C, c = u % C;
    double best = kInf;
    int arg = -1;
    for (int k = 0; k < 8; ++k) {
        const int dr = NB_R[k], dc = NB_C[k], nr = r + dr, nc = c + dc;
        if (nr < 0 || nc < 0 || nr >= R || nc >= C) continue;
        const int s = u + dr * C + dc;
        const double v = b_.cost(u, s, dr, dc) + g[s];
        if (v < best) {
            best = v;
            arg = s;
        }
    }
    return {best, arg};
}

long DStarLite::compute() {
    const int C = b_.C, R = b_.R;
    const long before = expanded;
    for (;;) {
        const Item* t = top();
        const auto ks = key(start);
        const std::pair<double, double> k_old = t ? std::make_pair(t->k1, t->k2) : std::make_pair(kInf, kInf);
        if (!(key_less(k_old, ks) || rhs[start] > g[start]) || !t) break;
        const int u = t->u;                  // read before any push moves the heap
        const auto k_new = key(u);
        if (key_less(k_old, k_new)) {
            push(u, k_new);
            continue;
        }
        const int r = u / C, c = u % C;
        if (g[u] > rhs[u]) {
            g[u] = rhs[u];
            inq_[u] = 0;
            ++expanded;
            for (int k = 0; k < 8; ++k) {    // predecessors p = u - (dr, dc); the move p -> u is (dr, dc)
                const int dr = NB_R[k], dc = NB_C[k], pr = r - dr, pc = c - dc;
                if (pr < 0 || pc < 0 || pr >= R || pc >= C) continue;
                const int p = u - dr * C - dc;
                if (p != goal) {
                    const double v = b_.cost(p, u, dr, dc) + g[u];
                    if (v < rhs[p]) rhs[p] = v;
                }
                update_vertex(p);
            }
        } else {
            const double g_old = g[u];
            g[u] = kInf;
            ++expanded;
            for (int k = 0; k < 8; ++k) {
                const int dr = NB_R[k], dc = NB_C[k], pr = r - dr, pc = c - dc;
                if (pr < 0 || pc < 0 || pr >= R || pc >= C) continue;
                const int p = u - dr * C - dc;
                if (p != goal && rhs[p] == b_.cost(p, u, dr, dc) + g_old) rhs[p] = best_succ(p).first;
                update_vertex(p);
            }
            update_vertex(u);
        }
    }
    return expanded - before;
}

void DStarLite::moved_and_sensed(const std::vector<int>& changed) {
    if (changed.empty()) return;
    km += b_.h(last, start);
    last = start;
    const int C = b_.C, R = b_.R;
    std::vector<int> touched;                // moves out of x, into x, and diagonals that squeeze past x
    for (int x : changed) {
        const int r = x / C, c = x % C;
        touched.push_back(x);
        for (int k = 0; k < 8; ++k) {
            const int nr = r + NB_R[k], nc = c + NB_C[k];
            if (nr >= 0 && nr < R && nc >= 0 && nc < C) touched.push_back(nr * C + nc);
        }
    }
    std::sort(touched.begin(), touched.end());
    touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
    for (int u : touched) {
        if (u != goal) rhs[u] = best_succ(u).first;
        update_vertex(u);
    }
}

std::vector<int> DStarLite::plan(long limit) const {
    std::vector<int> out{start};
    int u = start;
    if (limit <= 0) limit = b_.N;
    while (u != goal && static_cast<long>(out.size()) <= limit) {
        const auto [best, next] = best_succ(u);
        if (best == kInf) return {};
        u = next;
        out.push_back(u);
    }
    return u == goal ? out : std::vector<int>{};
}

// ------------------------------------------------------------------------------------------------ the drive

namespace {

// The belief as a graph for the planners' engine: the A*-from-scratch baseline (robotnav.replan._BeliefGraph).
struct BeliefGraph {
    const Belief& b;
    template <class F>
    void neighbours(int u, F&& f) const {
        const int r = u / b.C, c = u % b.C;
        for (int k = 0; k < 8; ++k) {
            const int dr = NB_R[k], dc = NB_C[k], nr = r + dr, nc = c + dc;
            if (nr >= 0 && nr < b.R && nc >= 0 && nc < b.C) {
                const double cost = b.cost(u, nr * b.C + nc, dr, dc);
                if (cost < kInf) f(nr * b.C + nc, cost);
            }
        }
    }
    double heuristic(int a, int z) const { return b.h(a, z); }
    double path_cost(const std::vector<int>& p) const {
        double total = 0.0;
        for (std::size_t k = 1; k < p.size(); ++k)
            total += b.cost(p[k - 1], p[k], p[k] / b.C - p[k - 1] / b.C, p[k] % b.C - p[k - 1] % b.C);
        return total;
    }
};

}  // namespace

std::vector<std::pair<int, int>> disk(double range) {
    const int n = static_cast<int>(std::floor(range));
    std::vector<std::pair<int, int>> out;
    for (int dr = -n; dr <= n; ++dr)
        for (int dc = -n; dc <= n; ++dc)
            if (dr * dr + dc * dc <= range * range) out.emplace_back(dr, dc);
    return out;
}

SensorRun sensor_drive(const Map& m, int si, int gi, double sensor_range, Prior prior,
                       const std::vector<Roadblock>& roadblocks, bool check, long max_steps) {
    const auto t0 = std::chrono::steady_clock::now();
    const int C = m.cols, R = m.rows, N = m.size();
    std::vector<int> appears(static_cast<std::size_t>(N), INT_MAX);   // roadblock cell -> the step it appears at
    for (const Roadblock& rb : roadblocks) appears[rb.cell] = std::min(appears[rb.cell], rb.appears);
    Belief b(m, prior);
    const auto offsets = disk(std::max(sensor_range, 2.0));   // the cells next to the rover are always sensed
    SensorRun run;
    run.first_seen.assign(static_cast<std::size_t>(N), 0);

    auto truth = [&](int x, int step) { return m.blocked(x) || appears[x] <= step; };
    auto sense = [&](int at, int step) {
        const int r = at / C, c = at % C;
        std::vector<int> changed;
        for (const auto& [dr, dc] : offsets) {
            const int nr = r + dr, nc = c + dc;
            if (nr < 0 || nc < 0 || nr >= R || nc >= C) continue;
            const int x = nr * C + nc;
            const bool blk = truth(x, step);
            if (!b.known[x] || static_cast<bool>(b.blk[x]) != blk) {
                b.known[x] = 1;
                b.blk[x] = blk;
                changed.push_back(x);
            }
            if (!run.first_seen[x]) run.first_seen[x] = step + 1;
        }
        std::sort(changed.begin(), changed.end());
        return changed;
    };
    auto edge_costs = [&](const std::vector<int>& path) {
        std::vector<double> out;
        for (std::size_t k = 1; k < path.size(); ++k)
            out.push_back(b.cost(path[k - 1], path[k], path[k] / C - path[k - 1] / C, path[k] % C - path[k - 1] % C));
        return out;
    };
    auto add_up = [](const std::vector<double>& xs) {
        double total = 0.0;
        for (double x : xs) total += x;
        return total;
    };
    auto astar_from_scratch = [&](int at) {
        const SearchResult res = best_first(BeliefGraph{b}, N, at, gi, "A*", [](double g, double h) { return g + h; });
        return std::make_pair(res.nodes_expanded, res.found() ? res.cost : kInf);
    };

    const SearchResult optimal = astar(m, si, gi);
    run.optimal = optimal.found() ? optimal.cost : kInf;

    std::vector<int> changed = sense(si, 0);
    DStarLite ds(b, si, gi);
    long work = ds.compute();
    std::vector<int> plan = ds.plan();
    std::vector<double> plan_costs = edge_costs(plan);
    const auto [n0, c0] = astar_from_scratch(si);
    run.astar_work += n0;
    if (check && !plan.empty()) run.max_check_error = std::max(run.max_check_error, std::fabs(add_up(plan_costs) - c0));
    run.steps.push_back({si, static_cast<int>(changed.size()), work, false, n0, true, plan});
    run.driven.push_back(si);
    int cur = si;
    const long limit = max_steps > 0 ? max_steps : 4L * R * C;
    bool found = !plan.empty();
    while (found && cur != gi && static_cast<long>(run.driven.size()) <= limit) {
        cur = plan[1];
        run.driven.push_back(cur);
        ds.start = cur;
        const int step = static_cast<int>(run.driven.size()) - 1;
        changed = sense(cur, step);
        std::vector<int> rest(plan.begin() + 1, plan.end());
        bool broke = false;
        if (!changed.empty()) {                         // did the news make a move on the remaining plan dearer?
            const std::vector<double> now = edge_costs(rest);
            broke = !std::equal(now.begin(), now.end(), plan_costs.begin() + 1, plan_costs.end());
        }
        work = 0;
        if (!changed.empty()) {
            ds.moved_and_sensed(changed);
            work = ds.compute();
        }
        SensorStep rec{cur, static_cast<int>(changed.size()), work, broke, 0, false, {}};
        if (broke) {
            ++run.replans;
            const auto [n, cost] = astar_from_scratch(cur);
            run.astar_work += n;
            rec.astar = n;
            plan = ds.plan();
            if (check && !plan.empty())
                run.max_check_error = std::max(run.max_check_error, std::fabs(add_up(edge_costs(plan)) - cost));
            rec.has_plan = true;
            rec.plan = plan;
        } else if (!changed.empty()) {
            std::vector<int> fresh = ds.plan();
            if (fresh != rest) {                        // an equally quick alternative (rare): follow D* Lite's choice
                rec.has_plan = true;
                rec.plan = fresh;
            }
            plan = std::move(fresh);
        } else {
            plan = std::move(rest);
        }
        if (plan.empty()) {
            found = false;
            run.steps.push_back(std::move(rec));
            break;
        }
        plan_costs = edge_costs(plan);
        run.steps.push_back(std::move(rec));
    }
    run.found = found && cur == gi;
    run.time = run.found ? m.path_cost(run.driven) : kInf;
    run.dstar_work = ds.expanded;
    run.sensed = static_cast<int>(std::count_if(run.first_seen.begin(), run.first_seen.end(), [](int v) { return v != 0; }));
    run.runtime_ms = detail::ms_since(t0);
    return run;
}

}  // namespace robotnav
