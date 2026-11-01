#include "robotnav/airspace.hpp"

#include <algorithm>
#include <chrono>
#include <queue>
#include <stdexcept>

namespace robotnav {

const std::vector<Move>& air_moves() {
    static const std::vector<Move> moves = [] {
        std::vector<Move> out;
        for (int k = 0; k < 8; ++k) out.push_back({NB_R[k], NB_C[k], 0});
        out.push_back({0, 0, 1});
        out.push_back({0, 0, -1});
        for (int k = 0; k < 8; ++k) out.push_back({NB_R[k], NB_C[k], 1});
        for (int k = 0; k < 8; ++k) out.push_back({NB_R[k], NB_C[k], -1});
        return out;
    }();
    return moves;
}

namespace {
// The ground or the water surface, whichever is higher (no trees or rocks).
std::vector<double> ground_or_water(const Map& m) {
    const std::vector<double> wl = m.water_levels();
    std::vector<double> base(static_cast<std::size_t>(m.size()));
    for (int i = 0; i < m.size(); ++i) base[i] = std::isnan(wl[i]) ? m.height[i] : std::max(m.height[i], wl[i]);
    return base;
}
}  // namespace

std::vector<double> ground_top(const Map& m) {
    std::vector<double> top = ground_or_water(m);
    for (int i = 0; i < m.size(); ++i)
        top[i] = top[i] + (m.obstacle[i] == TREE ? AIR.tree : m.obstacle[i] == ROCK ? AIR.rock : 0.0);
    return top;
}

std::vector<std::pair<int, int>> place_towers(const Map& m, int n, double spacing) {
    const int R = m.rows, C = m.cols, edge = std::max(4, R / 12);
    std::vector<int> cand;                                   // open cells away from the edges, row by row
    for (int r = edge; r < R - edge; ++r)
        for (int c = edge; c < C - edge; ++c)
            if (m.obstacle[r * C + c] == OPEN) cand.push_back(r * C + c);
    std::stable_sort(cand.begin(), cand.end(), [&](int a, int b) {   // highest first, then row, then column
        if (m.height[a] != m.height[b]) return m.height[a] > m.height[b];
        return a < b;
    });
    std::vector<std::pair<int, int>> picked;
    for (int i : cand) {
        const int r = i / C, c = i % C;
        bool apart = true;
        for (const auto& [a, b] : picked)
            if (!((r - a) * (r - a) + (c - b) * (c - b) >= spacing * spacing)) apart = false;   // exact in any language
        if (apart) {
            picked.emplace_back(r, c);
            if (static_cast<int>(picked.size()) == n) break;
        }
    }
    return picked;
}

int Airspace::voxel(int cell) const {
    const int r = cell / C, c = cell % C;
    for (int k = 0; k < L; ++k)
        if (free[(k * R + r) * C + c]) return (k * R + r) * C + c;
    throw std::invalid_argument("no free airspace over cell (" + std::to_string(r) + ", " + std::to_string(c) + ")");
}

Airspace make_airspace(const Map& m) { return make_airspace(m, place_towers(m)); }

Airspace make_airspace(const Map& m, const std::vector<std::pair<int, int>>& towers) {
    Airspace air;
    air.L = AIR.levels;
    air.R = m.rows;
    air.C = m.cols;
    air.towers = towers;
    const int L = air.L, R = air.R, C = air.C, N = m.size();
    air.top = ground_top(m);
    const std::vector<double> base = ground_or_water(m);
    const double lo = *std::min_element(base.begin(), base.end());
    for (int k = 0; k < L; ++k) air.alt.push_back(lo + (k + 1) * AIR.dz);
    air.free.assign(static_cast<std::size_t>(L) * N, 0);
    air.seen.assign(static_cast<std::size_t>(L) * N, 0);
    for (int k = 0; k < L; ++k)
        for (int i = 0; i < N; ++i) air.free[static_cast<std::size_t>(k) * N + i] = air.alt[k] >= air.top[i] + AIR.clear;
    const double sight2 = AIR.sight * AIR.sight;
    for (const auto& [tr, tc] : towers) {
        const double E = m.height[tr * C + tc] + AIR.eye;
        air.eyes.push_back(E);
        for (int k = 0; k < L; ++k) {
            const double dy = air.alt[k] - E;
            for (int r = 0; r < R; ++r)
                for (int c = 0; c < C; ++c) {
                    const int dr = r - tr, dc = c - tc;
                    const std::size_t v = (static_cast<std::size_t>(k) * R + r) * C + c;
                    if (!((dr * dr + dc * dc) + dy * dy <= sight2) || !air.free[v] || (dr == 0 && dc == 0)) continue;
                    const int n = std::max(std::abs(dr), std::abs(dc));          // one sample per cell crossed
                    bool hidden = false;
                    for (int j = 1; j < n && !hidden; ++j) {
                        const double f = static_cast<double>(j) / n;
                        const double pr = tr + f * (r - tr);
                        const double pc = tc + f * (c - tc);
                        const double line = E + f * (air.alt[k] - E);
                        const int sr = static_cast<int>(std::floor(pr + 0.5)), sc = static_cast<int>(std::floor(pc + 0.5));
                        hidden = air.top[sr * C + sc] > line;                    // the ground or a tree is in the way
                    }
                    if (!hidden) air.seen[v] = 1;
                }
        }
    }
    return air;
}

bool move_ok(const Airspace& air, int r, int c, int k, int dr, int dc, int dk) {
    // Legal if every voxel in the move's little box (start, end, and the ones it passes beside) is free.
    if (!(0 <= r + dr && r + dr < air.R && 0 <= c + dc && c + dc < air.C && 0 <= k + dk && k + dk < air.L)) return false;
    for (int kk = k; ; kk += dk) {
        for (int rr = r; ; rr += dr) {
            for (int cc = c; ; cc += dc) {
                if (!air.free[(static_cast<std::size_t>(kk) * air.R + rr) * air.C + cc]) return false;
                if (!dc || cc == c + dc) break;
            }
            if (!dr || rr == r + dr) break;
        }
        if (!dk || kk == k + dk) break;
    }
    return true;
}

double move_time(int dr, int dc, int dk) {
    // Seconds to fly one move: its length at cruise speed, plus a little per metre climbed.
    const double h = (dr && dc) ? kSqrt2 : ((dr || dc) ? 1.0 : 0.0);
    const double dy = dk * AIR.dz;
    return std::sqrt(h * h + dy * dy) / AIR.speed + AIR.climb * std::max(dy, 0.0);
}

namespace {
struct Entry {
    double f;
    long tie;
    int v;
};
struct Later {
    bool operator()(const Entry& a, const Entry& b) const { return a.f > b.f || (a.f == b.f && a.tie > b.tie); }
};
}  // namespace

Flight fly(const Airspace& air, int start, int goal, bool stealth, bool heuristic) {
    const auto t0 = std::chrono::steady_clock::now();
    const int R = air.R, C = air.C, RC = R * C;
    const std::size_t V = static_cast<std::size_t>(air.L) * RC;
    const int s = air.voxel(start), g = air.voxel(goal);
    const int gk = g / RC, gr = (g % RC) / C, gc = g % C;
    const double w = stealth ? AIR.exposure : 0.0;
    const auto& moves = air_moves();
    std::vector<double> MT;
    for (const Move& mv : moves) MT.push_back(move_time(mv.dr, mv.dc, mv.dk));
    auto h = [&](int v) {
        if (!heuristic) return 0.0;
        const int k = v / RC, r = (v % RC) / C, c = v % C;
        const double y = (k - gk) * AIR.dz;
        return std::sqrt((r - gr) * (r - gr) + (c - gc) * (c - gc) + y * y) / AIR.speed;
    };
    std::vector<double> gs(V, kInf);
    std::vector<int> parent(V, -1);
    std::vector<char> closed(V, 0);
    std::priority_queue<Entry, std::vector<Entry>, Later> heap;
    long tie = 0;
    gs[s] = 0.0;
    heap.push({h(s), tie++, s});
    Flight out;
    out.name = stealth ? "Stealth" : "Direct";
    while (!heap.empty()) {
        const int v = heap.top().v;
        heap.pop();
        if (closed[v]) continue;
        closed[v] = 1;
        ++out.nodes;
        if (v == g) break;
        const int k = v / RC, r = (v % RC) / C, c = v % C;
        for (std::size_t mi = 0; mi < moves.size(); ++mi) {
            const Move& mv = moves[mi];
            if (!move_ok(air, r, c, k, mv.dr, mv.dc, mv.dk)) continue;
            const int u = v + (mv.dk * R + mv.dr) * C + mv.dc;
            const double step = MT[mi] + (air.seen[u] ? w * MT[mi] : 0.0);
            const double ng = gs[v] + step;
            if (ng < gs[u]) {
                gs[u] = ng;
                parent[u] = v;
                heap.push({ng + h(u), tie++, u});
            }
        }
    }
    out.found = closed[g];
    if (out.found) {
        for (int v = g; v != -1; v = parent[v]) out.voxels.push_back(v);
        std::reverse(out.voxels.begin(), out.voxels.end());
        out.cost = gs[g];
    }
    out.runtime_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (out.found) {
        const FlightStats st = flight_stats(air, out.voxels);
        out.time = st.time;
        out.seen = st.seen;
        out.distance = st.distance;
        out.top = st.top;
    }
    return out;
}

FlightStats flight_stats(const Airspace& air, const std::vector<int>& voxels) {
    const int C = air.C, RC = air.R * air.C;
    FlightStats st;
    for (std::size_t i = 1; i < voxels.size(); ++i) {
        const int a = voxels[i - 1], b = voxels[i];
        const int ka = a / RC, kb = b / RC, r0 = (a % RC) / C, c0 = a % C, r1 = (b % RC) / C, c1 = b % C;
        const double mt = move_time(r1 - r0, c1 - c0, kb - ka);
        st.time += mt;
        if (air.seen[b]) st.seen += mt;
        const double hgt = (r1 != r0 && c1 != c0) ? kSqrt2 : ((r1 != r0 || c1 != c0) ? 1.0 : 0.0);
        const double dy = (kb - ka) * AIR.dz;
        st.distance += std::sqrt(hgt * hgt + dy * dy);
        st.top = std::max(st.top, air.alt[kb] - air.top[r1 * C + c1]);
    }
    return st;
}

}  // namespace robotnav
