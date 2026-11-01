// robotnav: the C++ command-line tool. It reads a map file written by `python main.py export` and prints the same
// tables as the Python commands (route, paths, explore, fly), number for number; only the "ms" timings differ.
//
//   robotnav route   maps/desert.map --preset 19
//   robotnav paths   maps/island.map
//   robotnav explore maps/desert.map --preset 19 --sensor 5 --block 40,50
//   robotnav fly     maps/grassland.map --preset 15
//   robotnav bench   maps/grassland.map maps/desert.map maps/island.map
#include <algorithm>
#include <cmath>
#include <cctype>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <map>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "robotnav/airspace.hpp"
#include "robotnav/mapio.hpp"
#include "robotnav/planners.hpp"
#include "robotnav/replan.hpp"

using namespace robotnav;

namespace {

// ------------------------------------------------------------------------------------------------ formatting

struct Exit : std::runtime_error {                    // like Python's SystemExit("message"): print it, exit 1
    using std::runtime_error::runtime_error;
};

std::string format(const char* f, ...) {
    va_list ap, ap2;
    va_start(ap, f);
    va_copy(ap2, ap);
    const int n = std::vsnprintf(nullptr, 0, f, ap);
    va_end(ap);
    std::string s(static_cast<std::size_t>(n), '\0');
    std::vsnprintf(s.data(), s.size() + 1, f, ap2);
    va_end(ap2);
    return s;
}

std::size_t width(const std::string& s) {             // characters, not bytes ("→" is one)
    return static_cast<std::size_t>(std::count_if(s.begin(), s.end(), [](char ch) { return (ch & 0xC0) != 0x80; }));
}
std::string ljust(const std::string& s, std::size_t w) { return s + std::string(w > width(s) ? w - width(s) : 0, ' '); }
std::string rjust(const std::string& s, std::size_t w) { return std::string(w > width(s) ? w - width(s) : 0, ' ') + s; }

std::string clock(double sec) {                       // Python: "{}:{:02d}".format(*divmod(round(sec), 60))
    const long s = static_cast<long>(std::nearbyint(sec));   // rounds half to even, like round()
    return format("%ld:%02ld", s / 60, s % 60);
}

std::string grouped(long n) {                         // Python's f"{n:,}"
    std::string digits = std::to_string(n < 0 ? -n : n), out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i && (digits.size() - i) % 3 == 0) out += ',';
        out += digits[i];
    }
    return n < 0 ? "-" + out : out;
}

std::string title(std::string s) {                    // Python's str.title() for one word
    bool start = true;
    for (char& ch : s) {
        ch = start ? static_cast<char>(std::toupper(static_cast<unsigned char>(ch)))
                   : static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        start = !std::isalpha(static_cast<unsigned char>(ch));
    }
    return s;
}

std::string cell_text(const Map& m, int i) { return format("(%d, %d)", i / m.cols, i % m.cols); }
std::string cell_text(int r, int c) { return format("(%d, %d)", r, c); }

std::string header(const Map& m) { return title(m.theme) + " (seed " + std::to_string(m.seed) + ")"; }

// ------------------------------------------------------------------------------------------------ arguments

struct Args {
    std::string command;
    std::vector<std::string> files;
    std::optional<int> preset;
    std::optional<std::string> start, goal;
    double sensor = 5.0;
    std::string prior = "none";
    std::vector<std::string> blocks;
    int repeat = 3;
};

const char* kUsage =
    "usage: robotnav <command> <map file> [options]\n"
    "\n"
    "Robot obstacle navigation, the C++ core. Map files come from `python main.py export`.\n"
    "\n"
    "commands:\n"
    "  route     the five planners and the hill-blind shortest route between two cells\n"
    "  paths     the map's 20 preset trips: quickest over the hills vs the hill-blind shortest route\n"
    "  explore   drive seeing only a few metres, re-planning with D* Lite\n"
    "  fly       the air route: a drone flies out of the watchtowers' sight\n"
    "  bench     time every planner on every preset trip (one or more map files)\n"
    "\n"
    "options:\n"
    "  --preset N     plan one of the map's preset trips (1-20)\n"
    "  --start R,C    start cell (default: the map's start flag)\n"
    "  --goal R,C     destination cell (default: the map's destination flag)\n"
    "  --sensor M     explore: how far the rover sees, in metres (default 5)\n"
    "  --prior P      explore: what it knows beforehand: none, elevation or map (default none)\n"
    "  --block R,C    explore: a roadblock that is not on the map (repeatable)\n"
    "  --repeat N     bench: runs per measurement, the fastest is kept (default 3)\n";

Args parse(int argc, char** argv) {
    Args a;
    std::vector<std::string> v(argv + 1, argv + argc);
    if (v.empty() || v[0] == "-h" || v[0] == "--help") {
        std::cout << kUsage;
        std::exit(v.empty() ? 2 : 0);
    }
    a.command = v[0];
    for (std::size_t i = 1; i < v.size(); ++i) {
        std::string opt = v[i], val;
        if (opt.rfind("--", 0) != 0) {
            a.files.push_back(opt);
            continue;
        }
        if (opt == "--help" || opt == "-h") {
            std::cout << kUsage;
            std::exit(0);
        }
        const auto eq = opt.find('=');
        if (eq != std::string::npos) {
            val = opt.substr(eq + 1);
            opt = opt.substr(0, eq);
        } else if (i + 1 < v.size()) {
            val = v[++i];
        } else {
            throw Exit("robotnav: " + opt + " needs a value");
        }
        try {
            if (opt == "--preset") a.preset = std::stoi(val);
            else if (opt == "--start") a.start = val;
            else if (opt == "--goal") a.goal = val;
            else if (opt == "--sensor") a.sensor = std::stod(val);
            else if (opt == "--prior") a.prior = val;
            else if (opt == "--block") a.blocks.push_back(val);
            else if (opt == "--repeat") a.repeat = std::max(1, std::stoi(val));
            else throw Exit("robotnav: unknown option " + opt + " (see robotnav --help)");
        } catch (const std::logic_error&) {
            throw Exit("robotnav: " + opt + " takes a number, not '" + val + "'");
        }
    }
    if (a.files.empty()) throw Exit("robotnav: give a map file (python main.py export writes them to maps/)");
    return a;
}

// '80,10', '80, 10', '80 10' or '(80, 10)' -> (80, 10): the same forms as the Python CLI and the viewer take.
std::pair<int, int> parse_cell(const std::string& text) {
    static const std::regex sep("[,;\\s]+"), number("-?\\d+");
    auto strip = [](std::string s, const char* chars) {
        const auto a = s.find_first_not_of(chars), b = s.find_last_not_of(chars);
        return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
    };
    const std::string inner = strip(strip(strip(text, " \t\n\r\f\v"), "()[]"), " \t\n\r\f\v");
    std::vector<std::string> parts;
    for (std::sregex_token_iterator it(inner.begin(), inner.end(), sep, -1), end; it != end; ++it)
        if (!it->str().empty()) parts.push_back(*it);
    if (parts.size() != 2 || !std::regex_match(parts[0], number) || !std::regex_match(parts[1], number))
        throw Exit("Give a cell as row,col (for example 80,10), not '" + text + "'.");
    return {std::stoi(parts[0]), std::stoi(parts[1])};
}

// Start and destination from --preset, or --start/--goal (default: the map's flags), checked like Python's _trip.
std::pair<int, int> trip(const Args& a, const Map& m) {
    if (a.preset && (a.start || a.goal)) throw Exit("Use --preset, or --start and --goal, not both.");
    std::pair<int, int> s{m.start / m.cols, m.start % m.cols}, g{m.goal / m.cols, m.goal % m.cols};
    if (a.start) s = parse_cell(*a.start);
    if (a.goal) g = parse_cell(*a.goal);
    if (a.preset) {
        const int n = static_cast<int>(m.presets.size());
        if (*a.preset < 1 || *a.preset > n) throw Exit("Pick a preset from 1 to " + std::to_string(n) + ".");
        const Preset& p = m.presets[*a.preset - 1];
        s = {p.start / m.cols, p.start % m.cols};
        g = {p.goal / m.cols, p.goal % m.cols};
    }
    for (const auto& [name, rc] : {std::make_pair("start", s), std::make_pair("destination", g)}) {
        const auto [r, c] = rc;
        if (!m.inside(r, c))
            throw Exit(format("The %s %s is outside the %dx%d map.", name, cell_text(r, c).c_str(), m.rows, m.cols));
        if (m.blocked(m.at(r, c)))
            throw Exit(format("The %s %s is on an obstacle or water. Pick an open cell.", name, cell_text(r, c).c_str()));
    }
    if (s == g) throw Exit("The start and destination are the same cell. Pick two different cells.");
    return {m.at(s.first, s.second), m.at(g.first, g.second)};
}

// ------------------------------------------------------------------------------------------------ commands

void print_results(const Map& m, const std::vector<SearchResult>& results) {
    struct Row {
        std::string name;
        double cost;
        PathStats st;
        long nodes;
        double ms;
    };
    std::vector<Row> rows;
    for (const SearchResult& r : results)
        if (r.found()) rows.push_back({r.algorithm.substr(0, r.algorithm.find(" (")), r.cost, path_stats(m, r.path),
                                       r.nodes_expanded, r.runtime_ms});
    double best = kInf;
    for (const Row& r : rows) best = std::min(best, r.cost);
    std::stable_sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) {   // quickest first, then fewest searched
        const double ra = py_round(a.cost, 6), rb = py_round(b.cost, 6);
        return ra != rb ? ra < rb : a.nodes < b.nodes;
    });
    std::printf("  %-16s%8s%9s%8s%9s%9s%10s%8s\n", "planner", "time", "vs best", "dist m", "climb m", "rough m",
                "searched", "ms");
    for (const Row& r : rows) {
        const long s = static_cast<long>(std::nearbyint(r.cost));
        std::printf("  %-16s%5ld:%02ld%+8.1f%%%8.0f%9.1f%9.0f%10ld%8s\n", r.name.c_str(), s / 60, s % 60,
                    (r.cost / best - 1.0) * 100.0, r.st.distance_m, r.st.climb_m, r.st.rough_m, r.nodes,
                    format("%.1f", r.ms).c_str());
    }
}

int route(const Args& a, const Map& m) {
    const auto [s, g] = trip(a, m);
    std::vector<SearchResult> results = plan_pair(m, s, g);
    results.push_back(shortest_route(m, s, g));
    std::printf("%s: start %s -> destination %s\n", header(m).c_str(), cell_text(m, s).c_str(), cell_text(m, g).c_str());
    if (std::none_of(results.begin(), results.end(), [](const SearchResult& r) { return r.found(); }))
        throw Exit("No drivable route between those cells: one is walled in by obstacles, water or steep slopes.");
    print_results(m, results);
    return 0;
}

int paths(const Args&, const Map& m) {
    std::printf("\n%s: quickest route over the hills vs the hill-blind shortest route\n", header(m).c_str());
    std::printf("  %s%s%10s%13s%10s%10s%8s%8s\n", ljust("path", 5).c_str(), ljust("from → to", 17).c_str(), "start",
                "destination", "quickest", "shortest", "longer", "dist m");
    int k = 0;
    for (const Preset& p : m.presets) {
        const SearchResult best = astar(m, p.start, p.goal), blind = shortest_route(m, p.start, p.goal);
        std::printf("  P%-4d%s%s%s%10s%10s%+7.0f%%%8.0f\n", ++k, ljust(p.name, 17).c_str(),
                    rjust(cell_text(m, p.start), 10).c_str(), rjust(cell_text(m, p.goal), 13).c_str(),
                    clock(best.cost).c_str(), clock(blind.cost).c_str(), (blind.cost / best.cost - 1.0) * 100.0,
                    path_stats(m, best.path).distance_m);
    }
    return 0;
}

const std::map<std::string, std::string> kPriorWords = {
    {"none", "knowing nothing beforehand"},
    {"elevation", "knowing the hills but not the obstacles"},
    {"map", "knowing the map but not the roadblocks"}};

int explore(const Args& a, const Map& m) {
    const auto [s, g] = trip(a, m);
    if (!kPriorWords.count(a.prior)) throw Exit("robotnav: --prior must be none, elevation or map");
    std::vector<Roadblock> blocks;
    for (const std::string& text : a.blocks) {
        const auto [r, c] = parse_cell(text);
        if (!m.inside(r, c))
            throw Exit(format("The roadblock %s is outside the %dx%d map.", cell_text(r, c).c_str(), m.rows, m.cols));
        const int x = m.at(r, c);
        if (m.blocked(x) || x == s || x == g)
            throw Exit("A roadblock at " + cell_text(r, c) + " would sit on an obstacle, water, the start or the destination.");
        blocks.push_back({x, 0});
    }
    const SensorRun run = sensor_drive(m, s, g, a.sensor, parse_prior(a.prior), blocks, true);
    std::printf("%s: start %s -> destination %s, sensor %g m, %s%s\n", header(m).c_str(), cell_text(m, s).c_str(),
                cell_text(m, g).c_str(), a.sensor, kPriorWords.at(a.prior).c_str(),
                blocks.empty() ? "" : format(", %zu roadblock(s)", blocks.size()).c_str());
    if (!run.found) throw Exit("The rover could not reach the destination: the roadblocks wall it off.");
    const double more = static_cast<double>(run.astar_work) / static_cast<double>(std::max(1L, run.dstar_work));
    std::printf("  drive time       %7s   %+.1f%% on A* with the whole map known (%s)\n", clock(run.time).c_str(),
                run.extra() * 100.0, clock(run.optimal).c_str());
    std::printf("  distance         %5.0f m   in %zu moves\n", path_stats(m, run.driven).distance_m, run.driven.size() - 1);
    std::printf("  re-plans         %7d   times newly sensed ground broke the plan\n", run.replans);
    std::printf("  D* Lite work     %7s   cells expanded over the whole drive\n", grouped(run.dstar_work).c_str());
    std::printf("  A* from scratch  %7s   cells, planning again each time the plan broke (%.1fx %s)\n",
                grouped(run.astar_work).c_str(), more, more >= 1 ? "more" : "less");
    std::printf("  sensed           %7s   cells (%.0f%% of the map)\n", grouped(run.sensed).c_str(),
                static_cast<double>(run.sensed) / (m.rows * m.cols) * 100.0);
    std::printf("  every repaired plan was exactly as quick as A* from scratch (largest difference %.1e s)\n",
                run.max_check_error);
    return 0;
}

int fly_cmd(const Args& a, const Map& m) {
    const auto [s, g] = trip(a, m);
    const Airspace air = make_airspace(m);
    const Flight st = fly(air, s, g, true), di = fly(air, s, g, false);
    std::string towers;
    for (const auto& [r, c] : air.towers) towers += (towers.empty() ? "" : ", ") + cell_text(r, c);
    std::printf("%s: start %s -> destination %s; %zu watchtowers see %g m (towers at %s)\n", header(m).c_str(),
                cell_text(m, s).c_str(), cell_text(m, g).c_str(), air.towers.size(), AIR.sight, towers.c_str());
    std::printf("  %-10s%7s%10s%8s%7s%10s%8s\n", "route", "time", "in sight", "dist m", "top m", "searched", "ms");
    for (const Flight* f : {&st, &di})
        std::printf("  %-10s%7s%9.1fs%8.0f%7.1f%10s%8.1f\n", f->name.c_str(), clock(f->time).c_str(), f->seen,
                    f->distance, f->top, grouped(f->nodes).c_str(), f->runtime_ms);
    std::printf("Staying hidden costs %+.0f%% flight time and cuts the time in sight from %.1f s to %.1f s.\n",
                (st.time / di.time - 1.0) * 100.0, di.seen, st.seen);
    return 0;
}

template <class F>
double fastest_ms(int repeat, F&& f) {                // best of `repeat` runs, in milliseconds
    double best = kInf;
    for (int k = 0; k < repeat; ++k) {
        const auto t0 = std::chrono::steady_clock::now();
        f();
        best = std::min(best, detail::ms_since(t0));
    }
    return best;
}

int bench(const Args& a, const std::vector<Map>& maps) {
    std::printf("Every preset trip, fastest of %d run(s); milliseconds per trip (mean)\n", a.repeat);
    std::printf("  %-12s%8s%10s%8s%13s%8s%10s%10s%10s%10s%10s\n", "map", "BFS", "Dijkstra", "A*", "Weighted A*",
                "Greedy", "Shortest", "D* 5 m", "D* 20 m", "Stealth", "Direct");
    for (const Map& m : maps) {
        const double n = static_cast<double>(std::max<std::size_t>(1, m.presets.size()));
        std::vector<double> ms(11, 0.0);
        const Airspace air = make_airspace(m);
        for (const Preset& p : m.presets) {
            const int s = p.start, g = p.goal;
            ms[0] += fastest_ms(a.repeat, [&] { bfs(m, s, g); });
            ms[1] += fastest_ms(a.repeat, [&] { dijkstra(m, s, g); });
            ms[2] += fastest_ms(a.repeat, [&] { astar(m, s, g); });
            ms[3] += fastest_ms(a.repeat, [&] { weighted_astar(m, s, g); });
            ms[4] += fastest_ms(a.repeat, [&] { greedy(m, s, g); });
            ms[5] += fastest_ms(a.repeat, [&] { shortest_route(m, s, g); });
            ms[6] += fastest_ms(a.repeat, [&] { sensor_drive(m, s, g, 5.0); });
            ms[7] += fastest_ms(a.repeat, [&] { sensor_drive(m, s, g, 20.0); });
            ms[8] += fastest_ms(a.repeat, [&] { fly(air, s, g, true); });
            ms[9] += fastest_ms(a.repeat, [&] { fly(air, s, g, false); });
        }
        ms[10] = fastest_ms(a.repeat, [&] { make_airspace(m); });
        std::printf("  %-12s%8.2f%10.2f%8.2f%13.2f%8.2f%10.2f%10.1f%10.1f%10.1f%10.1f\n", title(m.theme).c_str(),
                    ms[0] / n, ms[1] / n, ms[2] / n, ms[3] / n, ms[4] / n, ms[5] / n, ms[6] / n, ms[7] / n, ms[8] / n,
                    ms[9] / n);
        std::printf("  %-12s building the airspace (%d levels, %zu towers' sight lines): %.1f ms\n", "", air.L,
                    air.towers.size(), ms[10]);
    }
    std::printf("(D* 5 m and D* 20 m are whole drives with a 5 m and a 20 m sensor, A* baseline included)\n");
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args a = parse(argc, argv);
        std::vector<Map> maps;
        for (const std::string& f : a.files) maps.push_back(load_map(f));
        if (a.command == "bench") return bench(a, maps);
        if (maps.size() != 1 && a.command != "paths") throw Exit("robotnav " + a.command + " takes one map file");
        if (a.command == "route") return route(a, maps[0]);
        if (a.command == "explore") return explore(a, maps[0]);
        if (a.command == "fly") return fly_cmd(a, maps[0]);
        if (a.command == "paths") {
            for (const Map& m : maps) paths(a, m);
            return 0;
        }
        throw Exit("robotnav: unknown command '" + a.command + "' (route, paths, explore, fly or bench)");
    } catch (const Exit& e) {
        std::fflush(stdout);
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    } catch (const std::exception& e) {
        std::fflush(stdout);
        std::fprintf(stderr, "robotnav: %s\n", e.what());
        return 1;
    }
}
