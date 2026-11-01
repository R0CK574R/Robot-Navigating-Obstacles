#include "robotnav/mapio.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace robotnav {

namespace {

std::vector<std::string> words(const std::string& s) {
    std::istringstream in(s);
    std::vector<std::string> out;
    for (std::string w; in >> w;) out.push_back(w);
    return out;
}

// The rest of a line after its first n words ("preset 1 2 3 4 NW → SE" after 5 words is "NW → SE").
std::string after_words(const std::string& line, int n) {
    std::size_t at = 0;
    for (int k = 0; k < n; ++k) {
        at = line.find_first_not_of(" \t", at);
        if (at == std::string::npos) return "";
        at = line.find_first_of(" \t", at);
        if (at == std::string::npos) return "";
    }
    at = line.find_first_not_of(" \t", at);
    return at == std::string::npos ? "" : line.substr(at);
}

double to_double(const std::string& w) {
    char* end = nullptr;
    errno = 0;
    const double v = std::strtod(w.c_str(), &end);          // correctly rounded: repr text reads back exactly
    if (end == w.c_str() || *end || errno == ERANGE) throw std::runtime_error("not a number: '" + w + "'");
    return v;
}

long to_long(const std::string& w) {
    char* end = nullptr;
    const long v = std::strtol(w.c_str(), &end, 10);
    if (end == w.c_str() || *end) throw std::runtime_error("not an integer: '" + w + "'");
    return v;
}

std::string fmt(double v) {                                  // 17 significant digits always read back exactly
    char buf[40];
    std::snprintf(buf, sizeof buf, "%.17g", v);
    return buf;
}

}  // namespace

Map load_map(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open map file " + path);
    std::string line;
    auto next = [&](const char* what) {
        if (!std::getline(in, line)) throw std::runtime_error(path + ": the file ends before " + what);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        return line;
    };
    if (words(next("its header")) != std::vector<std::string>{"robotnav-map", "1"})
        throw std::runtime_error(path + " is not a robotnav map file (version 1)");
    Map m;
    auto cell = [&](const std::string& r, const std::string& c) {
        const long rr = to_long(r), cc = to_long(c);
        if (rr < 0 || cc < 0 || rr >= m.rows || cc >= m.cols) throw std::runtime_error(path + ": a cell is off the map");
        return static_cast<int>(rr * m.cols + cc);
    };
    auto grid = [&](const char* what, auto read) {
        for (int r = 0; r < m.rows; ++r) {
            const auto w = words(next(what));
            if (static_cast<int>(w.size()) != m.cols) throw std::runtime_error(path + ": a " + what + " row has the wrong length");
            for (const auto& x : w) read(x);
        }
    };
    bool ended = false;
    while (!ended && std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const auto w = words(line);
        if (w.empty()) continue;
        const std::string& key = w[0];
        const std::string rest = after_words(line, 1);
        try {
            if (key == "theme") m.theme = rest;
            else if (key == "seed") m.seed = to_long(w.at(1));
            else if (key == "rows") m.rows = static_cast<int>(to_long(w.at(1)));
            else if (key == "cols") m.cols = static_cast<int>(to_long(w.at(1)));
            else if (key == "cell") m.cell = to_double(w.at(1));
            else if (key == "max_grade") m.max_grade = to_double(w.at(1));
            else if (key == "uphill") m.uphill = to_double(w.at(1));
            else if (key == "short_tie") m.short_tie = to_double(w.at(1));
            else if (key == "surface_costs") {
                for (std::size_t i = 1; i < w.size(); ++i) m.surface_cost.push_back(to_double(w[i]));
            } else if (key == "surface_names") {
                std::istringstream names(rest);
                for (std::string n; std::getline(names, n, '|');) m.surface_names.push_back(n);
            } else if (key == "sea") {
                if (w.at(1) != "none") m.sea = to_double(w[1]);
            } else if (key == "start") m.start = cell(w.at(1), w.at(2));
            else if (key == "goal") m.goal = cell(w.at(1), w.at(2));
            else if (key == "pond") {
                Pond p;
                p.level = to_double(w.at(1));
                const long n = to_long(w.at(2));
                if (static_cast<long>(w.size()) != 3 + 2 * n) throw std::runtime_error("a pond lists the wrong number of cells");
                for (long k = 0; k < n; ++k) p.cells.push_back(cell(w[3 + 2 * k], w[4 + 2 * k]));
                m.ponds.push_back(std::move(p));
            } else if (key == "preset") {
                Preset p;
                p.start = cell(w.at(1), w.at(2));
                p.goal = cell(w.at(3), w.at(4));
                p.name = after_words(line, 5);               // the name: everything after the four numbers
                m.presets.push_back(std::move(p));
            } else if (key == "ponds" || key == "presets") {
                // counts, for people reading the file; the pond and preset lines speak for themselves
            } else if (key == "height") {
                m.height.reserve(static_cast<std::size_t>(m.size()));
                grid("height", [&](const std::string& x) { m.height.push_back(to_double(x)); });
            } else if (key == "surface") {
                grid("surface", [&](const std::string& x) { m.surface.push_back(static_cast<int>(to_long(x))); });
            } else if (key == "obstacle") {
                grid("obstacle", [&](const std::string& x) { m.obstacle.push_back(static_cast<int>(to_long(x))); });
            } else if (key == "end") ended = true;
            else throw std::runtime_error("unknown entry '" + key + "'");
        } catch (const std::out_of_range&) {
            throw std::runtime_error(path + ": '" + key + "' is missing a value");
        } catch (const std::runtime_error& e) {
            const std::string msg = e.what();
            throw std::runtime_error(msg.rfind(path, 0) == 0 ? msg : path + ": " + msg);
        }
    }
    const auto n = static_cast<std::size_t>(m.size());
    if (!ended || m.rows <= 0 || m.cols <= 0 || m.height.size() != n || m.surface.size() != n || m.obstacle.size() != n)
        throw std::runtime_error(path + " is incomplete");
    for (int s : m.surface)
        if (s < 0 || s >= static_cast<int>(m.surface_cost.size())) throw std::runtime_error(path + ": a surface has no cost");
    return m;
}

void save_map(const Map& m, const std::string& path) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write " + path);
    const int C = m.cols;
    out << "robotnav-map 1\ntheme " << m.theme << "\nseed " << m.seed << "\nrows " << m.rows << "\ncols " << m.cols
        << "\ncell " << fmt(m.cell) << "\nmax_grade " << fmt(m.max_grade) << "\nuphill " << fmt(m.uphill)
        << "\nshort_tie " << fmt(m.short_tie) << "\nsurface_costs";
    for (double c : m.surface_cost) out << ' ' << fmt(c);
    out << "\nsurface_names ";
    for (std::size_t i = 0; i < m.surface_names.size(); ++i) out << (i ? "|" : "") << m.surface_names[i];
    out << "\nsea " << (m.sea ? fmt(*m.sea) : "none") << "\nstart " << m.start / C << ' ' << m.start % C << "\ngoal "
        << m.goal / C << ' ' << m.goal % C << "\nponds " << m.ponds.size() << '\n';
    for (const Pond& p : m.ponds) {
        out << "pond " << fmt(p.level) << ' ' << p.cells.size();
        for (int i : p.cells) out << ' ' << i / C << ' ' << i % C;
        out << '\n';
    }
    out << "presets " << m.presets.size() << '\n';
    for (const Preset& p : m.presets)
        out << "preset " << p.start / C << ' ' << p.start % C << ' ' << p.goal / C << ' ' << p.goal % C << ' ' << p.name << '\n';
    auto grid = [&](const char* what, auto get) {
        out << what << '\n';
        for (int r = 0; r < m.rows; ++r) {
            for (int c = 0; c < C; ++c) out << (c ? " " : "") << get(r * C + c);
            out << '\n';
        }
    };
    grid("height", [&](int i) { return fmt(m.height[i]); });
    grid("surface", [&](int i) { return std::to_string(m.surface[i]); });
    grid("obstacle", [&](int i) { return std::to_string(m.obstacle[i]); });
    out << "end\n";
    if (!out) throw std::runtime_error("could not finish writing " + path);
}

}  // namespace robotnav
