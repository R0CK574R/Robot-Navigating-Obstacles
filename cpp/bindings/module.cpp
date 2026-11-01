// robotnav_cpp: the C++ core as a Python module (pybind11). robotnav/cpp.py wraps it so a Terrain goes in and the
// Python reference's own result types come out, which is how the tests compare the two number for number.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "robotnav/airspace.hpp"
#include "robotnav/mapio.hpp"
#include "robotnav/planners.hpp"
#include "robotnav/replan.hpp"

namespace py = pybind11;
using namespace robotnav;

namespace {

using Grid2D = py::array_t<double, py::array::c_style | py::array::forcecast>;
using Ints2D = py::array_t<int, py::array::c_style | py::array::forcecast>;

template <class T>
std::vector<T> flat(const py::array_t<T, py::array::c_style | py::array::forcecast>& a, int rows, int cols,
                    const char* what) {
    if (a.ndim() != 2 || a.shape(0) != rows || a.shape(1) != cols)
        throw std::invalid_argument(std::string(what) + " must be a rows x cols array");
    return std::vector<T>(a.data(), a.data() + a.size());
}

template <class T>
py::array_t<T> grid(const std::vector<T>& v, std::vector<py::ssize_t> shape) {
    py::array_t<T> out(shape);
    std::copy(v.begin(), v.end(), out.mutable_data());
    return out;
}

py::array_t<bool> bool_grid(const std::vector<char>& v, std::vector<py::ssize_t> shape) {
    py::array_t<bool> out(shape);
    bool* d = out.mutable_data();
    for (std::size_t i = 0; i < v.size(); ++i) d[i] = v[i] != 0;
    return out;
}

}  // namespace

PYBIND11_MODULE(robotnav_cpp, mod) {
    mod.doc() = "Robot obstacle navigation: the route-deciding core in C++ (planners, D* Lite, the air route).";

    py::class_<Pond>(mod, "Pond")
        .def(py::init([](double level, std::vector<int> cells) { return Pond{level, std::move(cells)}; }),
             py::arg("level"), py::arg("cells"))
        .def_readwrite("level", &Pond::level)
        .def_readwrite("cells", &Pond::cells);

    py::class_<Preset>(mod, "Preset")
        .def(py::init([](int start, int goal, std::string name) { return Preset{start, goal, std::move(name)}; }),
             py::arg("start"), py::arg("goal"), py::arg("name") = "")
        .def_readwrite("start", &Preset::start)
        .def_readwrite("goal", &Preset::goal)
        .def_readwrite("name", &Preset::name);

    py::class_<Map>(mod, "Map", "A map and its drive-time model. Cells are flat indices, row * cols + col.")
        .def(py::init<>())
        .def("set_grids",
             [](Map& m, const Grid2D& height, const Ints2D& surface, const Ints2D& obstacle) {
                 if (height.ndim() != 2) throw std::invalid_argument("height must be a 2-D array");
                 m.rows = static_cast<int>(height.shape(0));
                 m.cols = static_cast<int>(height.shape(1));
                 m.height = flat<double>(height, m.rows, m.cols, "height");
                 m.surface = flat<int>(surface, m.rows, m.cols, "surface");
                 m.obstacle = flat<int>(obstacle, m.rows, m.cols, "obstacle");
             },
             py::arg("height"), py::arg("surface"), py::arg("obstacle"))
        .def_readwrite("theme", &Map::theme)
        .def_readwrite("seed", &Map::seed)
        .def_readonly("rows", &Map::rows)
        .def_readonly("cols", &Map::cols)
        .def_readwrite("cell", &Map::cell)
        .def_readwrite("max_grade", &Map::max_grade)
        .def_readwrite("uphill", &Map::uphill)
        .def_readwrite("short_tie", &Map::short_tie)
        .def_readwrite("surface_cost", &Map::surface_cost)
        .def_readwrite("surface_names", &Map::surface_names)
        .def_readwrite("sea", &Map::sea)
        .def_readwrite("ponds", &Map::ponds)
        .def_readwrite("start", &Map::start)
        .def_readwrite("goal", &Map::goal)
        .def_readwrite("presets", &Map::presets)
        .def_property_readonly("height", [](const Map& m) { return grid(m.height, {m.rows, m.cols}); })
        .def_property_readonly("surface", [](const Map& m) { return grid(m.surface, {m.rows, m.cols}); })
        .def_property_readonly("obstacle", [](const Map& m) { return grid(m.obstacle, {m.rows, m.cols}); })
        .def("edge_cost", [](const Map& m, int a, int b) -> py::object {
                 const double c = m.edge_cost(a, b);
                 return c < 0 ? py::object(py::none()) : py::object(py::float_(c));
             }, "Seconds to drive a -> b, or None if steeper than the grade limit.")
        .def("heuristic", &Map::heuristic)
        .def("path_cost", &Map::path_cost)
        .def("water_levels", [](const Map& m) { return grid(m.water_levels(), {m.rows, m.cols}); });

    py::class_<SearchResult>(mod, "SearchResult")
        .def_readonly("algorithm", &SearchResult::algorithm)
        .def_readonly("path", &SearchResult::path)
        .def_readonly("cost", &SearchResult::cost)
        .def_readonly("nodes_expanded", &SearchResult::nodes_expanded)
        .def_readonly("max_frontier", &SearchResult::max_frontier)
        .def_readonly("runtime_ms", &SearchResult::runtime_ms)
        .def_readonly("explored", &SearchResult::explored)
        .def_property_readonly("found", &SearchResult::found);

    const auto release = py::call_guard<py::gil_scoped_release>();
    mod.def("bfs", &bfs, py::arg("map"), py::arg("start"), py::arg("goal"), release);
    mod.def("dijkstra", &dijkstra, py::arg("map"), py::arg("start"), py::arg("goal"), release);
    mod.def("astar", &astar, py::arg("map"), py::arg("start"), py::arg("goal"), release);
    mod.def("weighted_astar", &weighted_astar, py::arg("map"), py::arg("start"), py::arg("goal"), py::arg("w") = 2.0,
            release);
    mod.def("greedy", &greedy, py::arg("map"), py::arg("start"), py::arg("goal"), release);
    mod.def("shortest_route", &shortest_route, py::arg("map"), py::arg("start"), py::arg("goal"), release);
    mod.def("plan_pair", &plan_pair, py::arg("map"), py::arg("start"), py::arg("goal"), release,
            "BFS, Dijkstra, A*, Weighted A* and Greedy from start to goal.");
    mod.def("path_stats", [](const Map& m, const std::vector<int>& path) {
        const PathStats s = path_stats(m, path);
        py::dict d;
        d["distance_m"] = s.distance_m;
        d["climb_m"] = s.climb_m;
        d["descent_m"] = s.descent_m;
        d["max_grade_pct"] = s.max_grade_pct;
        d["rough_m"] = s.rough_m;
        return d;
    }, py::arg("map"), py::arg("path"));

    py::class_<SensorStep>(mod, "SensorStep")
        .def_readonly("cell", &SensorStep::cell)
        .def_readonly("changed", &SensorStep::changed)
        .def_readonly("work", &SensorStep::work)
        .def_readonly("broke", &SensorStep::broke)
        .def_readonly("astar", &SensorStep::astar)
        .def_readonly("has_plan", &SensorStep::has_plan)
        .def_readonly("plan", &SensorStep::plan);

    py::class_<SensorRun>(mod, "SensorRun")
        .def_readonly("found", &SensorRun::found)
        .def_readonly("driven", &SensorRun::driven)
        .def_readonly("time", &SensorRun::time)
        .def_readonly("optimal", &SensorRun::optimal)
        .def_readonly("steps", &SensorRun::steps)
        .def_readonly("dstar_work", &SensorRun::dstar_work)
        .def_readonly("astar_work", &SensorRun::astar_work)
        .def_readonly("replans", &SensorRun::replans)
        .def_readonly("sensed", &SensorRun::sensed)
        .def_readonly("runtime_ms", &SensorRun::runtime_ms)
        .def_readonly("max_check_error", &SensorRun::max_check_error)
        .def_readonly("first_seen", &SensorRun::first_seen)
        .def_property_readonly("extra", &SensorRun::extra);

    mod.def("disk", &disk, py::arg("sensor_range"));
    mod.def("sensor_drive",
            [](const Map& m, int start, int goal, double sensor_range, const std::string& prior,
               const std::vector<std::pair<int, int>>& roadblocks, bool check, long max_steps) {
                std::vector<Roadblock> blocks;
                for (const auto& [cell, appears] : roadblocks) blocks.push_back({cell, appears});
                const Prior p = parse_prior(prior);
                py::gil_scoped_release unlocked;
                return sensor_drive(m, start, goal, sensor_range, p, blocks, check, max_steps);
            },
            py::arg("map"), py::arg("start"), py::arg("goal"), py::arg("sensor_range") = 10.0, py::arg("prior") = "none",
            py::arg("roadblocks") = std::vector<std::pair<int, int>>{}, py::arg("check") = false,
            py::arg("max_steps") = 0,
            "Drive seeing only sensor_range metres, re-planning with D* Lite. roadblocks: (cell, appears at step).");

    py::class_<Airspace>(mod, "Airspace")
        .def_readonly("towers", &Airspace::towers)
        .def_readonly("alt", &Airspace::alt)
        .def_readonly("eyes", &Airspace::eyes)
        .def_property_readonly("shape", [](const Airspace& a) { return py::make_tuple(a.L, a.R, a.C); })
        .def_property_readonly("free", [](const Airspace& a) { return bool_grid(a.free, {a.L, a.R, a.C}); })
        .def_property_readonly("seen", [](const Airspace& a) { return bool_grid(a.seen, {a.L, a.R, a.C}); })
        .def_property_readonly("top", [](const Airspace& a) { return grid(a.top, {a.R, a.C}); })
        .def("voxel", &Airspace::voxel, py::arg("cell"));

    py::class_<Flight>(mod, "Flight")
        .def_readonly("name", &Flight::name)
        .def_readonly("voxels", &Flight::voxels)
        .def_readonly("found", &Flight::found)
        .def_readonly("cost", &Flight::cost)
        .def_readonly("time", &Flight::time)
        .def_readonly("seen", &Flight::seen)
        .def_readonly("nodes", &Flight::nodes)
        .def_readonly("runtime_ms", &Flight::runtime_ms)
        .def_readonly("distance", &Flight::distance)
        .def_readonly("top", &Flight::top);

    mod.def("place_towers", &place_towers, py::arg("map"), py::arg("n") = AIR.towers, py::arg("spacing") = AIR.spacing);
    mod.def("make_airspace",
            [](const Map& m, std::optional<std::vector<std::pair<int, int>>> towers) {
                py::gil_scoped_release unlocked;
                return towers ? make_airspace(m, *towers) : make_airspace(m);
            },
            py::arg("map"), py::arg("towers") = py::none());
    mod.def("fly", &fly, py::arg("air"), py::arg("start"), py::arg("goal"), py::arg("stealth") = true,
            py::arg("heuristic") = true, release);
    mod.def("move_time", &move_time);
    mod.def("load_map", &load_map, py::arg("path"));
    mod.def("save_map", &save_map, py::arg("map"), py::arg("path"));

    py::dict air;
    air["levels"] = AIR.levels;
    air["dz"] = AIR.dz;
    air["clear"] = AIR.clear;
    air["tree"] = AIR.tree;
    air["rock"] = AIR.rock;
    air["towers"] = AIR.towers;
    air["spacing"] = AIR.spacing;
    air["eye"] = AIR.eye;
    air["sight"] = AIR.sight;
    air["speed"] = AIR.speed;
    air["climb"] = AIR.climb;
    air["exposure"] = AIR.exposure;
    mod.attr("AIR") = air;
}
