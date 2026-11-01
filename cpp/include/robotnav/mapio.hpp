// Map files: how the Python side hands a generated map (heights, surfaces, obstacles, water, the default flags and
// the 20 preset trips) to the C++ tools. `python main.py export` writes them.
//
// A plain text format, one item per line, numbers written so they read back exactly (Python's repr of a float is
// the shortest text that round-trips, and strtod reads it back to the very same double):
//
//   robotnav-map 1
//   theme desert
//   seed 3
//   rows 96
//   cols 96
//   cell 1.0
//   max_grade 0.6
//   uphill 3.0
//   short_tie 1e-06
//   surface_costs 1.0 2.4 1.3 1.0
//   surface_names Firm sand|Soft sand|Rocky ground|Oasis grass
//   sea none                                   (or the sea's surface, metres)
//   start 89 6
//   goal 6 89
//   ponds 3
//   pond <level> <n> r c r c ...               (n cells)
//   presets 20
//   preset <start r> <start c> <goal r> <goal c> <name>
//   height                                     (then rows lines of cols numbers)
//   surface                                    (rows lines of cols integers)
//   obstacle                                   (rows lines of cols integers: 0 open, 1 rock, 2 tree, 3 water)
//   end
#pragma once

#include <string>

#include "robotnav/map.hpp"

namespace robotnav {

Map load_map(const std::string& path);                   // throws std::runtime_error on a malformed file
void save_map(const Map& m, const std::string& path);

}  // namespace robotnav
