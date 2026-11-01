// The air route: a drone flies a 3-D route over the map and keeps out of the watchtowers' sight.
// A C++ port of robotnav.airspace.
//
// The airspace is a stack of flight levels over the map's grid, `dz` metres apart, the first a little above the
// lowest ground. A voxel is free if it clears the ground, trees, rocks and water below it by `clear` metres.
// Watchtowers on the hilltops see `sight` metres, along a clear line of sight only (terrain masking). Moves go to
// any of the 26 neighbouring voxels as long as every voxel the move passes beside is free; the stealth route also
// pays `exposure` seconds for every second in sight. Both routes are found with A* and a straight-line estimate.
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "robotnav/map.hpp"

namespace robotnav {

// What the drone and the towers are like (robotnav.airspace.AIR; the viewer gets the same numbers).
struct AirParams {
    int levels = 8;            // flight levels
    double dz = 3.0;           // metres between levels
    double clear = 2.0;        // clearance over whatever is below
    double tree = 6.0;         // how tall trees stand
    double rock = 1.2;         // and rocks
    int towers = 4;            // watchtowers
    double spacing = 30.0;     // at least this far apart (metres)
    double eye = 6.0;          // eye height over the ground
    double sight = 24.0;       // how far they see
    double speed = 5.0;        // cruise speed (m/s)
    double climb = 0.2;        // seconds per metre climbed
    double exposure = 10.0;    // seconds the stealth route pays per second in sight
};
inline const AirParams AIR{};

// The 26 moves (dr, dc, dk): level first, then straight up and down, then climbing and descending.
struct Move {
    int dr, dc, dk;
};
const std::vector<Move>& air_moves();

struct Airspace {
    int L = 0, R = 0, C = 0;                         // levels, rows, columns
    std::vector<std::pair<int, int>> towers;         // (row, col)
    std::vector<double> alt;                         // altitude of each level (metres)
    std::vector<char> free, seen;                    // per voxel ((level * rows) + row) * cols + col
    std::vector<double> eyes;                        // each tower's eye height (metres)
    std::vector<double> top;                         // per cell: what the drone must clear

    int voxel(int cell) const;                       // the lowest free voxel over a cell (take-off and landing)
};

// What the drone must clear and what blocks a tower's view: the ground or water, plus any tree or rock.
std::vector<double> ground_top(const Map& m);

// Watchtowers on the highest open ground, at least `spacing` metres apart, away from the map's edges.
std::vector<std::pair<int, int>> place_towers(const Map& m, int n = AIR.towers, double spacing = AIR.spacing);

// The airspace over a map, with its watchtowers (on the hilltops unless given).
Airspace make_airspace(const Map& m);
Airspace make_airspace(const Map& m, const std::vector<std::pair<int, int>>& towers);

bool move_ok(const Airspace& air, int r, int c, int k, int dr, int dc, int dk);
double move_time(int dr, int dc, int dk);

struct Flight {
    std::string name;                                // "Stealth" or "Direct"
    std::vector<int> voxels;
    bool found = false;
    double cost = kInf;                              // what the planner minimised (seconds, exposure included)
    double time = kInf;                              // flight time (seconds)
    double seen = kInf;                              // seconds of it in some tower's sight
    long nodes = 0;                                  // voxels expanded
    double runtime_ms = 0.0;
    double distance = 0.0;                           // metres flown
    double top = 0.0;                                // highest altitude above the ground below (metres)
};

// The quickest route from the ground at `start` to the ground at `goal` (flat cell indices). stealth also pays for
// time in sight; heuristic = false searches without the straight-line estimate (Dijkstra), to check A*.
Flight fly(const Airspace& air, int start, int goal, bool stealth = true, bool heuristic = true);

struct FlightStats {
    double time = 0.0, seen = 0.0, distance = 0.0, top = 0.0;
};
FlightStats flight_stats(const Airspace& air, const std::vector<int>& voxels);

}  // namespace robotnav
