# robotnav C++ core

The route-deciding code of the project in C++17: the five planners (BFS, Dijkstra, A\*, Weighted A\*,
Greedy), the hill-blind shortest route, D\* Lite driving with a limited sensor, and the drone's air route
with the watchtowers' lines of sight. It gives the same answers as the Python reference to the last bit:
the same routes, the same cells searched in the same order, the same frontier peaks and the same times,
compared with `==`, no tolerance. On the 60 preset trips it runs the planners 68 times faster than
Python, the D\* Lite drives 27 times and the flights 66 times.

Python still generates the maps (NumPy and SciPy noise, the oases, the 20 preset trips) and hands them
over either directly (the `robotnav_cpp` module) or as a map file (`python main.py export`).

## Build

You need CMake 3.20+ and a C++17 compiler (GCC, Clang, Apple Clang or MSVC). From the project folder:

```bash
pip install pybind11
cmake -S cpp -B cpp/build
cmake --build cpp/build
ctest --test-dir cpp/build
```

That builds three things in `cpp/build`:

| Target | What it is |
|---|---|
| `robotnav` | The command-line tool. It reads a map file and prints the same tables as the Python commands |
| `robotnav_tests` | Unit tests on small hand-made maps (`ctest` runs them) |
| `robotnav_cpp` | The Python module (needs pybind11; skipped with a message if it's missing) |

## Run

```bash
cpp/build/robotnav route maps/desert.map --preset 19
cpp/build/robotnav explore maps/island.map --preset 20 --sensor 5 --block 40,50
cpp/build/robotnav fly maps/grassland.map --preset 15
cpp/build/robotnav paths maps/island.map
cpp/build/robotnav bench maps/grassland.map maps/desert.map maps/island.map
```

`maps/` holds the three maps of the live demo, so the tool runs without Python too (`python main.py export`
writes them again, with `--seed` and `--size` for other maps). Each command prints exactly what
`python main.py <command> --map <map> ...` prints, except the "ms" timings.

From Python, every command that plans can use the C++ core instead (`--engine cpp`), and
`python main.py parity` holds the two side by side on every preset trip.

## Layout

| File | What it does |
|---|---|
| `include/robotnav/map.hpp`, `src/map.cpp` | The map and the drive-time model: edge costs, legal moves in neighbour order, the octile estimate, route statistics |
| `include/robotnav/planners.hpp`, `src/planners.cpp` | One best-first engine (a template over the graph and the priority) behind Dijkstra, A\*, Weighted A\*, Greedy and the shortest route; BFS |
| `include/robotnav/replan.hpp`, `src/replan.cpp` | D\* Lite (the optimised version), the rover's belief, the sensor drive with roadblocks, the A\*-from-scratch baseline |
| `include/robotnav/airspace.hpp`, `src/airspace.cpp` | Flight levels, watchtowers and their lines of sight, 3-D A\* for the stealth and direct flights |
| `include/robotnav/mapio.hpp`, `src/mapio.cpp` | Map files (the format is described in the header) |
| `src/cli.cpp` | The `robotnav` tool |
| `bindings/module.cpp` | The pybind11 module `robotnav_cpp` (`robotnav/cpp.py` wraps it) |
| `tests/test_core.cpp` | Unit tests |

## How it stays identical

The same algorithm isn't enough: two programs that add the same numbers in a different order, or round
once instead of twice, drift apart in the last bit. Ties on equal costs then break differently and the
routes split. So the port keeps every rule the Python reference follows:

- **The same operations in the same order**: `sqrt(run * run + rise * rise)` (not `hypot`, whose rounding
  differs between libraries), the same grouping of every product and sum, and drive times added up left
  to right. Python's own `sum()` has compensated for rounding since 3.12, so the reference adds with a
  plain loop (`robotnav.terrain.add_up`), as C++ and the browser do.
- **No fused multiply-add, no fast-math**: CMake builds with `-ffp-contract=off` (`/fp:precise` on MSVC).
  Otherwise a compiler may fuse `a * b + c` into one instruction that rounds once, as GCC and Clang do on
  ARM (Apple Silicon) and on x86 with `-march=native`. Built that way, on the 60 preset trips 51 of 360
  planner times changed in their last bits and D\* Lite expanded a different number of cells on 31 of
  60 drives. The routes happened to survive, but only by luck: one tie broken the other way changes
  them. With contraction off, GCC `-O0`, GCC `-O2 -march=native` and Clang `-O3 -march=native` all
  give the same bits.
- **The same neighbour order and tie-breaking**: up, down, left, right, then the diagonals; the frontier
  pops the lowest priority and, among equals, the first pushed, just as Python's `heapq` does with its
  `(priority, counter, cell)` tuples. D\* Lite's queue orders by `(k1, k2, stamp)` with lazy deletion.
- **Integers where Python compares integers**: tower spacing and sensor disks compare squared integer
  distances, which are exact in any language.
- **Numbers carried exactly**: the map file writes heights with Python's `repr` (the shortest text that
  reads back to the same double) and C++ reads them with `strtod`, which rounds correctly.
