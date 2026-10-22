# Rover Hill Course — 3D Off-Road Robot Navigation

A tracked rover, a matte-black rhomboid tank with a hedgehog coat of steel spikes, finds its way
across three hilly maps (grassland, desert dunes, and a tropical island) using five pathfinding
algorithms. Each map opens empty: add any of its 20 preset routes, or set your own start and
finish, and the rover works out the quickest path over the hills while a follow drone films it.
A dashed line shows the shortest route a robot that ignores hills would take, so you can see what
reading the terrain buys you. Python generates the maps, picks the routes, and is the reference
implementation of every planner.

![The viewer on the desert map: the tank on its quickest route, the planners ranked on the stage board](docs/screenshot.png)

## Quick start

```bash
pip install -r requirements.txt
python main.py course                     # 3D viewer with all three maps -> output/course_3d.html (~15 s)
python main.py paths --map island         # the 20 preset paths behind "More paths"
python main.py route --map island --preset 3   # race the planners on preset P3
python main.py route --map desert --start 80,10 --goal 12,85   # same START and FINISH as in the viewer
python main.py score-route route.json     # score a route you drew in the viewer
python main.py terrain-bench              # hills / slope limit / obstacles sweep (~25 s)
python -m pytest -q
```

Open `output/course_3d.html` in any browser. It needs internet once to load Three.js from a CDN.
The original 2D grid version still works: `python main.py demo`, `python main.py benchmark`.

## Using the viewer

- **Pick a map**: Grassland, Desert or Island (top left). Every map opens empty, in Overview, with
  the rover parked in the middle and no route on the board.
- **More paths** (next to Set start & finish): 20 routes per map, all optional. Tick one and its quickest
  path over the hills is drawn as a labelled line (P1–P20). The first path you add starts the rover
  on it. **Drive** on any row sends the rover along that path with the drone following, and the
  stage board races the planners on it. Each row shows the quickest time against the shortest
  route's time (for example `1:21 vs 1:59`). **Add all** draws every path; **Remove all** clears
  the map again. Taking off the path being driven switches the rover to the last path you added.
- **The shortest route** (dashed white): the least-distance route between the same two points,
  which is what a robot that ignores hills would drive. The board ranks it with the planners, and
  the green line under the table spells out the difference, such as "1:21 instead of 1:59 on the
  shortest route, 39 s saved". The **Shortest route** chip hides the dashed line; **All planners**
  shows every planner's route; **Search area** shows the cells the selected planner looked at.
- **Set start & finish**: the camera rises to a top-down map with a START and a FINISH marker you
  can set in any order and move one at a time. Three ways, all equivalent:
  - **Click the map**: the next click places whichever marker is chosen (the **Start** / **Finish**
    buttons). After the START, the FINISH is chosen automatically; after that, clicks keep moving
    the FINISH. A ring under the pointer shows where it would land (red where it can't), and a
    readout gives the exact row, column and ground.
  - **Drag a flag**: press on a START or FINISH flag (its ring or its label) and drag it. A plain
    click on a flag chooses it. Dragging anywhere else still moves the map.
  - **Type a position**: `row, col` in the box next to each button (`40, 12`, `40 12` and
    `(40, 12)` all work), then Enter. Escape undoes the typing.

  Every placement is checked: water, rocks, trees, the other marker's cell, spots off the map,
  and walled-in pockets (cut off by obstacles or slopes too steep to drive) are refused with the
  reason. As soon as both markers are down, all five planners and the shortest route are planned
  and drawn; any later move re-plans at once. **⇄** swaps start and finish. Uphill becomes downhill,
  so the times change. **Drive** sends the rover; Escape or **Close** leaves. The positions use
  the same `row, col` as `python main.py route --start … --goal …`, which gives identical results.
- **Draw your own**: in the planner, once there's a start and finish, draw a route by
  clicking points (each joins the last with a straight line). Lines that hit an obstacle or a
  too-steep slope are refused with the reason. **Drive it** ranks your route against the planners,
  and **Copy your route** gives JSON you can score in Python.
- **Cameras**: Drone (follows from ~15 m up), Top-down, Chase, Overview. The Overview fits the whole
  map into the space the panels leave free, so every path label stays visible. Far views draw
  thicker lines, close views thin ones. The added paths, the route being driven and a drawn route
  are remembered per map in your browser.

## The rover

The rover is modelled on the British Mark IV/V "male" tank of 1917–18, at about a quarter scale (1.9 m long):

![The tank from the front, the back and above](output/rover_design.png)

- **Rhomboid hull**: the tracks run right round each side frame (high front horns, a long sloped
  nose, a flat top run, a sloped rear). The track is 55 plates a side, each with raised grousers.
  They move with the ground speed, so the plates under the tank stay put on the ground while it drives.
- **Sponsons** on both sides with an angled lower face and a short gun in a ball mount. The guns
  swing toward the next bend and sweep from side to side at the finish.
- **A raised cab** at the front of the roof with a hatch, vision ports with dark slits, pistol ports,
  a rear door, and about 480 rivets along the plate edges and seams.
- **Spikes**: about 270 steel spikes cover the roof and the cab top. Each one leans back like a
  hedgehog's quill and fans outward toward the sides.

Everything except the spikes is matte black: a physically based paint (rough, non-metallic) that
picks up only a soft sheen from the sky. The spikes are polished steel (fully metallic, low
roughness). They reflect an environment map built from each map's sky and toned toward grey, so
they read as steel on every map.

## The maps

| Map | Terrain | Surfaces (time factor) | Obstacles |
|---|---|---|---|
| Grassland | rolling green hills, up to 12 m | grass 1.0, tall grass 1.8 | trees, rock clusters |
| Desert | dune field: gentle windward slopes, steep slip faces | firm sand 1.0, soft sand 2.4, rocky ground 1.3 | cacti, sandstone boulders |
| Island | ragged coast, beach, central peak | grass 1.0, sand 1.6, jungle 2.0 | palm trees, rocks, the sea |

## Cost model: drive time

The rover cruises at 1 m/s on the fastest surface and slows on rough ground and climbs:

    seconds per move = 3D distance × surface factor × (1 + 3 × uphill grade)

Moves steeper than the 60% grade limit, into rocks, trees or water, or squeezing diagonally
between two obstacles are impossible. Every surface factor is at least 1, so the octile
distance never overestimates and A* always finds the quickest route. The viewer plays the
rover back by this clock, so it visibly slows in soft sand and on climbs.

## The shortest route: a robot that ignores hills

`shortest_route()` in `robotnav/terrain.py` runs Dijkstra with each move priced by its flat
distance. It obeys the same rules as the rover (no obstacles, no water, no slope over the grade
limit), but it doesn't care about climbs or soft ground. Many routes on a grid are equally short,
so each move also carries a hair (1e-6) of its drive time: among equally short routes it takes
the quickest. That makes it a fair baseline rather than an unlucky pick. Its time is then measured
with the real drive-time model, so it can be ranked against the planners.

## How the 20 preset paths are picked

`preset_routes()` looks for routes that show hill-aware planning at its clearest:

- **Long and crossing**: start and destination at least about half the land's width apart (the
  island's limit is its coastline, not the map edge), in every direction across the map.
- **Shaped by the hills**: the quickest path must clearly beat the shortest route, and visibly split
  from it, running several metres apart for much of the way.
- **New ground**: each pick may share only a little of its path with the routes picked before it,
  and endpoints are kept apart so the labels don't pile up.

Candidates are screened fast: a whole-map Dijkstra from about 500 lattice cells (scipy) gives every
pair's quickest time and the time its shortest route really takes. The best are then re-planned
exactly with A* and `shortest_route`, the same searches the browser runs. If a map is too small
for 20 such routes, the limits loosen step by step. There's no randomness, so the list is the
same every time, sorted from quickest to slowest, and `python main.py paths` matches the panel
row for row.

## Python and the browser give identical answers

Driving a route can't wait for Python, so the browser runs the five planners and the shortest
route too: a line-for-line port with the same neighbour order, costs, heuristic and heap
tie-breaking. A headless-browser test ran 90 random start/finish pairs across the three maps:
every route was identical cell for cell, every planner searched exactly as many cells as in
Python, and drive times matched to within 1e-13 s. All 60 preset lines embedded in the viewer are
identical to the browser's own A* path. Every way of setting a start and finish (clicks, typed
positions, flag drags with mouse and touch, swapping) was checked the same way: after each one, the
browser's six routes matched Python's cell for cell, and `main.py route` printed the same times.
Hand-drawn routes are scored by the same rules in both (`robotnav.terrain.evaluate_route`).

## Layout

| File | What it does |
|---|---|
| `robotnav/terrain.py` | Height maps, drive-time model, the three themed maps (`make_map`), `plan_pair`, `shortest_route`, `preset_routes`, `evaluate_route` |
| `robotnav/algorithms.py` | BFS, Dijkstra, A*, Weighted A*, Greedy — one shared priority-queue engine |
| `robotnav/scene3d.py` | Exports the three maps and their preset paths into the self-contained viewer |
| `robotnav/viewer3d.html` | Three.js viewer: maps, planners in the browser, preset paths, picking, drawing, drone camera, the spiked tank |
| `robotnav/terrain_bench.py` | Sweeps hill height, slope limit and obstacle density |
| `robotnav/grid.py`, `visualize.py`, `benchmark.py` | Original 2D grid world, plots and benchmark |
| `tests/` | 139 checks: optimality on every map, the shortest route, all 20 presets per map, grade limit, water, route rules, map rebuilds, the `row,col` command line |

## Findings

Over each map's 20 preset paths (seed 3), how much longer each approach takes than A*
(median, and the worst case). A* and Dijkstra always tie for the quickest time.

| Map | Weighted A* | Greedy | BFS | Shortest route |
|---|---|---|---|---|
| Grassland | +8% (up to +22%) | +38% (up to +65%) | +27% (up to +52%) | +29% (up to +48%) |
| Desert | +5% (up to +43%) | +58% (up to +138%) | +53% (up to +110%) | +55% (up to +81%) |
| Island | +6% (up to +40%) | +32% (up to +79%) | +28% (up to +66%) | +27% (up to +61%) |

A* searched a median of 950–2,100 cells per route, Dijkstra 3,200–6,300, Weighted A* 160–300 and
Greedy 50–70.

- **Reading the hills matters most in the desert**: soft sand on the dune crests and steep slip
  faces make the hill-blind shortest route take a median 55% longer, and up to 81% longer.
- **Short isn't quick**: on these routes the shortest route always loses to A*, though it drives
  fewer metres.
- **Greedy** heads straight for the goal and often ends up slower than even the hill-blind
  shortest route. In the desert it once took 2.4× the optimal time.
- **Weighted A\*** is the best trade-off: usually within 5–8% of the optimum while searching about
  a sixth of A*'s cells.

From the terrain benchmark (64 m courses, 12 trials per level):

- **A\*** always matched Dijkstra's optimal time while searching about 3× fewer cells.
- **Its advantage shrinks on taller hills**: A* searched ~1,100 cells at 4 m hills but ~2,800 at
  20 m, because flat distance underestimates climbing time more as hills grow.
- **Hills limit reachability more than obstacles**: only a third of courses with 20 m hills
  were drivable at a 60% grade limit, while even "very dense" rocks and trees left every course drivable.

## Ideas for next steps

- Animate the search: show A* expanding cells before the path appears
- Moving obstacles and replanning (D* Lite)
- A smarter heuristic that counts the climb to the destination's elevation
- Sensor range: the rover only sees nearby terrain and replans as it explores
