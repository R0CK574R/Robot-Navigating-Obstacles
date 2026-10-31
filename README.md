# Rover Hill Course — 3D Off-Road Robot Navigation

**[▶ Open the live demo](https://r0ck574r.github.io/Robot-Navigating-Obstacles/)**: it runs in your browser, on a laptop or a phone,
with nothing to install. Press **▶ Tour** for a guided one-minute run through all three maps.

A tracked rover, a matte-black rhomboid tank with a hedgehog coat of steel spikes, finds its way
across three hilly maps (grassland, desert dunes with oasis ponds, and a tropical island) using five pathfinding
algorithms. Each map opens empty: add any of its 20 preset routes, or set your own start and
finish, and the rover works out the quickest path over the hills while a follow drone films it,
at golden hour under a pastel sky.
A dashed line shows the shortest route a robot that ignores hills would take, so you can see what
reading the terrain buys you. Python generates the maps, picks the routes, and is the reference
implementation of every planner.

![The tour on the desert map: the camera flies in over the dunes and oases, five planners race the same trip, and the drone follows the tank along the quickest route](docs/demo.gif)

## Show it to someone

- **The live demo** (https://r0ck574r.github.io/Robot-Navigating-Obstacles/) is the same page as `output/course_3d.html`,
  served by GitHub Pages from `docs/`. Shared links show a preview card with a picture of the
  desert oasis.
- **▶ Tour** (top left, or add `#tour` to the address) plays a guided loop: each map flies in,
  five planners race its showcase path, the drone and chase cameras follow the rover, and the
  overview lays every planner's route side by side. A caption says what's happening at each step.
  Touch anything to take over; the tour doesn't change the paths your browser remembers.
- **Links to a view**: the address keeps up with what's on screen, so you can copy it and send it.
  [`#desert/p19/drone`](https://r0ck574r.github.io/Robot-Navigating-Obstacles/#desert/p19/drone) opens the desert's path P19 past
  both oases with the drone camera, [`#island/p20/chase`](https://r0ck574r.github.io/Robot-Navigating-Obstacles/#island/p20/chase)
  rides along the island's west coast, and `#tour/island` starts the tour on the island.

## Run it yourself

You need [Python 3.10 or newer](https://www.python.org/downloads/) and [Git](https://git-scm.com/downloads).
Paste the lines one block at a time into a terminal (Terminal on Mac, PowerShell on Windows).

**1. Download the project**

```bash
git clone https://github.com/R0CK574R/Robot-Navigating-Obstacles.git
cd Robot-Navigating-Obstacles
```

**2. Install the packages** in a virtual environment

Mac or Linux:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Windows (PowerShell):

```powershell
py -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

If PowerShell says running scripts is disabled, run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`
once and activate again.

**3. Build the 3D viewer** (takes about 15 seconds)

```bash
python main.py course
```

**4. Open it in your browser**

Mac:

```bash
open output/course_3d.html
```

Windows:

```powershell
start output\course_3d.html
```

Or double-click `output/course_3d.html` in your file browser. The page needs internet the first
time to load the 3D library (Three.js). Pick a map at the top left, then click **More paths** or
**Set start & finish** to send the tank driving.

> Just want to look? Open the [live demo](https://r0ck574r.github.io/Robot-Navigating-Obstacles/), or download
> `output/course_3d.html` from the repo and open it. Neither needs Python.

### More commands

Run these from the same terminal, with the virtual environment active.

| Command | What it does |
|---|---|
| `python main.py paths --map island` | List the 20 preset paths behind "More paths" |
| `python main.py route --map island --preset 3` | Race the five planners and the shortest route on preset P3 |
| `python main.py route --map desert --start 80,10 --goal 12,85` | Same, for your own START and FINISH (the `row, col` shown in the viewer) |
| `python main.py score-route route.json` | Score a route you drew in the viewer and copied |
| `python main.py terrain-bench` | Hills, slope limit and obstacle sweep (about 25 seconds) |
| `python -m pytest -q` | Run the 143 tests |
| `python main.py demo`, `python main.py benchmark` | The original 2D grid version |

`python main.py course` writes the viewer twice: `output/course_3d.html` to open yourself, and
`docs/index.html`, the live demo, which adds the tags for link previews.

## Using the viewer

- **Pick a map**: Grassland, Desert or Island (top left). Every map opens with a short fly-in to the
  Overview, empty, with the rover parked in the middle and no route on the board.
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
  thicker lines, close views thin ones. Leaves and cacti between the camera and the rover dissolve,
  so a plant never hides it. The added paths, the route being driven and a drawn route are
  remembered per map in your browser.

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
| Desert | dune field: gentle windward slopes, steep slip faces, three oases | firm sand 1.0, soft sand 2.4, rocky ground 1.3, oasis grass 1.0 | cacti, sandstone boulders, oasis ponds and palms |
| Island | ragged coast, beach, central peak | grass 1.0, sand 1.6, jungle 2.0 | palm trees, rocks, the sea |

![The viewer on the desert map at golden hour: the tank drives its quickest route past an oasis of palms toward the finish flag on a dune, under a sky-blue and pink sky, with the planners ranked on the stage board](docs/screenshot.png)

The desert's oases are carved in Python, so they are real obstacles: the planners route around them
and the browser and Python still agree. Each sits in the lowest broad ground between the dunes, stretched
along the troughs like a real interdune lake, in a basin levelled over about 8 m so its shores stay
drivable. It's filled to just below its rim, so it can never spill. Firm wet sand, a patchy ring of
oasis grass and palms surround it.

## How the water is drawn

The island's sea and the desert's ponds share one water surface, drawn in a single pass. It is a
Three.js `MeshStandardMaterial` (so it gets the sun, shadows, fog and a small sky map to reflect),
with its shader extended in `viewer3d.html`:

- **Ripples**: a few travelling waves, warped and stronger in gusty patches, bend the surface per pixel.
- **Sun glitter**: a 128×128 tiling texture of random wave slopes, made once at startup and sampled
  as two layers drifting different ways, supplies tiny facets that flash when they mirror the sun into
  the camera. Its mipmaps calm it with distance, so far water doesn't flicker, and the broad sun
  reflection is kept soft so the sparkles carry it.
- **Clear water**: colour and opacity follow the depth, so the shallows show the sandy bed (with
  moving caustics) and deep water turns dark teal. The depth is a vertex attribute: the water mesh
  reuses the terrain's grid and triangle split, so it matches the ground exactly and the shoreline
  falls where the ground meets the water.
- **Foam**: a thin, lapping rim at the waterline (a wider surf line on the island).

There's no second camera rendering a mirror reflection or the scene under the water, which is
what usually makes water expensive. In software rendering, with the sea filling the screen, the
full effect costs under 1 ms more per frame than a plain surface.

## The sky, light and scenery

Every texture below is drawn in code when the page starts. The page still loads nothing but Three.js.

- **Sky**: a dome round the camera with a pastel gradient (sky blue overhead, lavender, light pink at
  the horizon), a warm glow toward a low sun about 35° up, the sun's disc, and a slow deck of soft
  clouds from a tiling noise texture. The light matches: a warm sun and a lavender sky fill, so
  shadows go cool. When the camera looks down at a floating map, the dome tips up, so the map
  floats in front of sky rather than the empty haze below the horizon. The island's sea runs to the
  real horizon instead.
- **Haze that matches the sky**: Three.js's fog shader chunks are replaced, so every material fades
  toward the sky's colour in that direction, warmer toward the sun, and distant hills melt into the
  horizon behind them. Water and the tank's steel reflect a small cube map of the same sky.
- **Palms** on the island and round the oases are modelled on the Canary Island date palm: a stout
  trunk with diamond leaf scars, the "pineapple" knob under the crown, and 36 feathery fronds on a
  golden-angle spiral. Young fronds stand up and old ones hang down. Each frond is a V-shaped strip
  textured with leaflets and alpha-tested, so it casts a leafy shadow.
- **Broadleaf trees** have crowns of leafy clumps that shade as one soft mass, and the **cacti** are
  ribbed saguaros with spines along the ribs. Leaf alpha is scaled up with the mipmap level, so
  crowns stay full from far away instead of thinning to twigs.
- **Wind**: palms, tree crowns and grass sway in the vertex shader, after instancing, so gusts roll
  across the map and the shadows sway with them.
- **Ground**: smooth-shaded and textured per pixel from world position. Grass is mottled at three
  scales, sand carries wind ripples (crests across the wind, gone on steep faces and far away), and
  steep or rocky ground is lumpy. Grassy cells grow seven-blade tufts, taller and denser in tall
  grass and jungle, so slow ground reads at a glance.
- **Shadows**: the sun's 2048×2048 shadow map covers the whole map from afar and a tight box round
  the rover from close by, so its spikes and treads cast crisp shadows. The box moves in whole
  shadow-map texels, so shadow edges don't crawl as it follows.
- **Quality governor**: if frames stay under about 40 fps for a few seconds, the viewer steps down
  once at a time: render scale, then soft leaf edges, then grass and shadow-map size. Add
  `?quality=high` to the address to keep full quality.

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
| `robotnav/terrain.py` | Height maps, drive-time model, the three themed maps (`make_map`) and the desert's oases, `plan_pair`, `shortest_route`, `preset_routes`, `evaluate_route` |
| `robotnav/algorithms.py` | BFS, Dijkstra, A*, Weighted A*, Greedy — one shared priority-queue engine |
| `robotnav/scene3d.py` | Exports the three maps and their preset paths into the self-contained viewer (and the live demo page, with link previews) |
| `robotnav/viewer3d.html` | Three.js viewer: maps, sky, water, palms and trees, planners in the browser, preset paths, picking, drawing, drone camera, the spiked tank, the tour, links to a view |
| `docs/` | The live demo served by GitHub Pages (`index.html`, built by `python main.py course`), its preview image, and the README's pictures |
| `robotnav/terrain_bench.py` | Sweeps hill height, slope limit and obstacle density |
| `robotnav/grid.py`, `visualize.py`, `benchmark.py` | Original 2D grid world, plots and benchmark |
| `tests/` | 143 checks: optimality on every map, the shortest route, all 20 presets per map, the desert's oases, grade limit, water, route rules, map rebuilds, the `row,col` command line, the page head and link previews |

## Findings

Over each map's 20 preset paths (seed 3), how much longer each approach takes than A*
(median, and the worst case). A* and Dijkstra always tie for the quickest time.

| Map | Weighted A* | Greedy | BFS | Shortest route |
|---|---|---|---|---|
| Grassland | +8% (up to +22%) | +38% (up to +65%) | +27% (up to +52%) | +29% (up to +48%) |
| Desert | +9% (up to +34%) | +56% (up to +86%) | +47% (up to +100%) | +42% (up to +82%) |
| Island | +6% (up to +40%) | +32% (up to +79%) | +28% (up to +66%) | +27% (up to +61%) |

A* searched a median of 950–2,100 cells per route, Dijkstra 3,200–6,300, Weighted A* 160–320 and
Greedy 50–70.

- **Reading the hills matters most in the desert**: soft sand on the dune crests, steep slip faces
  and the oases make the hill-blind shortest route take a median 42% longer, and up to 82% longer.
- **Short isn't quick**: on these routes the shortest route always loses to A*, though it drives
  fewer metres.
- **Greedy** heads straight for the goal and often ends up slower than even the hill-blind
  shortest route. In the desert it once took 1.9× the optimal time.
- **Weighted A\*** is the best trade-off: usually within 6–9% of the optimum while searching about
  a sixth of A*'s cells or fewer.

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
