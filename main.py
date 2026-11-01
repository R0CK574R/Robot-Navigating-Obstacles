"""Command-line entry point.

    python main.py demo                 # compare algorithms on one map + GIF
    python main.py demo --map maze --diagonal
    python main.py benchmark            # full sweep -> CSV + charts
    python main.py benchmark --quick
    python main.py course               # 3D viewer with grassland, desert and island maps
    python main.py route --map desert --start 80,10 --goal 12,85   # same as two clicks in the viewer
    python main.py paths --map island    # the 20 preset paths (More paths in the viewer)
    python main.py route --map island --preset 3
    python main.py terrain-bench        # sweep hilliness / grade limit / obstacles
    python main.py score-route my_route.json   # score a route drawn in the 3D viewer
    python main.py explore --map desert --preset 19 --sensor 5   # drive seeing 5 m, re-planning with D* Lite
    python main.py explore-bench        # every preset at 5, 10 and 20 m: time lost, D* Lite vs A* from scratch
    python main.py fly --map desert --preset 19   # the air route: a drone flies out of the watchtowers' sight
    python main.py route --map desert --preset 19 --engine cpp   # the same, computed by the C++ core
    python main.py export               # map files for the C++ tool (cpp/build/robotnav route maps/desert.map ...)
    python main.py parity               # the C++ core against the Python reference: identical results, and the speed-up
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

from robotnav import ALGORITHMS, maze_grid, random_grid, rooms_grid
from robotnav.grid import solvable
from robotnav.benchmark import plot_benchmark, run_benchmark, run_map_type_benchmark, summarize
from robotnav.visualize import animate_search, plot_comparison

OUT = Path("output")
ROOT = Path(__file__).resolve().parent
PAGES_URL = "https://r0ck574r.github.io/Robot-Navigating-Obstacles/"   # where GitHub Pages serves docs/


def demo(args) -> None:
    OUT.mkdir(exist_ok=True)
    gen = {"random": random_grid, "maze": maze_grid, "rooms": rooms_grid}[args.map]
    kw = dict(diagonal=args.diagonal, seed=args.seed)
    if args.map == "random":
        grid = solvable(gen, args.size, args.size, args.density, terrain=args.terrain, **kw)
    else:
        grid = solvable(gen, args.size, args.size, **kw)

    results = [fn(grid) for fn in ALGORITHMS.values()]
    print(f"{'algorithm':<20}{'cost':>9}{'expanded':>10}{'frontier':>10}{'ms':>9}")
    for r in results:
        print(f"{r.algorithm:<20}{r.cost:>9.2f}{r.nodes_expanded:>10}"
              f"{r.max_frontier:>10}{r.runtime_ms:>9.2f}")

    moves = "8-way" if args.diagonal else "4-way"
    tag = f"{args.map}_{args.size}_{moves}"
    plot_comparison(grid, results, OUT / f"compare_{tag}.png",
                    title=f"{args.map} map · {args.size}×{args.size} · {moves}")
    astar_res = next(r for r in results if r.algorithm == "A*")
    animate_search(grid, astar_res, OUT / f"astar_{tag}.gif")
    print(f"\nSaved images to {OUT.resolve()}")


def benchmark(args) -> None:
    OUT.mkdir(exist_ok=True)
    if args.quick:
        df = run_benchmark(sizes=(20, 40), densities=(0.1, 0.3), trials=3)
    else:
        df = run_benchmark(trials=args.trials)
    df.to_csv(OUT / "benchmark_results.csv", index=False)

    print("\n=== Overall (all conditions) ===")
    print(summarize(df).to_string())
    print("\n=== By movement model ===")
    print(summarize(df, by=("moves", "algorithm")).to_string())

    mt = run_map_type_benchmark(trials=3 if args.quick else 10)
    mt.to_csv(OUT / "map_type_results.csv", index=False)
    print("\n=== By map type (41×41, 4-way) ===")
    print(mt.groupby(["map", "algorithm"])[["nodes_expanded", "runtime_ms", "optimality"]]
            .mean().round(3).to_string())

    for p in plot_benchmark(df, OUT):
        print("chart:", p)


def _cell(text: str) -> tuple[int, int]:
    """'80,10', '80, 10', '80 10' or '(80, 10)' -> (80, 10): the same forms the viewer's position boxes take."""
    parts = [v for v in re.split(r"[,;\s]+", text.strip().strip("()[]").strip()) if v]
    if len(parts) != 2 or not all(re.fullmatch(r"-?\d+", v) for v in parts):
        raise SystemExit(f"Give a cell as row,col (for example 80,10), not {text!r}.")
    return int(parts[0]), int(parts[1])


def _print_results(t, results, extra=()) -> None:
    rows = [(r.algorithm.split(" (")[0], r.cost, t.path_stats(r.path), r.nodes_expanded, r.runtime_ms)
            for r in results if r.found] + list(extra)
    best = min(r[1] for r in rows)
    print(f"  {'planner':<16}{'time':>8}{'vs best':>9}{'dist m':>8}{'climb m':>9}{'rough m':>9}{'searched':>10}{'ms':>8}")
    for name, cost, st, nodes, ms in sorted(rows, key=lambda r: (round(r[1], 6), r[3] is None, r[3] or 0)):
        m, sec = divmod(round(cost), 60)
        print(f"  {name:<16}{m:>5}:{sec:02d}{(cost / best - 1) * 100:>+8.1f}%{st['distance_m']:>8.0f}"
              f"{st['climb_m']:>9.1f}{st['rough_m']:>9.0f}{'—' if nodes is None else nodes:>10}"
              f"{'—' if ms is None else f'{ms:.1f}':>8}")


def _race(t, start, goal, engine=None):
    """The five planners plus the hill-blind shortest route, as the viewer's stage board shows them."""
    if engine is not None:
        return engine.race(t, start, goal)
    from robotnav.terrain import plan_pair, shortest_route
    return plan_pair(t, start, goal) + [shortest_route(t, start, goal)]


def _engine(args):
    """robotnav.cpp (the C++ core) for --engine cpp, None for the Python reference."""
    if getattr(args, "engine", "python") != "cpp":
        return None
    from robotnav import cpp
    try:
        cpp.core()
    except ImportError as e:
        raise SystemExit(str(e))
    return cpp


def course(args) -> None:
    """Build the 3D viewer with the grassland, desert and island maps and their 20 preset paths."""
    from robotnav.scene3d import map_scene, write_viewer
    from robotnav.terrain import THEMES, solvable_map

    OUT.mkdir(exist_ok=True)
    maps = [solvable_map(theme, seed=args.seed, size=args.size, max_grade=args.grade) for theme in THEMES]
    scenes = [map_scene(t) for t in maps]
    out = write_viewer(maps, OUT / "course_3d.html", scenes=scenes)
    site = write_viewer(maps, ROOT / "docs" / "index.html", scenes=scenes, site_url=PAGES_URL)   # the live demo
    (site.parent / ".nojekyll").touch()             # GitHub Pages: serve the files as they are
    for t, sc in zip(maps, scenes):
        ps = sc["presets"]
        longer = [p["short_time"] / p["time"] - 1 for p in ps]
        print(f"{t.meta['theme'].title():<10} (seed {t.meta['seed']}): {len(ps)} preset paths; on them the hill-blind "
              f"shortest route takes {min(longer):.0%} to {max(longer):.0%} longer than the quickest")
    print(f"\n3D viewer: {out.resolve()}  (open in a browser)")
    print(f"Live demo page (GitHub Pages serves docs/): {site.resolve()}")
    print("List a map's paths with: python main.py paths --map grassland")


def route(args) -> None:
    """Race the planners between any two cells, exactly like setting START and FINISH in the viewer."""
    from robotnav.terrain import solvable_map

    t = solvable_map(args.map, seed=args.seed, size=args.size, max_grade=args.grade)
    start, goal = _trip(args, t)
    results = _race(t, start, goal, _engine(args))
    print(f"{t.meta['theme'].title()} (seed {t.meta['seed']}): start {start} -> destination {goal}")
    if not any(r.found for r in results):
        raise SystemExit("No drivable route between those cells: one is walled in by obstacles, water or steep slopes.")
    _print_results(t, results)


def terrain_bench(args) -> None:
    from robotnav.terrain_bench import plot_terrain_benchmark, run_terrain_benchmark

    OUT.mkdir(exist_ok=True)
    df = run_terrain_benchmark(trials=args.trials)
    df.to_csv(OUT / "terrain_results.csv", index=False)
    for factor in ("hilliness", "max_grade", "obstacles"):
        part = df[df.factor == factor]
        print(f"\n=== varying {factor} ===")
        tbl = (part.groupby(["order", "level", "algorithm"])
                   [["success", "time_ratio", "climb_m", "nodes", "ms"]].mean().round(2))
        print(tbl.droplevel("order").to_string())
    for p in plot_terrain_benchmark(df, OUT):
        print("chart:", p)


def paths(args) -> None:
    """List each map's 20 preset paths (More paths in the viewer)."""
    from robotnav.terrain import THEMES, preset_routes, solvable_map

    clock = lambda sec: "{}:{:02d}".format(*divmod(round(sec), 60))
    for theme in ([args.map] if args.map else THEMES):
        t = solvable_map(theme, seed=args.seed, size=args.size, max_grade=args.grade)
        print(f"\n{theme.title()} (seed {t.meta['seed']}): quickest route over the hills vs the hill-blind shortest route")
        print(f"  {'path':<5}{'from → to':<17}{'start':>10}{'destination':>13}{'quickest':>10}{'shortest':>10}"
              f"{'longer':>8}{'dist m':>8}")
        for k, pr in enumerate(preset_routes(t), start=1):
            print(f"  P{k:<4}{pr['name']:<17}{str(pr['start']):>10}{str(pr['goal']):>13}{clock(pr['time']):>10}"
                  f"{clock(pr['short_time']):>10}{pr['short_time'] / pr['time'] - 1:>+8.0%}{pr['distance_m']:>8.0f}")


def _trip(args, t):
    """Start and destination from --preset, or --start/--goal (default: the map's flags), checked like the viewer does."""
    if args.preset is not None and (args.start or args.goal):
        raise SystemExit("Use --preset, or --start and --goal, not both.")
    start = _cell(args.start) if args.start else t.start
    goal = _cell(args.goal) if args.goal else t.goal
    if args.preset is not None:
        from robotnav.terrain import preset_routes
        presets = preset_routes(t)
        if not 1 <= args.preset <= len(presets):
            raise SystemExit(f"Pick a preset from 1 to {len(presets)}.")
        start, goal = presets[args.preset - 1]["start"], presets[args.preset - 1]["goal"]
    for name, (r, c) in (("start", start), ("destination", goal)):
        if not (0 <= r < t.rows and 0 <= c < t.cols):
            raise SystemExit(f"The {name} {(r, c)} is outside the {t.rows}x{t.cols} map.")
        if t.blocked[r, c]:
            raise SystemExit(f"The {name} {(r, c)} is on an obstacle or water. Pick an open cell.")
    if start == goal:
        raise SystemExit("The start and destination are the same cell. Pick two different cells.")
    return tuple(start), tuple(goal)


PRIOR_WORDS = {"none": "knowing nothing beforehand", "elevation": "knowing the hills but not the obstacles",
               "map": "knowing the map but not the roadblocks"}


def explore(args) -> None:
    """Drive with a limited sensor, re-planning with D* Lite: the viewer's D* Lite row."""
    from robotnav.replan import Roadblock, sensor_drive
    from robotnav.terrain import solvable_map

    t = solvable_map(args.map, seed=args.seed, size=args.size, max_grade=args.grade)
    start, goal = _trip(args, t)
    engine = _engine(args)
    blocks = [Roadblock(_cell(b)) for b in (args.block or [])]
    for b in blocks:
        if not (0 <= b.cell[0] < t.rows and 0 <= b.cell[1] < t.cols):
            raise SystemExit(f"The roadblock {b.cell} is outside the {t.rows}x{t.cols} map.")
        if t.blocked[b.cell] or b.cell in (start, goal):
            raise SystemExit(f"A roadblock at {b.cell} would sit on an obstacle, water, the start or the destination.")
    drive = sensor_drive if engine is None else engine.sensor_drive
    run = drive(t, start, goal, sensor_range=args.sensor, prior=args.prior, roadblocks=blocks, check=True)
    clock = lambda sec: "{}:{:02d}".format(*divmod(round(sec), 60))
    print(f"{t.meta['theme'].title()} (seed {t.meta['seed']}): start {start} -> destination {goal}, "
          f"sensor {args.sensor:g} m, {PRIOR_WORDS[args.prior]}" + (f", {len(blocks)} roadblock(s)" if blocks else ""))
    if not run.found:
        raise SystemExit("The rover could not reach the destination: the roadblocks wall it off.")
    cells = [divmod(i, t.cols) for i in run.driven]
    more = run.astar_work / max(1, run.dstar_work)
    print(f"  drive time       {clock(run.time):>7}   {run.extra:+.1%} on A* with the whole map known ({clock(run.optimal)})")
    print(f"  distance         {t.path_stats(cells)['distance_m']:>5.0f} m   in {len(cells) - 1} moves")
    print(f"  re-plans         {run.replans:>7}   times newly sensed ground broke the plan")
    print(f"  D* Lite work     {run.dstar_work:>7,}   cells expanded over the whole drive")
    print(f"  A* from scratch  {run.astar_work:>7,}   cells, planning again each time the plan broke "
          f"({more:.1f}x {'more' if more >= 1 else 'less'})")
    print(f"  sensed           {run.sensed:>7,}   cells ({run.sensed / (t.rows * t.cols):.0%} of the map)")
    print(f"  every repaired plan was exactly as quick as A* from scratch (largest difference {run.max_check_error:.1e} s)")


def explore_bench(args) -> None:
    """Every preset path of every map at several sensor ranges: the time lost by not knowing, and D* Lite's work."""
    import pandas as pd
    from robotnav.replan import sensor_drive
    from robotnav.terrain import THEMES, preset_routes, solvable_map

    OUT.mkdir(exist_ok=True)
    rows = []
    for theme in THEMES:
        t = solvable_map(theme, seed=args.seed)
        for k, pr in enumerate(preset_routes(t), start=1):
            for rng in args.ranges:
                run = sensor_drive(t, pr["start"], pr["goal"], sensor_range=rng, prior=args.prior)
                rows.append(dict(map=theme, path=k, sensor_m=rng, found=run.found, extra=run.extra, replans=run.replans,
                                 dstar_work=run.dstar_work, astar_work=run.astar_work,
                                 work_ratio=run.astar_work / max(1, run.dstar_work), ms=run.runtime_ms))
        print(f"{theme}: done", flush=True)
    df = pd.DataFrame(rows)
    df.to_csv(OUT / "explore_results.csv", index=False)
    print(f"\nDriving with a limited sensor, {PRIOR_WORDS[args.prior]} (seed {args.seed}, 20 preset paths per map)")
    print(f"  {'map':<11}{'sensor':>7}{'time lost':>11}{'worst':>8}{'re-plans':>10}{'D* Lite':>9}{'A* again':>10}{'A*/D*':>8}")
    for (theme, rng), g in df.groupby(["map", "sensor_m"], sort=False):
        print(f"  {theme:<11}{rng:>5g} m{g.extra.median():>+10.0%}{g.extra.max():>+8.0%}{g.replans.median():>10.0f}"
              f"{g.dstar_work.median():>9,.0f}{g.astar_work.median():>10,.0f}{g.work_ratio.median():>7.2f}x")
    print("\n(medians; time lost is the extra drive time over A* with the whole map known; A*/D* above 1 means")
    print(" D* Lite did less work than planning again from scratch with A* each time the plan broke)")
    print(f"Saved {OUT / 'explore_results.csv'}")


def fly_cmd(args) -> None:
    """The air route between two cells: the stealth flight against the direct one, as the viewer's Air mode races them."""
    from robotnav.airspace import AIR, fly, make_airspace
    from robotnav.terrain import solvable_map

    t = solvable_map(args.map, seed=args.seed, size=args.size, max_grade=args.grade)
    start, goal = _trip(args, t)
    engine = _engine(args)
    if engine is not None:
        make_airspace, fly = engine.make_airspace, engine.fly
    air = make_airspace(t)
    flights = [fly(air, start, goal, stealth=True), fly(air, start, goal, stealth=False)]
    clock = lambda sec: "{}:{:02d}".format(*divmod(round(sec), 60))
    print(f"{t.meta['theme'].title()} (seed {t.meta['seed']}): start {start} -> destination {goal}; "
          f"{len(air.towers)} watchtowers see {AIR['sight']:g} m (towers at {', '.join(map(str, air.towers))})")
    print(f"  {'route':<10}{'time':>7}{'in sight':>10}{'dist m':>8}{'top m':>7}{'searched':>10}{'ms':>8}")
    for f in flights:
        print(f"  {f.name:<10}{clock(f.time):>7}{f.seen:>9.1f}s{f.distance:>8.0f}{f.top:>7.1f}{f.nodes:>10,}{f.runtime_ms:>8.1f}")
    st, di = flights
    print(f"Staying hidden costs {st.time / di.time - 1:+.0%} flight time and cuts the time in sight from {di.seen:.1f} s to {st.seen:.1f} s.")


def export(args) -> None:
    """Write each map, with its default flags and its 20 preset trips, to a map file for the C++ tool."""
    from robotnav.mapfile import write_map
    from robotnav.terrain import THEMES, solvable_map

    for theme in ([args.map] if args.map else THEMES):
        t = solvable_map(theme, seed=args.seed, size=args.size, max_grade=args.grade)
        path = write_map(t, Path(args.out) / f"{theme}.map")
        print(f"{path}  ({theme}, seed {t.meta['seed']}, {t.rows}x{t.cols})")
    print(f"The C++ tool reads them, for example: cpp/build/robotnav route {Path(args.out) / 'desert.map'} --preset 19")


def _secs(ms: float) -> str:
    return f"{ms / 1e3:.2f} s" if ms >= 1e3 else f"{ms:.0f} ms" if ms >= 10 else f"{ms:.1f} ms"


def parity(args) -> None:
    """The C++ core against the Python reference on the preset trips: identical results, and how much faster."""
    from robotnav import cpp
    from robotnav.parity import compare_map
    from robotnav.terrain import THEMES, preset_routes, solvable_map

    try:
        cpp.core()
    except ImportError as e:
        raise SystemExit(str(e))
    n = 5 if args.quick else 20
    print(f"C++ core vs the Python reference: seed {args.seed}, {args.size}x{args.size} maps, {n} preset trips each")
    rows = []
    for theme in THEMES:
        t = solvable_map(theme, seed=args.seed, size=args.size)
        print(f"{theme:<10}", end="", flush=True)
        rows.append((theme, compare_map(t, preset_routes(t)[:n], log=print)))
    print(f"\n  {'map':<11}{'edge costs':>14}{'planners':>10}{'path stats':>12}{'D* Lite':>9}{'airspace':>10}{'flights':>9}")
    for theme, res in rows:
        print(f"  {theme:<11}{str(res['edges']):>14}{str(res['planners']):>10}{str(res['stats']):>12}"
              f"{str(res['drives']):>9}{str(res['airspace']):>10}{str(res['flights']):>9}")
    diffs = [d for _, res in rows for tally in res.values() for d in tally.diffs]
    if diffs:
        print("\nDifferences:")
        for d in diffs[:20]:
            print("  " + d)
        raise SystemExit(f"{len(diffs)} result(s) differ.")
    print("\nIdentical, compared with == (no tolerance): every edge cost, route, cell searched (in order), frontier peak,")
    print("drive time and path statistic; every step of every D* Lite drive; the airspace; every flight.")
    print(f"\n  {'speed, all trips together':<44}{'Python':>10}{'C++':>10}{'faster':>9}")
    for key, label in (("planners", "five planners + shortest route"), ("drives", "D* Lite drives, A* baseline included"),
                       ("flights", "air routes, stealth and direct"), ("airspace", "building the airspace")):
        py = sum(res[key].py_ms for _, res in rows)
        cc = sum(res[key].cpp_ms for _, res in rows)
        count = sum(res[key].total for _, res in rows)
        print(f"  {f'{label} ({count})':<44}{_secs(py):>10}{_secs(cc):>10}{py / max(cc, 1e-9):>8.0f}x")


def score_route(args) -> None:
    """Score a route copied from the 3D viewer against the five planners."""
    import json
    from dataclasses import replace

    from robotnav.terrain import evaluate_route, make_map

    data = json.loads(Path(args.file).read_text())
    t = make_map(**data["map"])
    t = replace(t, start=tuple(data["start"]), goal=tuple(data["goal"]), checkpoints=[])
    mine = evaluate_route(t, data["path"])
    if not mine["valid"]:
        raise SystemExit(f"That route is not drivable: {mine['reason']}.")
    print(f"{t.meta['theme'].title()} (seed {t.meta['seed']}): start {t.start} -> destination {t.goal}")
    _print_results(t, _race(t, t.start, t.goal), extra=[("Your route", mine["time"], mine, None, None)])


def main() -> None:
    ap = argparse.ArgumentParser(description="Robot obstacle navigation with pathfinding")
    sub = ap.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("demo", help="run all algorithms on one map and visualize")
    d.add_argument("--map", choices=["random", "maze", "rooms"], default="random")
    d.add_argument("--size", type=int, default=40)
    d.add_argument("--density", type=float, default=0.25)
    d.add_argument("--diagonal", action="store_true", help="allow 8-way movement")
    d.add_argument("--terrain", action="store_true", help="add high-cost rough cells")
    d.add_argument("--seed", type=int, default=7)
    d.set_defaults(func=demo)

    b = sub.add_parser("benchmark", help="sweep conditions and measure performance")
    b.add_argument("--trials", type=int, default=10)
    b.add_argument("--quick", action="store_true")
    b.set_defaults(func=benchmark)

    c = sub.add_parser("course", help="3D viewer with grassland, desert and island maps")
    c.add_argument("--size", type=int, default=96)
    c.add_argument("--grade", type=float, default=0.6, help="max drivable slope (0.6 = 60%%)")
    c.add_argument("--seed", type=int, default=3)
    c.set_defaults(func=course)

    rt = sub.add_parser("route", help="quickest path for each planner between two cells")
    rt.add_argument("--map", choices=["grassland", "desert", "island"], default="grassland")
    rt.add_argument("--start", help="row,col (default: the map's start flag)")
    rt.add_argument("--goal", help="row,col (default: the map's destination flag)")
    rt.add_argument("--preset", type=int, help="plan one of the 20 preset paths (1-20) instead")
    rt.add_argument("--size", type=int, default=96)
    rt.add_argument("--grade", type=float, default=0.6)
    rt.add_argument("--seed", type=int, default=3)
    rt.set_defaults(func=route)

    tb = sub.add_parser("terrain-bench", help="measure planners as terrain gets harder")
    tb.add_argument("--trials", type=int, default=8)
    tb.set_defaults(func=terrain_bench)

    pp = sub.add_parser("paths", help="list the 20 preset paths of each map")
    pp.add_argument("--map", choices=["grassland", "desert", "island"])
    pp.add_argument("--size", type=int, default=96)
    pp.add_argument("--grade", type=float, default=0.6)
    pp.add_argument("--seed", type=int, default=3)
    pp.set_defaults(func=paths)

    sr = sub.add_parser("score-route", help="score a route copied from the 3D viewer")
    sr.add_argument("file", help="JSON file with the copied route")
    sr.set_defaults(func=score_route)

    ex = sub.add_parser("explore", help="drive seeing only a few metres, re-planning with D* Lite")
    ex.add_argument("--map", choices=["grassland", "desert", "island"], default="grassland")
    ex.add_argument("--start", help="row,col (default: the map's start flag)")
    ex.add_argument("--goal", help="row,col (default: the map's destination flag)")
    ex.add_argument("--preset", type=int, help="drive one of the 20 preset paths (1-20) instead")
    ex.add_argument("--sensor", type=float, default=5.0, help="how far the rover sees, in metres (default 5)")
    ex.add_argument("--prior", choices=["none", "elevation", "map"], default="none",
                    help="what it knows beforehand: nothing, the hills only, or the map without the roadblocks")
    ex.add_argument("--block", action="append", metavar="ROW,COL", help="a roadblock not on the map (repeatable)")
    ex.add_argument("--size", type=int, default=96)
    ex.add_argument("--grade", type=float, default=0.6)
    ex.add_argument("--seed", type=int, default=3)
    ex.set_defaults(func=explore)

    fl = sub.add_parser("fly", help="the air route: a drone flies out of the watchtowers' sight")
    fl.add_argument("--map", choices=["grassland", "desert", "island"], default="grassland")
    fl.add_argument("--start", help="row,col (default: the map's start flag)")
    fl.add_argument("--goal", help="row,col (default: the map's destination flag)")
    fl.add_argument("--preset", type=int, help="fly one of the 20 preset paths (1-20) instead")
    fl.add_argument("--size", type=int, default=96)
    fl.add_argument("--grade", type=float, default=0.6)
    fl.add_argument("--seed", type=int, default=3)
    fl.set_defaults(func=fly_cmd)

    for p in (rt, ex, fl):
        p.add_argument("--engine", choices=["python", "cpp"], default="python",
                       help="python (the reference) or cpp (the C++ core, built in cpp/build)")

    xp = sub.add_parser("export", help="write the maps to map files for the C++ tool")
    xp.add_argument("--map", choices=["grassland", "desert", "island"], help="one map (default: all three)")
    xp.add_argument("--out", default="maps", help="folder for the map files (default maps/)")
    xp.add_argument("--size", type=int, default=96)
    xp.add_argument("--grade", type=float, default=0.6)
    xp.add_argument("--seed", type=int, default=3)
    xp.set_defaults(func=export)

    pa = sub.add_parser("parity", help="the C++ core against the Python reference: identical results, and the speed-up")
    pa.add_argument("--quick", action="store_true", help="5 preset trips per map instead of 20")
    pa.add_argument("--size", type=int, default=96)
    pa.add_argument("--seed", type=int, default=3)
    pa.set_defaults(func=parity)

    eb = sub.add_parser("explore-bench", help="sensor drives on every preset: time lost, D* Lite vs A* from scratch")
    eb.add_argument("--ranges", type=float, nargs="+", default=[5.0, 10.0, 20.0], help="sensor ranges in metres")
    eb.add_argument("--prior", choices=["none", "elevation", "map"], default="none")
    eb.add_argument("--seed", type=int, default=3)
    eb.set_defaults(func=explore_bench)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
