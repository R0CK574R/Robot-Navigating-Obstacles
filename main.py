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


def _race(t, start, goal):
    """The five planners plus the hill-blind shortest route, as the viewer's stage board shows them."""
    from robotnav.terrain import plan_pair, shortest_route
    return plan_pair(t, start, goal) + [shortest_route(t, start, goal)]


def course(args) -> None:
    """Build the 3D viewer with the grassland, desert and island maps and their 20 preset paths."""
    from robotnav.scene3d import map_scene, write_viewer
    from robotnav.terrain import THEMES, solvable_map

    OUT.mkdir(exist_ok=True)
    maps = [solvable_map(theme, seed=args.seed, size=args.size, max_grade=args.grade) for theme in THEMES]
    scenes = [map_scene(t) for t in maps]
    out = write_viewer(maps, OUT / "course_3d.html", scenes=scenes)
    for t, sc in zip(maps, scenes):
        ps = sc["presets"]
        longer = [p["short_time"] / p["time"] - 1 for p in ps]
        print(f"{t.meta['theme'].title():<10} (seed {t.meta['seed']}): {len(ps)} preset paths; on them the hill-blind "
              f"shortest route takes {min(longer):.0%} to {max(longer):.0%} longer than the quickest")
    print(f"\n3D viewer: {out.resolve()}  (open in a browser)")
    print("List a map's paths with: python main.py paths --map grassland")


def route(args) -> None:
    """Race the planners between any two cells, exactly like setting START and FINISH in the viewer."""
    from robotnav.terrain import solvable_map

    if args.preset is not None and (args.start or args.goal):
        raise SystemExit("Use --preset, or --start and --goal, not both.")
    t = solvable_map(args.map, seed=args.seed, size=args.size, max_grade=args.grade)
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
    results = _race(t, start, goal)
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

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
