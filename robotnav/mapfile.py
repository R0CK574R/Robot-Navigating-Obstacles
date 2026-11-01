"""Map files: a generated map, its default flags and its preset trips as plain text, for the C++ tools.

`python main.py export` writes one per map; the C++ command-line tool (cpp/build/robotnav) reads them. The format
is described in cpp/include/robotnav/mapio.hpp. Floats are written with repr, the shortest text that reads back
to the very same double, so the C++ side computes on exactly the numbers Python does.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .terrain import NONE, SHORTEST_TIE, Terrain, preset_routes


def write_map(t: Terrain, path, presets=None) -> Path:
    """Write `t` (with its preset trips: preset_routes(t) unless given) to `path`."""
    if not np.array_equal(t.blocked, t.obstacle != NONE):
        raise ValueError("a map file stores obstacles, so the map's blocked cells must be exactly its obstacles")
    presets = preset_routes(t) if presets is None else presets
    num = lambda v: repr(float(v))
    lines = ["robotnav-map 1",
             f"theme {t.meta.get('theme', 'course')}",
             f"seed {t.meta.get('seed', 0)}",
             f"rows {t.rows}",
             f"cols {t.cols}",
             f"cell {num(t.cell_size)}",
             f"max_grade {num(t.max_grade)}",
             f"uphill {num(t.uphill_penalty)}",
             f"short_tie {num(SHORTEST_TIE)}",
             "surface_costs " + " ".join(num(c) for c in t.surface_costs),
             "surface_names " + "|".join(t.surface_names),
             "sea " + ("none" if t.sea_level is None else num(t.sea_level)),
             f"start {t.start[0]} {t.start[1]}",
             f"goal {t.goal[0]} {t.goal[1]}",
             f"ponds {len(t.ponds)}"]
    lines += [f"pond {num(p['level'])} {len(p['cells'])} " + " ".join(f"{r} {c}" for r, c in p["cells"]) for p in t.ponds]
    lines.append(f"presets {len(presets)}")
    lines += [f"preset {p['start'][0]} {p['start'][1]} {p['goal'][0]} {p['goal'][1]} {p['name']}" for p in presets]
    lines.append("height")
    lines += [" ".join(num(v) for v in row) for row in t.height]
    lines.append("surface")
    lines += [" ".join(str(int(v)) for v in row) for row in t.surface]
    lines.append("obstacle")
    lines += [" ".join(str(int(v)) for v in row) for row in t.obstacle]
    lines.append("end")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def read_map(path) -> tuple[Terrain, list[dict]]:
    """Read a map file back: the Terrain and its preset trips ({"name", "start", "goal"})."""
    rows = cols = 0
    meta, ponds, presets, grids = {}, [], [], {}
    fields = dict(cell="cell_size", max_grade="max_grade", uphill="uphill_penalty")
    kw: dict = {}
    lines = iter(Path(path).read_text(encoding="utf-8").splitlines())
    if next(lines, "").split() != ["robotnav-map", "1"]:
        raise ValueError(f"{path} is not a robotnav map file (version 1)")
    for line in lines:
        key, _, rest = line.partition(" ")
        if key == "theme":
            meta["theme"] = rest
        elif key == "seed":
            meta["seed"] = int(rest)
        elif key == "rows":
            rows = int(rest)
        elif key == "cols":
            cols = int(rest)
        elif key in fields:
            kw[fields[key]] = float(rest)
        elif key == "surface_costs":
            kw["surface_costs"] = tuple(float(v) for v in rest.split())
        elif key == "surface_names":
            kw["surface_names"] = tuple(rest.split("|"))
        elif key == "sea":
            kw["sea_level"] = None if rest == "none" else float(rest)
        elif key in ("start", "goal"):
            kw[key] = tuple(int(v) for v in rest.split())
        elif key == "pond":
            v = rest.split()
            n = int(v[1])
            ponds.append(dict(level=float(v[0]), cells=[(int(v[2 + 2 * k]), int(v[3 + 2 * k])) for k in range(n)]))
        elif key == "preset":
            v = rest.split(" ", 4)
            presets.append(dict(name=v[4] if len(v) > 4 else "", start=(int(v[0]), int(v[1])), goal=(int(v[2]), int(v[3]))))
        elif key in ("height", "surface", "obstacle"):
            conv = float if key == "height" else int
            grids[key] = np.array([[conv(x) for x in next(lines).split()] for _ in range(rows)],
                                  dtype=float if key == "height" else np.int8)
        elif key == "end":
            break
    if set(grids) != {"height", "surface", "obstacle"} or grids["height"].shape != (rows, cols):
        raise ValueError(f"{path} is incomplete")
    meta.update(size=rows, max_grade=kw.get("max_grade", 0.6))
    t = Terrain(blocked=grids["obstacle"] != NONE, diagonal=True, height=grids["height"], surface=grids["surface"],
                obstacle=grids["obstacle"], ponds=ponds, meta=meta, **kw)
    return t, presets
