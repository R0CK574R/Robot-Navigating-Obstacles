"""Export the three themed maps to one self-contained 3-D viewer (HTML + Three.js).

Python generates the maps, picks each map's 20 optional preset routes, and is the
reference implementation of every planner. When you drive a route, the browser runs
the same five planners plus the hill-blind shortest route (ported line for line), and
the tests check both give identical routes and times.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .terrain import SHORTEST_TIE, Terrain, preset_routes

TEMPLATE = Path(__file__).with_name("viewer3d.html")
NAMES = {"grassland": "Grassland", "desert": "Desert", "island": "Island"}


def map_scene(t: Terrain) -> dict:
    theme = t.meta["theme"]
    return dict(
        key=theme, name=NAMES[theme], seed=t.meta["seed"], meta=t.meta,
        rows=t.rows, cols=t.cols, max_grade=t.max_grade, sea=t.sea_level,
        relief=round(float(t.height.max() - t.height.min()), 2),
        heights=[float(v) for v in t.height.ravel()],        # exact, so the browser plans like Python
        surface=[int(v) for v in t.surface.ravel()],
        obstacle=[int(v) for v in t.obstacle.ravel()],       # 0 open, 1 rock, 2 tree/cactus/palm, 3 water
        surfaces=[dict(name=n, cost=float(c)) for n, c in zip(t.surface_names, t.surface_costs)],
        start=[int(v) for v in t.start], goal=[int(v) for v in t.goal],
        presets=[dict(name=p["name"], start=list(p["start"]), goal=list(p["goal"]), time=float(p["time"]),
                      distance_m=float(p["distance_m"]), short_time=float(p["short_time"]),
                      saving=float(p["saving"]), path=[int(v) for cell in p["path"] for v in cell])
                 for p in preset_routes(t)],                 # the 20 optional paths, quickest route each
    )


def write_viewer(maps: list[Terrain], out: str | Path, scenes: list[dict] | None = None) -> Path:
    """Write the viewer. Pass `scenes` (from map_scene) if you already built them; picking presets takes a few seconds."""
    scene = dict(cost=dict(uphill=maps[0].uphill_penalty, cell=maps[0].cell_size, short_tie=SHORTEST_TIE),
                 maps=scenes if scenes is not None else [map_scene(t) for t in maps])
    html = TEMPLATE.read_text().replace("/*__SCENE__*/null", json.dumps(scene, separators=(",", ":")))
    out = Path(out)
    out.write_text(html)
    return out
