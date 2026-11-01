"""Export the three themed maps to one self-contained 3-D viewer (HTML + Three.js).

Python generates the maps, picks each map's 20 optional preset routes, and is the
reference implementation of every planner. When you drive a route, the browser runs
the same five planners plus the hill-blind shortest route (ported line for line), and
the tests check both give identical routes and times.
"""
from __future__ import annotations

import json
from html import escape
from pathlib import Path

import numpy as np

from .airspace import AIR, place_towers
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
        ponds=[dict(level=float(p["level"]), cells=[int(r * t.cols + c) for r, c in p["cells"]])
               for p in t.ponds],                            # desert oases: water level and cells (flat index)
        surfaces=[dict(name=n, cost=float(c)) for n, c in zip(t.surface_names, t.surface_costs)],
        start=[int(v) for v in t.start], goal=[int(v) for v in t.goal],
        presets=[dict(name=p["name"], start=list(p["start"]), goal=list(p["goal"]), time=float(p["time"]),
                      distance_m=float(p["distance_m"]), short_time=float(p["short_time"]),
                      saving=float(p["saving"]), path=[int(v) for cell in p["path"] for v in cell])
                 for p in preset_routes(t)],                 # the 20 optional paths, quickest route each
        towers=[list(cell) for cell in place_towers(t)],     # the air route's watchtowers
    )


TITLE = "Rover Hill Course"
DESCRIPTION = ("A rover finds the quickest path over hilly terrain: five pathfinding algorithms raced in 3D, "
               "the search replayed side by side, D* Lite re-planning with a limited sensor, and a drone that flies "
               "an air route out of enemy sight. Runs in the browser.")


def write_viewer(maps: list[Terrain], out: str | Path, scenes: list[dict] | None = None,
                 site_url: str | None = None) -> Path:
    """Write the viewer as a standalone HTML page.

    Pass `scenes` (from map_scene) if you already built them; picking presets takes a few seconds.
    Pass `site_url` (where the page will be hosted, ending in "/") to add link-preview tags, so a shared
    link shows the title, a description and social.png from that address.
    """
    scene = dict(cost=dict(uphill=maps[0].uphill_penalty, cell=maps[0].cell_size, short_tie=SHORTEST_TIE), air=AIR,
                 maps=scenes if scenes is not None else [map_scene(t) for t in maps])
    body = TEMPLATE.read_text().replace("/*__SCENE__*/null", json.dumps(scene, separators=(",", ":")))
    head = ["<!doctype html>", '<html lang="en">', '<meta charset="utf-8">',
            f'<meta name="description" content="{escape(DESCRIPTION)}">']
    if site_url:
        image = site_url + "social.png"
        head += [f'<link rel="canonical" href="{escape(site_url)}">',
                 '<meta property="og:type" content="website">',
                 f'<meta property="og:title" content="{escape(TITLE)}">',
                 f'<meta property="og:description" content="{escape(DESCRIPTION)}">',
                 f'<meta property="og:url" content="{escape(site_url)}">',
                 f'<meta property="og:image" content="{escape(image)}">',
                 '<meta property="og:image:width" content="1200">', '<meta property="og:image:height" content="630">',
                 '<meta name="twitter:card" content="summary_large_image">',
                 f'<meta name="twitter:title" content="{escape(TITLE)}">',
                 f'<meta name="twitter:description" content="{escape(DESCRIPTION)}">',
                 f'<meta name="twitter:image" content="{escape(image)}">']
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(head) + "\n" + body)
    return out
