"""The command line takes start and destination cells the same way the viewer's position boxes do."""
import sys
from argparse import Namespace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import main  # noqa: E402
from robotnav.terrain import solvable_map  # noqa: E402


@pytest.mark.parametrize("text, cell", [("80,10", (80, 10)), ("80, 10", (80, 10)), (" 80 10 ", (80, 10)),
                                        ("(80, 10)", (80, 10)), ("[80,10]", (80, 10)), ("80;10", (80, 10))])
def test_cell_accepts_the_viewer_forms(text, cell):
    assert main._cell(text) == cell


@pytest.mark.parametrize("text", ["80", "80,10,5", "a,b", "80.5,10", "", "80 - 10", "row 80"])
def test_cell_rejects_anything_else_with_a_message(text):
    with pytest.raises(SystemExit, match="row,col"):
        main._cell(text)


@pytest.fixture(scope="module")
def small():
    return solvable_map("grassland", seed=3, size=48)


def _args(**kw):
    return Namespace(**{**dict(map="grassland", start=None, goal=None, preset=None, size=48, grade=0.6, seed=3), **kw})


def _at(cell):
    return f"{cell[0]},{cell[1]}"


def test_route_rejects_the_same_cell_twice(small):
    with pytest.raises(SystemExit, match="same cell"):
        main.route(_args(start=_at(small.start), goal=_at(small.start)))


def test_route_rejects_cells_off_the_map_or_blocked(small):
    with pytest.raises(SystemExit, match="outside"):
        main.route(_args(start="48,0", goal=_at(small.goal)))
    blocked = tuple(int(v) for v in np.argwhere(small.blocked)[0])
    with pytest.raises(SystemExit, match="obstacle or water"):
        main.route(_args(start=_at(small.start), goal=_at(blocked)))


def test_route_rejects_preset_with_cells(small):
    with pytest.raises(SystemExit, match="not both"):
        main.route(_args(preset=1, start=_at(small.start)))


def test_route_races_planners_and_the_shortest_route(small, capsys):
    main.route(_args(start=_at(small.start), goal=_at(small.goal)))
    out = capsys.readouterr().out
    for name in ("A*", "Dijkstra", "Weighted A*", "Greedy", "BFS", "Shortest route"):
        assert name in out
    assert f"start {small.start} -> destination {small.goal}" in out
