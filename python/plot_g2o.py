#!/usr/bin/env python
from pathlib import Path

import numpy as np
from spatial_effects import SE3

from pyvista_plotting import plot_sfm_scene
from g2o_interop import read_sfm_g2o


def plot(
    g2o_in: str | Path, html_out: str | Path, factor_symbols: list[list[str]] = []
):
    """Plot a 3D scene from a g2o file and save to html_out.
    GTSAM lacks support for printing some measurement factors to g2o format.
    The symbols list provides a partial (and optional) workaround for this
    by associating nodes.
    """
    graph = read_sfm_g2o(g2o_in)

    cameras: list[SE3] = list(graph.nodes.values())
    tags: np.ndarray = np.vstack(list(graph.features.values()))

    # Get a list of (camera, tag) index pairs
    edges: list[tuple[int, int]] = []
    for symbols in factor_symbols:
        if len(symbols) == 2:  # Only want binary factors
            c, t = symbols
            edges.append((int(c[1:]), int(t[1:])))

    plot_sfm_scene(cameras, tags, html_name=html_out, edges=edges)


if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)

    # Read in factor graph symbols to visualize edges
    symbols = []
    with open("symbols.txt") as f:
        for line in f.readlines():
            symbols.append(line.strip().split())

    plot("result.g2o", "result.html", factor_symbols=symbols)
