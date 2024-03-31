#!/usr/bin/env python
from pathlib import Path

import numpy as np
from spatial_effects import SE3

from pyvista_plotting import plot_sfm_scene
from g2o_interop import read_sfm_g2o


def plot(
    g2o_in: str | Path,
    html_out: str | Path,
    factor_symbols: list[list[str]] = [],
    tag_dict: dict[int, np.ndarray] = {},
):
    """Plot a 3D scene from a g2o file and save to html_out.
    GTSAM lacks support for printing some measurement factors to g2o format.
    The symbols list provides a partial (and optional) workaround for this
    by associating nodes.
    """
    graph = read_sfm_g2o(g2o_in)

    cameras: list[SE3] = list(graph.nodes.values())

    tag_centers = np.vstack(list(graph.features.values()))

    # Get a list of (camera, tag) index pairs
    edges: list[tuple[int, int]] = []
    for symbols in factor_symbols:
        if len(symbols) == 2:  # Only want binary factors
            c, t = symbols
            edges.append((int(c[1:]), int(t[1:])))

    plot_sfm_scene(
        cameras, tag_centers, html_name=html_out, edges=edges, tag_dict=tag_dict
    )


if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)

    # Read in factor graph symbols to visualize edges
    symbols = []
    with open("symbols.txt") as f:
        for line in f.readlines():
            symbols.append(line.strip().split())

    tags: dict[int, np.ndarray] = dict()  # id => SE(3) matrix
    with open("tag-poses.txt") as f:
        for line in f.readlines():
            strings = line.split()
            id = int(strings[0])
            matrix = np.array([float(x) for x in strings[1:]]).reshape([4, 4])
            tags[id] = matrix

    # Version showing april tags
    # plot("result.g2o", "result.html", factor_symbols=symbols)

    # Version overlaying graph edges
    plot("result.g2o", "edges.html", factor_symbols=symbols, tag_dict=tags)
