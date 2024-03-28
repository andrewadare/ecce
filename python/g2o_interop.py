"""Utilities for conversion between g2o strings and numpy arrays or spatial_effects.SE3 objects.
This is by no means complete or general. More g2o types can be supported as it becomes necessary.
"""

from dataclasses import dataclass
from pathlib import Path

from enum import Enum, auto

import numpy as np
from spatial_effects import SE3


class G2OType(Enum):
    VERTEX_SE3_QUAT = auto()
    VERTEX_TRACKXYZ = auto()
    EDGE_SE3_QUAT = auto()


@dataclass
class EdgeSE3:
    i: int
    j: int
    transform: SE3
    precision: np.ndarray  # (6, 6)


@dataclass
class GraphSE3:
    nodes: dict[int, SE3]
    features: dict[int, np.ndarray | SE3]  # features can be points or poses
    edges: dict[tuple[int, int], EdgeSE3]


def read_sfm_g2o(filename: str | Path) -> GraphSE3:

    nodes: dict[int, SE3] = dict()
    features: dict[int, np.ndarray | SE3] = dict()
    edges: dict[tuple[int, int], EdgeSE3] = dict()

    with open(filename) as f:
        for line in f.readlines():
            token = line.split(" ")[0]
            if token == "VERTEX_SE3:QUAT":
                id, transform = read_g2o_se3_quat(line)
                nodes[id] = transform
            elif token == "VERTEX_TRACKXYZ":
                id, point = read_xyz(line)
                features[id] = point
            else:
                raise NotImplementedError(f"No parser for {token}")

    return GraphSE3(nodes, features, edges)


def read_g2o_se3_quat(line: str) -> tuple[int, SE3]:
    token = "VERTEX_SE3:QUAT"
    if not line.startswith(token):
        raise ValueError(f"Line must begin with {token}: {line}")

    a = np.fromstring(line.split(token)[1], sep=" ")
    id = int(a[0])
    t = a[1:4]
    q = np.roll(a[4:8], 1)  # xyzw -> wxyz, no thanks to Eigen
    return (id, SE3(t, q))


def read_xyz(line: str) -> tuple[int, np.ndarray]:
    """Read a VERTEX_TRACKXYZ line and return (id, np.array([x,y,z]))."""
    token = "VERTEX_TRACKXYZ"
    if not line.startswith(token):
        raise ValueError(f"Line must begin with {token}: {line}")

    a = np.fromstring(line.split(token)[1], sep=" ")
    id = int(a[0])
    t = a[1:4]
    return (id, t)


def lambda_matrix(variances: np.ndarray):
    """Returns a diagonal precision (inverse covariance) matrix.

    variances is a [position, rotation] uncertainty 6-vector (sigma**2)
    """
    return np.diag(1.0 / variances)


def create_g2o_se3_vertex(pose: SE3, i: int, precision=6) -> str:
    """Returns a VERTEX_SE3:QUAT ... g2o string from an SE3 object."""
    return (
        f"VERTEX_SE3:QUAT {i}"
        f" {np.array2string(pose.t, precision=precision)[1:-1]}"
        f" {np.array2string(np.roll(pose.q, -1), precision=precision)[1:-1]}"
        "\n"
    )


def create_g2o_se3_edge(Z: SE3, i: int, j: int, P: np.ndarray, precision=6) -> str:
    """Measurement or estimate in SE(3) for transform from node i -> j.
    Precision matrix P = inv(cov) should be should be (6,6)."""

    # Upper triangle of precision matrix flattened row-wise
    u = P[np.triu_indices_from(P)]

    return (
        f"EDGE_SE3:QUAT {i} {j}"
        f" {np.array2string(Z.t, precision=precision)[1:-1]}"
        f" {np.array2string(np.roll(Z.q, -1), precision=precision)[1:-1]}"
        f" {np.array2string(u, precision=precision)[1:-1]}"
        "\n"
    )
