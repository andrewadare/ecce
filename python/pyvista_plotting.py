import numpy as np
import pyvista as pv
from matplotlib.colors import ListedColormap
from spatial_effects import SE3


rgb_colormap = ListedColormap(["red", "lawngreen", "dodgerblue"])


def frame_axes(T: np.ndarray, size: float = 1.0) -> pv.PolyData:
    """Return basis vectors of a coordinate frame with the provided 4x4 transform matrix T.

    To display the returned mesh with RGB coloring for XYZ, do something like

    plotter.add_mesh(
        mesh, show_edges=True, line_width=10, scalars=range(3), cmap=rgb_colormap, show_scalar_bar=False,
    )
    """
    vertices = size * np.eye(4, 3, -1)
    edges = [2, 0, 1, 2, 0, 2, 2, 0, 3]
    mesh = pv.PolyData(vertices, lines=edges)
    mesh.transform(T)
    return mesh


def plot_sfm_scene(
    cameras: list[SE3],
    features: np.ndarray | list[SE3],
    html_name="graph.html",
    edges: list[tuple[int, int]] = [],  # (camera, tag) pairs
):
    """Draw a BA/SFM scene using pyvista. `features` can either be
    3D points in a (N, 3) array or poses in a list of SE(3) objects.
    """

    plotter = pv.Plotter()
    plotter.set_background("midnightblue")

    for pose in cameras:
        plotter.add_mesh(
            frame_axes(pose.matrix, size=0.5),
            show_edges=True,
            line_width=10,
            scalars=range(3),
            cmap=rgb_colormap,
            show_scalar_bar=False,
        )

    if isinstance(features, np.ndarray):
        plotter.add_points(
            features,
            point_size=10,
            render_points_as_spheres=True,  # no effect in exported HTML?
            pickable=True,
        )
    else:
        raise NotImplementedError("TODO plot features as SE3 poses")

    for i, j in edges:
        a = cameras[i].t
        b = features[j]
        mesh = pv.PolyData([a, b], lines=[2, 0, 1])
        plotter.add_mesh(
            mesh,
            show_edges=True,
            line_width=2,
            # scalars=range(3),
            # cmap=rgb_colormap,
            # show_scalar_bar=False,
        )

    plotter.show_axes()
    # plotter.show()
    plotter.export_html(html_name)
