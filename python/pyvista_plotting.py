import numpy as np
import pyvista as pv
from matplotlib.colors import ListedColormap
from spatial_effects import SE3

from opencv_visualization import create_tag

rgb_colormap = ListedColormap(["red", "lawngreen", "dodgerblue"])


def frame_axes(transform: np.ndarray, size: float = 1.0) -> pv.PolyData:
    """Return basis vectors of a coordinate frame with the provided 4x4 transform matrix.

    To display the returned mesh with RGB coloring for XYZ, do something like

    plotter.add_mesh(
        mesh, show_edges=True, line_width=10, scalars=range(3), cmap=rgb_colormap, show_scalar_bar=False,
    )
    """
    vertices = size * np.eye(4, 3, -1)
    edges = [2, 0, 1, 2, 0, 2, 2, 0, 3]
    mesh = pv.PolyData(vertices, lines=edges)
    mesh.transform(transform)
    return mesh


def add_tag(im: np.ndarray, transform: np.ndarray, plotter: pv.Plotter):
    tag_size = 0.5
    texture = pv.numpy_to_texture(im)
    plane = pv.Plane(i_size=tag_size, j_size=tag_size)
    plane.transform(transform)
    plotter.add_mesh(plane, texture=texture)

    # TODO figure out texture mapping to front face only.
    # For now, cover the back with another plane skooched back a hair
    offset = np.eye(4)
    offset[2, 3] = -0.001
    back = pv.Plane(i_size=tag_size, j_size=tag_size)
    back.transform(transform @ offset)
    plotter.add_mesh(back, color="white")


def plot_sfm_scene(
    cameras: list[SE3],
    features: np.ndarray,
    html_name="graph.html",
    tag_dict: dict = {},
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

    for id, transform in tag_dict.items():
        im = create_tag("DICT_APRILTAG_16h5", id, 300, 50)
        add_tag(im, transform, plotter)

    for i, j in edges:
        a = cameras[i].t
        b = features[j]
        mesh = pv.PolyData([a, b], lines=[2, 0, 1])
        plotter.add_mesh(
            mesh,
            show_edges=True,
            line_width=2,
        )

    # Extra illumination on the left and right sides of the scene
    plotter.add_light(pv.Light(position=(-10, 0, -10), light_type="scene light"))
    plotter.add_light(pv.Light(position=(10, 0, -10), light_type="scene light"))

    # For interactive display
    # plotter.show_axes()
    # plotter.show()

    plotter.export_html(html_name)
