#!/usr/bin/env python

import numpy as np
import pyvista as pv

from spatial_effects import SE3

from opencv_visualization import create_tag
from pyvista_plotting import frame_axes, rgb_colormap


def find_intersection(c: np.ndarray, u: np.ndarray, p0: np.ndarray, n: np.ndarray):
    """Find the intersection point between a ray from point c with direction u
    and a plane with unit normal vector n containing point p0.
    All inputs have shape (3,).

    Let
        c = camera position
        u = unit vector of a ray back-projected from c
        n = unit normal vector of a plane intersecting the ray
        p0 = any known point on the plane
        p_int = ray-plane intersection point to calculate

    Then
        p_int = c + lambda u
    where lambda scales u to contact the plane.

    Also, by definition the plane contains all points p such that
        (p - p0) dot n = 0.

    At the intersection point p = p_int, this is
        (c + lambda u - p0) dot n = 0
    so
        lambda = (p0 - c) dot n / (u dot n).
    """
    denominator = u @ n
    if denominator == 0:
        raise ValueError("No intersection")

    lambda_ = (p0 - c) @ n / denominator

    return c + lambda_ * u


def main():

    # Tag
    tag_size = 0.5
    tag_pose = SE3([0, 0, 1.0], [np.pi / 6, np.pi, -0.1])
    im = create_tag("DICT_APRILTAG_16h5", 0, 300, 50)
    texture = pv.numpy_to_texture(im)
    # By setting the resolution all the way down to 1, plane.points is only the 4 corners.
    tag = pv.Plane(i_size=tag_size, j_size=tag_size, i_resolution=1, j_resolution=1)
    tag.transform(tag_pose.matrix)

    # Camera
    camera_pose = SE3()
    camera = frame_axes(camera_pose.matrix, size=0.1)

    # Rays from corner points to camera
    ray_points = np.vstack([camera_pose.t, tag.points])
    edges = []
    for i, _ in enumerate(ray_points):
        edges.extend([2, 0, i + 1])  # 2 points/line: camera point and corner point
    rays = pv.PolyData(ray_points, lines=edges)

    # Image plane
    img_size = 0.2
    img_transform = np.eye(4)
    img_transform[2, 3] = 0.25  # in z direction
    img = pv.Plane(i_size=img_size, j_size=img_size, i_resolution=1, j_resolution=1)
    img.transform(img_transform)

    # Image points
    # Unit vectors from camera to tag corners
    unit_rays = tag.points - camera_pose.t
    unit_rays /= np.linalg.norm(unit_rays, axis=1)[:, np.newaxis]
    n = np.array([0, 0, -1])  # Image plane normal
    p0 = img_transform[:3, 3]  # Known point on image plane
    points_on_image = []
    for u in unit_rays:
        p = find_intersection(camera_pose.t, u, p0, n)
        points_on_image.append(p)

    plotter = pv.Plotter(notebook=False)

    plotter.add_mesh(
        camera,
        line_width=10,
        scalars=range(3),
        cmap=rgb_colormap,
        show_scalar_bar=False,
    )
    plotter.add_mesh(tag, texture=texture)
    plotter.add_mesh(
        frame_axes(tag_pose.matrix, size=0.1),
        line_width=10,
        scalars=range(3),
        cmap=rgb_colormap,
        show_scalar_bar=False,
    )

    plotter.add_points(
        ray_points,
        point_size=15,
        render_points_as_spheres=True,
        pickable=True,
        color="orange",
    )

    plotter.add_mesh(
        rays,
        show_edges=True,
        line_width=3,
    )

    plotter.add_mesh(
        img,
        opacity=0.5,
        show_edges=True,
    )

    plotter.add_points(
        np.array(points_on_image),
        point_size=15,
        color="red",
        render_points_as_spheres=True,
        pickable=True,
    )

    plotter.set_background("midnightblue")
    plotter.add_light(pv.Light(position=(0, 0, -10), light_type="scene light"))
    plotter.show()


if __name__ == "__main__":
    main()
