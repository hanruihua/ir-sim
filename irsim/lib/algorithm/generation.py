"""
This file is the implementation of the generation of random polygons.

Author: Ruihua Han
"""

import math
from typing import Any

import numpy as np

from irsim.util.random import rng


def clip(value: float, lower: float, upper: float) -> float:
    """
    Clip a value to a specified range.

    Args:
        value (float): Value to be clipped.
        lower (float): Lower bound of the range.
        upper (float): Upper bound of the range.

    Returns:
        float: Clipped value.
    """
    return min(upper, max(value, lower))


def random_generate_polygon(
    number: int = 1,
    center_range: list[float] | None = None,
    avg_radius_range: list[float] | None = None,
    irregularity_range: list[float] | None = None,
    spikeyness_range: list[float] | None = None,
    num_vertices_range: list[int] | None = None,
    **kwargs: Any,
) -> np.ndarray | list[np.ndarray]:
    """
    reference: https://stackoverflow.com/questions/8997099/algorithm-to-generate-random-2d-polygon

    Generate random polygons with specified properties.

    Args:
        number (int): Number of polygons to generate (default 1).
        center_range (List[float]): Range for the polygon center [min_x, min_y, max_x, max_y].
        avg_radius_range (List[float]): Range for the average radius of the polygons.
        irregularity_range (List[float]): Range for the irregularity of the polygons.
        spikeyness_range (List[float]): Range for the spikeyness of the polygons.
        num_vertices_range (List[int]): Range for the number of vertices of the polygons.

    Returns:
        List of vertices for each polygon or a single polygon's vertices if number=1.
    """
    if center_range is None:
        center_range = [0, 0, 0, 0]
    if avg_radius_range is None:
        avg_radius_range = [0.1, 1]
    if irregularity_range is None:
        irregularity_range = [0, 1]
    if spikeyness_range is None:
        spikeyness_range = [0, 1]
    if num_vertices_range is None:
        num_vertices_range = [4, 10]

    center = rng.uniform(low=center_range[0:2], high=center_range[2:], size=(number, 2))
    avg_radius = rng.uniform(
        low=avg_radius_range[0], high=avg_radius_range[1], size=(number,)
    )
    irregularity = rng.uniform(
        low=irregularity_range[0], high=irregularity_range[1], size=(number,)
    )
    spikeyness = rng.uniform(
        low=spikeyness_range[0], high=spikeyness_range[1], size=(number,)
    )
    num_vertices = rng.integers(
        low=num_vertices_range[0], high=num_vertices_range[1], size=(number,)
    )

    vertices_list = [
        generate_polygon(
            center[i, :], avg_radius[i], irregularity[i], spikeyness[i], num_vertices[i]
        )
        for i in range(number)
    ]

    if number == 1:
        return vertices_list[0]

    return vertices_list


def generate_polygon(
    center: list[float],
    avg_radius: float,
    irregularity: float,
    spikeyness: float,
    num_vertices: int,
) -> np.ndarray:
    """
    Generate a random polygon around a center point.

    Args:
        center (Tuple[float, float]): Center of the polygon.
        avg_radius (float): Average radius from the center to vertices.
        irregularity (float): Variance of angle spacing between vertices. Range [0, 1]
        spikeyness (float): Variance of radius from the center. Range [0, 1]
        num_vertices (int): Number of vertices for the polygon.

    Returns:
        numpy.ndarray: Vertices of the polygon in CCW order.
    """
    if irregularity < 0 or irregularity > 1:
        raise ValueError("Irregularity must be between 0 and 1.")
    if spikeyness < 0 or spikeyness > 1:
        raise ValueError("Spikeyness must be between 0 and 1.")

    irregularity *= 2 * math.pi / num_vertices
    spikeyness *= avg_radius
    angle_steps = random_angle_steps(num_vertices, irregularity)

    points = []
    angle = rng.uniform(0, 2 * math.pi)
    for i in range(num_vertices):
        radius = clip(rng.normal(avg_radius, spikeyness), 0, 2 * avg_radius)
        point = (
            center[0] + radius * math.cos(angle),
            center[1] + radius * math.sin(angle),
        )
        points.append(point)
        angle += angle_steps[i]

    return np.array(points)


def random_angle_steps(steps: int, irregularity: float) -> list[float]:
    """
    Generate random angle steps for polygon vertices.

    Args:
        steps (int): Number of angles to generate.
        irregularity (float): Variance of angle spacing.

    Returns:
        List[float]: Random angles in radians.
    """
    angles = []
    lower = (2 * math.pi / steps) - irregularity
    upper = (2 * math.pi / steps) + irregularity
    cumsum = 0
    for _ in range(steps):
        angle = rng.uniform(lower, upper)
        angles.append(angle)
        cumsum += angle

    cumsum /= 2 * math.pi
    for i in range(steps):
        angles[i] /= cumsum
    return angles
