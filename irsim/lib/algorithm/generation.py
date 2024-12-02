import math
import random
from typing import List, Tuple
from PIL import Image, ImageDraw
import numpy as np


def random_generate_polygon(
    number=1,
    center_range=[0, 0, 10, 10],
    avg_radius_range=[0.1, 1],
    irregularity_range=[0, 1],
    spikeyness_range=[0, 1],
    num_vertices_range=[4, 10],
    **kwargs,
):
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
    center = np.random.uniform(
        low=center_range[0:2], high=center_range[2:], size=(number, 2)
    )
    avg_radius = np.random.uniform(
        low=avg_radius_range[0], high=avg_radius_range[1], size=(number,)
    )
    irregularity = np.random.uniform(
        low=irregularity_range[0], high=irregularity_range[1], size=(number,)
    )
    spikeyness = np.random.uniform(
        low=spikeyness_range[0], high=spikeyness_range[1], size=(number,)
    )
    num_vertices = np.random.randint(
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


def generate_polygon(center, avg_radius, irregularity, spikeyness, num_vertices):
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
    angle = random.uniform(0, 2 * math.pi)
    for i in range(num_vertices):
        radius = clip(random.gauss(avg_radius, spikeyness), 0, 2 * avg_radius)
        point = (
            center[0] + radius * math.cos(angle),
            center[1] + radius * math.sin(angle),
        )
        points.append(point)
        angle += angle_steps[i]

    vertices = np.array(points)

    return vertices


def random_angle_steps(steps: int, irregularity: float) -> List[float]:
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
    for i in range(steps):
        angle = random.uniform(lower, upper)
        angles.append(angle)
        cumsum += angle

    cumsum /= 2 * math.pi
    for i in range(steps):
        angles[i] /= cumsum
    return angles


def clip(value, lower, upper):
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
