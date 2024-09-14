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
    2d range: min_x, min_y, max_x, max_y
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

    # reference: https://stackoverflow.com/questions/8997099/algorithm-to-generate-random-2d-polygon
    """
    Start with the center of the polygon at center, then creates the
    polygon by sampling points on a circle around the center.
    Random noise is added by varying the angular spacing between
    sequential points, and by varying the radial distance of each
    point from the centre.

    Args:
        center (Tuple[float, float]):
            a pair representing the center of the circumference used
            to generate the polygon.
        avg_radius (float):
            the average radius (distance of each generated vertex to
            the center of the circumference) used to generate points
            with a normal distribution.
        irregularity (float): 0 - 1
            variance of the spacing of the angles between consecutive
            vertices.
        spikeyness (float): 0 - 1
            variance of the distance of each vertex to the center of
            the circumference.
        num_vertices (int):
            the number of vertices of the polygon.
    Returns:
        (N, 2) numpy Matrix, in CCW order.
    """

    # Parameter check
    if irregularity < 0 or irregularity > 1:
        raise ValueError("Irregularity must be between 0 and 1.")
    if spikeyness < 0 or spikeyness > 1:
        raise ValueError("spikeyness must be between 0 and 1.")

    irregularity *= 2 * math.pi / num_vertices
    spikeyness *= avg_radius
    angle_steps = random_angle_steps(num_vertices, irregularity)

    # now generate the points
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
    """Generates the division of a circumference in random angles.

    Args:
        steps (int):
            the number of angles to generate.
        irregularity (float):
            variance of the spacing of the angles between consecutive vertices.
    Returns:
        List[float]: the list of the random angles.
    """
    # generate n angle steps
    angles = []
    lower = (2 * math.pi / steps) - irregularity
    upper = (2 * math.pi / steps) + irregularity
    cumsum = 0
    for i in range(steps):
        angle = random.uniform(lower, upper)
        angles.append(angle)
        cumsum += angle

    # normalize the steps so that point 0 and point n+1 are the same
    cumsum /= 2 * math.pi
    for i in range(steps):
        angles[i] /= cumsum
    return angles


def clip(value, lower, upper):
    """
    Given an interval, values outside the interval are clipped to the interval
    edges.
    """
    return min(upper, max(value, lower))
