"""Coverage test for issue #302.

``lidar2d`` groups the per-scan subtractions by geometry type (linestrings vs
polygons) and differences each group separately (mixing them in one
``GeometryCollection`` made GEOS's ``difference`` 2-3x slower with dynamic
obstacles). This test builds a scan containing a static map linestring and a
dynamic polygon obstacle and checks that both reduce the beam ranges, which
exercises the grouped-difference path.
"""

import numpy as np
import pytest
import shapely
from shapely import STRtree

from irsim.world.sensors.lidar2d import Lidar2D


class _EnvParam:
    def __init__(self, objects, tree):
        self.objects = objects
        self.GeometryTree = tree


class _Env:
    def __init__(self, env_param):
        self._env_param = env_param


class _Parent:
    def __init__(self, env_param):
        self._env = _Env(env_param)


class _MapObject:
    """Mimics a grid-map object: linestrings + an STRtree (``shape == 'map'``)."""

    def __init__(self, obj_id, linestrings):
        self._id = obj_id
        self.linestrings = list(linestrings)
        self._geometry = shapely.MultiLineString(self.linestrings)
        self._geometry_valid = True
        self.shape = "map"
        self.unobstructed = False
        self.geometry_tree = STRtree(self.linestrings)

    @property
    def geometry(self):
        return self._geometry


class _Obstacle:
    """Mimics a dynamic obstacle with a polygon geometry."""

    def __init__(self, obj_id, geometry, shape="circle"):
        self._id = obj_id
        self._geometry = geometry
        self._geometry_valid = True
        self.shape = shape
        self.unobstructed = False

    @property
    def geometry(self):
        return self._geometry


def test_lidar_subtracts_map_lines_and_dynamic_polygon():
    """A static map linestring and a dynamic polygon must both block beams in one
    scan, exercising the grouped-difference path added for issue #302."""
    range_max = 10.0
    # static map: a short vertical wall at x = -3
    map_obj = _MapObject(2, [shapely.LineString([(-3.0, -0.5), (-3.0, 0.5)])])
    # dynamic obstacle: a circle (polygon) centred at x = 2, radius 0.5
    circle = _Obstacle(3, shapely.Point(2.0, 0.0).buffer(0.5))
    env_param = _EnvParam(
        [map_obj, circle], STRtree([map_obj.geometry, circle.geometry])
    )

    lidar = Lidar2D(
        state=np.array([[0.0], [0.0], [0.0]]),
        obj_id=1,
        number=36,
        angle_range=6.283185,
        range_max=range_max,
    )
    lidar.parent = _Parent(env_param)
    lidar.step(lidar.state)

    ranges = np.asarray(lidar.get_scan()["ranges"])
    blocked = ranges[ranges < range_max - 1e-6]

    # both kinds of obstacle were subtracted: the nearest hit is the polygon
    # (circle, near edge at x=1.5) and a farther hit is the line (wall at x=-3).
    assert len(blocked) >= 2
    assert ranges.min() == pytest.approx(1.5, abs=0.3)  # circle -> polygon group
    assert blocked.max() == pytest.approx(3.0, abs=0.3)  # wall   -> line group
