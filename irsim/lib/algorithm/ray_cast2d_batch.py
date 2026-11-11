"""Batched 2D LiDAR ray casting: all plain ``Lidar2D`` sensors of a world at once.

:class:`RayCast2DBatch` casts every beam of every lidar with array
operations, and gives exactly the scans a sensor casting alone would, bit for
bit. Each
:meth:`Lidar2D.step <irsim.world.sensors.lidar2d.Lidar2D.step>` asks for its
scan (:func:`scan`). The first lidar to ask after the environment rebuilds its
geometry tree, which it does whenever objects move, triggers one cast of all
lidars of the environment; the others read their scans from that cast. The
cast has four stages:

1. **Rays.** The beams of each sensor become an origin, unit directions and a
   range (:class:`Rays`). Sensors with the same beam count are cast together.
2. **Scene.** Each body lidars can see becomes boundary segments
   (:class:`Segments`), except ``circle`` bodies, which stay circle polygons
   (:class:`CircleBodies`). Segments and polygon vertices are read from each
   body's world geometry; those of static bodies are cached. Circle source
   vertices are checked against that geometry; a replacement waiting to be
   placed uses the current world's boundary segments instead.
3. **Cast.** A sensor only meets the segments and circles within its range. For
   each such (sensor, primitive) pair, all beams of the sensor are intersected
   at once, and the nearest hit of each beam is kept (:class:`Hits`). A beam is
   first tested against the circle around a circle polygon: entering from
   outside, it can only cross the three polygon edges around the entry point;
   starting inside, it may cross any edge. With numba installed
   (``pip install ir-sim[fast]``), the compiled :func:`cast_kernel` at the end
   of this module runs this stage in plain loops instead of the NumPy kernels.
   NumPy generates and consumes candidate pairs in bounded chunks, so large
   maps do not require the full sensor-by-segment matrix in memory.
4. **Scans.** Each sensor's share of the hits becomes its scan
   (:class:`_Scan`). Sensors still apply their own range noise in their own
   step, so seeded noise is drawn in object order, as without the batch.

A scan that rounding could decide is *ambiguous*: a beam grazing a circle or
running along a segment, a hit at a segment end, the range limit or the sensor
origin, or two targets at the same distance. The batch hands out no ambiguous
scan, so those sensors cast alone, as do sensors with angular noise or a
requested pose different from the cached pose. Scans are reused until the
environment rebuilds its geometry tree, which it does in every step. A sensor
stepped after an object was moved by hand (``EnvParam.geometry_changed``)
casts alone and sees the move; the batch takes the tree rebuilt for it as its
own and casts every lidar on it at the next sensor step that finds nothing
moved in between. A loop of manual moves and scans so costs one scan per move
rather than a cast of every lidar, and many moves followed by many scans cast
every lidar once.

Bit for bit: every other scan repeats the arithmetic of the per-sensor cast
(:func:`~irsim.lib.algorithm.ray_casting_2d.cast_ray_segments`) exactly. The
beams are moved as :func:`~irsim.util.util.geometry_transform` moves them, the
edges come from the same world geometry with their ends in the same order, and
each crossing uses the same elementwise products in the same order, without
fused or BLAS arithmetic, which rounds differently.

Why three edges are enough for a circle body: its buffer polygon is a regular
polygon of ``M`` vertices inscribed in the circle of radius ``R``, and contains
the inner circle of radius ``R cos(pi / M)``. A beam entering the circle meets
the polygon after turning less than one edge (``2 pi / M``) around the centre.
If the beam passes outside the inner circle, its whole chord through the circle
spans at most ``2 pi / M``; otherwise it is inside the polygon once it reaches
the inner circle, which takes at most ``pi / M``. So the edge it crosses is the
one at its entry angle or a neighbour. A crossing exactly at a vertex is
ambiguous, so the sensor casts alone.

References:

* R. Goldman, "Intersection of two lines in three-space", in A. Glassner
  (ed.), *Graphics Gems*, Academic Press, 1990. The beam/segment crossing of
  :func:`_crossings` (parameters ``t`` and ``u`` from 2D cross products).
* C. Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2004. The
  beam/circle test of :func:`_circle_crossings`, the bounding-box culling of
  :func:`_segments_in_range`, and testing a bounding circle before the exact
  shape.
* J. R. Shewchuk, "Adaptive precision floating-point arithmetic and fast
  robust geometric predicates", *Discrete & Computational Geometry* 18(3),
  305-363, 1997. The idea behind ambiguous scans: a fast floating-point test
  with an error tolerance, and a slower exact path (here the sensor's own
  cast) when rounding could decide the result.
* IR-SIM's per-sensor caster, :func:`irsim.lib.algorithm.ray_casting_2d.cast_rays`,
  is the reference the batch is tested against, together with independent
  GEOS intersections through Shapely.
"""

from __future__ import annotations

import math
import warnings
from typing import NamedTuple

import numpy as np
import shapely

from irsim.lib.algorithm.ray_casting_2d import ORIGIN_EPS, boundary_segments

try:
    from numba import njit
except ImportError:  # pragma: no cover - optional dependency
    njit = None

# Whether the compiled cast_kernel (end of this module) can be built.
NUMBA_AVAILABLE = njit is not None

# Largest candidate (sensor x primitive) or intersection (pair x beam) block,
# in array elements (intersection work always includes at least one pair).
MAX_BLOCK_ELEMENTS = 1 << 22
# Relative tolerance of "rounding could decide this". It only sends scans to
# the sensor's own step; it never widens what counts as a hit.
ROUND_OFF = 1e-12
# A polygon is a circle polygon when its vertices lie on one circle at even
# angles, to this relative tolerance.
_CIRCLE_RTOL = 1e-9
# The three edges around an entry point, as offsets from the edge that starts
# at the last vertex at or below the entry angle.
_ENTRY_EDGES = np.array([-1, 0, 1])


# ---------------------------------------------------------------- data types
class Rays(NamedTuple):
    """The beams of ``S`` sensors with ``N`` beams each.

    The field order is the argument order of the compiled kernel.
    """

    origin: np.ndarray  # (S, 2) sensor position
    direction: np.ndarray  # (S, N, 2) unit beam directions
    max_range: np.ndarray  # (S,) range of each sensor
    owner: np.ndarray  # (S,) object index of the body carrying the sensor


class Segments(NamedTuple):
    """``E`` boundary segments, each tagged with the object it belongs to.

    The field order is the argument order of the compiled kernel.
    """

    start: np.ndarray  # (E, 2)
    end: np.ndarray  # (E, 2)
    box: np.ndarray  # (E, 4) bounding box [min_x, max_x, min_y, max_y]
    owner: np.ndarray  # (E,) object index

    @classmethod
    def of(cls, start: np.ndarray, end: np.ndarray, owner: int) -> Segments:
        """The segments ``start -> end`` of the object with index ``owner``."""
        box = np.column_stack(
            (
                np.minimum(start[:, 0], end[:, 0]),
                np.maximum(start[:, 0], end[:, 0]),
                np.minimum(start[:, 1], end[:, 1]),
                np.maximum(start[:, 1], end[:, 1]),
            )
        )
        return cls(start, end, box, np.full(len(start), owner, dtype=int))

    @classmethod
    def concat(cls, parts) -> Segments:
        """All segments of ``parts`` as one set; a lone set is returned as is."""
        parts = [part for part in parts if len(part.start)] or [_NO_SEGMENTS]
        if len(parts) == 1:
            return parts[0]
        return cls(*(np.concatenate(column) for column in zip(*parts, strict=True)))


class CirclePolygon(NamedTuple):
    """The body-frame buffer polygon of a ``circle`` body: a regular polygon
    of ``M`` vertices inscribed in its circle.

    Edge ``k`` joins the ``k``-th and the next vertex by angle; ``edge_start``
    and ``edge_end`` give its ends as indices into the polygon's ring, in the
    ring's direction, so it is cast exactly as the per-sensor scan casts it.
    """

    angles: np.ndarray  # (M,) vertex angles around the centre, ascending
    edge_start: np.ndarray  # (M,) ring index where edge k starts
    edge_end: np.ndarray  # (M,) ring index where edge k ends
    center: np.ndarray  # (2,)
    radius: float
    vertices: np.ndarray  # (M, 2) source vertices, in ring order


class CircleBodies(NamedTuple):
    """``C`` circle bodies, cast as their circle polygons of ``M`` vertices each."""

    objects: list  # the bodies; their poses and geometries are read every step
    owner: np.ndarray  # (C,) object index
    center: np.ndarray  # (C, 2) polygon centre in the body frame
    radius: np.ndarray  # (C,) circle radius
    angles: np.ndarray  # (C, M) vertex angles, ascending
    edge_start: np.ndarray  # (C, M) ring index where edge k starts
    edge_end: np.ndarray  # (C, M) ring index where edge k ends
    shared_angles: bool  # every polygon has the same vertex angles

    @classmethod
    def gather(cls, items) -> CircleBodies:
        """The bodies of ``items`` = ``[(index, object, CirclePolygon)]``,
        whose polygons all have the same number of vertices."""
        if not items:
            return _NO_CIRCLES
        owners, objects, polygons = zip(*items, strict=True)
        angles = np.stack([p.angles for p in polygons])
        return cls(
            objects=list(objects),
            owner=np.array(owners, dtype=int),
            center=np.array([p.center for p in polygons], dtype=float),
            radius=np.array([p.radius for p in polygons], dtype=float),
            angles=angles,
            edge_start=np.stack([p.edge_start for p in polygons]),
            edge_end=np.stack([p.edge_end for p in polygons]),
            shared_angles=bool(np.all(angles == angles[0])),
        )

    def poses(self) -> np.ndarray:
        """``(C, 3)`` current poses ``[x, y, theta]`` of the bodies."""
        return np.array([_pose(obj) for obj in self.objects]).reshape(-1, 3)

    def centers(self, poses: np.ndarray) -> np.ndarray:
        """``(C, 2)`` world positions of the polygon centres."""
        return _to_world(self.center[:, None], poses)[:, 0]

    def ring_vertices(self) -> np.ndarray:
        """``(C, M, 2)`` world vertices in ring order, read from each body's
        geometry: the coordinates the per-sensor scan casts against."""
        m = self.angles.shape[1]
        coords = shapely.get_coordinates([obj._geometry for obj in self.objects])
        return np.ascontiguousarray(coords.reshape(-1, m + 1, 2)[:, :m])

    def kernel_arrays(self) -> tuple:
        """The bodies as the compiled kernel takes them: positions, headings,
        centres, radii, owners, world ring vertices, vertex angles and the
        ring ends of every edge."""
        poses = self.poses()
        return (
            np.ascontiguousarray(poses[:, :2]),
            np.ascontiguousarray(poses[:, 2]),
            self.center,
            self.radius,
            self.owner,
            self.ring_vertices(),
            self.angles,
            self.edge_start,
            self.edge_end,
        )


class Hits(NamedTuple):
    """The nearest hit of every beam of ``S`` sensors, filled in by the casts.

    The field order is the argument order of the compiled kernel.
    """

    distance: np.ndarray  # (S, N) distance to the nearest hit, inf for none
    target: np.ndarray  # (S, N) object index of that hit, -1 for none
    ambiguous: np.ndarray  # (S,) the sensor must cast the scan alone

    @classmethod
    def none(cls, n_sensors: int, n_beams: int) -> Hits:
        """No hits yet."""
        return cls(
            np.full((n_sensors, n_beams), np.inf),
            np.full((n_sensors, n_beams), -1, dtype=np.int64),
            np.zeros(n_sensors, dtype=bool),
        )


# A scene without segments or circle bodies (typed for the compiled kernel).
_NO_SEGMENTS = Segments(
    np.empty((0, 2)), np.empty((0, 2)), np.empty((0, 4)), np.empty(0, dtype=np.int64)
)
_NO_CIRCLES = CircleBodies(
    objects=[],
    owner=np.empty(0, dtype=np.int64),
    center=np.empty((0, 2)),
    radius=np.empty(0),
    angles=np.empty((0, 1)),
    edge_start=np.empty((0, 1), dtype=np.int64),
    edge_end=np.empty((0, 1), dtype=np.int64),
    shared_angles=True,
)


# -------------------------------------------------------------------- caster
class RayCast2DBatch:
    """Casts the beams of all plain ``Lidar2D`` sensors in one pass per step.

    The compiled :func:`cast_kernel` casts when numba is installed, the NumPy
    kernels of this module otherwise.
    """

    def __init__(self) -> None:
        # The compiled cast_kernel, or None for the NumPy kernels.
        self._kernel = cast_kernel if NUMBA_AVAILABLE else None
        self._tree = None  # the geometry tree of the scene the scans were cast in
        # Lidar2D -> _Scan of that cast; None: the tree is not cast yet
        self._scans: dict | None = None
        self._boundaries: dict = {}  # obj id -> (geometry, (start, end))
        self._circle_polygons: dict = {}  # obj id -> (geometry, CirclePolygon | None)
        self._sensor_groups = _LastBuilt()
        self._static_segments = _LastBuilt()
        self._circle_bodies = _LastBuilt()

    @property
    def backend(self) -> str:
        """``"numba"`` or ``"numpy"``: the kernel in use."""
        return "numpy" if self._kernel is None else "numba"

    def scan(self, sensor, state) -> _Scan | None:
        """The scan of ``sensor`` from ``state``, out of one cast of all lidars
        of its environment.

        All lidars are recast when the environment has rebuilt its geometry
        tree since the last cast, as it does in every step. Returns ``None``
        when the sensor must cast alone: the batch does not cast it (a
        ``Lidar2D`` subclass or angular noise), its scan is ambiguous,
        ``state`` differs from the carrier pose the scan was cast from, or an
        object was moved by hand since the cast. In that last case the batch
        takes the tree rebuilt for the move as its own and casts every lidar
        on it at the next sensor step that finds nothing moved in between:
        a loop of single moves and scans casts each scan alone, while many
        moves followed by many scans cast every lidar once.
        """
        env_param = sensor._env_param
        if env_param.geometry_changed and env_param.GeometryTree is self._tree:
            self._tree = env_param.geometry_tree()  # rebuilt after the manual move
            self._scans = None
            return None
        tree = env_param.geometry_tree()
        if tree is not self._tree or self._scans is None:
            self._tree = tree
            self._scans = {}
            for group in self._groups(env_param.objects):
                self._scans.update(self._scan_group(group, env_param.objects))
        scan = self._scans.get(sensor)
        if (
            scan is None
            or (sensor.noise and sensor.angle_std > 0)
            or not np.array_equal(scan.state, state)
        ):
            return None
        return scan

    # ---------------------------------------------------------------- rays
    def _groups(self, objects) -> list[_SensorGroup]:
        """The lidars the batch casts, grouped by beam count (cached)."""
        found = _batched_lidars(objects)
        key = tuple(
            (id(s), i, id(s._original_geometry), s.range_max, s.has_velocity)
            for s, i in found
        )
        return self._sensor_groups.get(key, lambda: _group_by_beam_count(found))

    def _scan_group(self, group: _SensorGroup, objects) -> dict:
        """The unambiguous scans of one sensor group, cast together."""
        poses = np.array([_pose(objects[i]) for i in group.owner])
        beams = _beams_to_world(group.beams, poses)
        origin = beams[:, 0]
        direction = (beams[:, 1::2] - origin[:, None]) / group.max_range[:, None, None]
        rays = Rays(origin, direction, group.max_range, group.owner)
        hits = self._cast(objects, rays, group.track)
        heading = poses[:, 2] + np.array([s.offset[2, 0] for s in group.sensors])
        return {
            sensor: _Scan(
                state=poses[k, :, None],
                origin=origin[k],
                heading=heading[k],
                direction=direction[k],
                distance=hits.distance[k],
                target=hits.target[k],
                objects=objects,
            )
            for k, sensor in enumerate(group.sensors)
            if not hits.ambiguous[k]
        }

    # ---------------------------------------------------------------- cast
    def _cast(self, objects, rays: Rays, track: bool) -> Hits:
        """The nearest hit of every beam in the scene of ``objects``.

        ``track``: some sensor reports hit velocities, so the NumPy kernels
        keep ``Hits.target`` and recast scans with tied targets (the compiled
        kernel always does).
        """
        segments, circles = self._scene(objects)
        hits = Hits.none(*rays.direction.shape[:2])
        if not self._cast_compiled(rays, segments, circles, hits):
            _cast_segments(rays, segments, hits, track)
            _cast_circles(rays, circles, hits, track)
        return hits

    def _cast_compiled(self, rays, segments, circles, hits) -> bool:
        """Cast with the numba kernel, if there is one. A kernel that fails (a
        numba install that cannot compile it) is dropped with a warning.
        ``False`` leaves the cast to the NumPy kernels."""
        if self._kernel is None:
            return False
        try:
            self._kernel(
                *rays,
                *segments,
                *circles.kernel_arrays(),
                ORIGIN_EPS,
                ROUND_OFF,
                *hits,
            )
        except Exception as error:
            warnings.warn(
                f"The numba lidar kernel failed ({error}); scanning with NumPy.",
                RuntimeWarning,
                stacklevel=2,
            )
            self._kernel = None
            return False
        return True

    # --------------------------------------------------------------- scene
    def _scene(self, objects) -> tuple[Segments, CircleBodies]:
        """What lidars see in ``objects``: boundary segments and circle bodies."""
        static, moving, circle = [], [], []
        for index, obj in enumerate(objects):
            kind, data = self._classify(obj)
            if kind == "static":
                static.append((index, obj, data))
            elif kind == "moving":
                moving.append((index, obj))
            elif kind == "circle":
                circle.append((index, obj, data))
        if len({len(polygon.angles) for _, _, polygon in circle}) > 1:
            # Circle polygons of different sizes cannot share one vertex array:
            # cast them as plain polygons, moved to their poses every step.
            moving += [(index, obj) for index, obj, _ in circle]
            circle = []
        if circle:
            placed = _placed_circles(circle)
            moving += [
                (i, obj)
                for (i, obj, _), ok in zip(circle, placed, strict=True)
                if not ok
            ]
            circle = [item for item, ok in zip(circle, placed, strict=True) if ok]
        static_segments = self._static_segments.get(
            tuple((i, obj._id, id(geometry)) for i, obj, geometry in static),
            lambda: Segments.concat(
                Segments.of(*self._boundary(obj, geometry), i)
                for i, obj, geometry in static
            ),
        )
        circles = self._circle_bodies.get(
            tuple((i, id(obj), id(polygon)) for i, obj, polygon in circle),
            lambda: CircleBodies.gather(circle),
        )
        moving_segments = [self._moved_segments(i, obj) for i, obj in moving]
        return Segments.concat([static_segments, *moving_segments]), circles

    def _classify(self, obj):
        """How ``obj`` enters the cast: ``("static", world geometry)``,
        ``("moving", None)``, ``("circle", CirclePolygon)``, or ``(None, None)``
        when lidars do not see it."""
        if not obj._geometry_valid or obj.unobstructed:
            return None, None
        if obj.shape == "map":
            return "static", obj.geometry
        if obj.shape == "circle":
            polygon = self._circle_polygon(obj)
            if polygon is not None:
                return "circle", polygon
        if obj.static:
            return "static", obj._geometry
        return "moving", None

    def _moved_segments(self, index: int, obj) -> Segments:
        """The boundary of a moving body, read from its current world geometry:
        the coordinates the per-sensor scan casts against."""
        return Segments.of(*self._boundary(obj, obj._geometry), index)

    def _boundary(self, obj, geometry) -> tuple[np.ndarray, np.ndarray]:
        """``(start, end)`` of the boundary segments of ``geometry`` (cached)."""
        return _memo(self._boundaries, obj, geometry, _boundary_arrays)

    def _circle_polygon(self, obj) -> CirclePolygon | None:
        """Metadata and source vertices of the base shape (cached). The scene
        verifies that this shape has been placed before using the metadata."""
        return _memo(
            self._circle_polygons, obj, obj.gf._original_geometry, _as_circle_polygon
        )


def scan(sensor, state) -> _Scan | None:
    """The batched scan of ``sensor`` from ``state`` (see :meth:`RayCast2DBatch.scan`)."""
    return _caster.scan(sensor, state)


# ------------------------------------------------------------------- sensors
class _SensorGroup(NamedTuple):
    """Plain lidars with the same beam count, cast together."""

    sensors: list  # the Lidar2D sensors
    owner: np.ndarray  # (S,) object index of the body carrying each sensor
    max_range: np.ndarray  # (S,)
    beams: np.ndarray  # (S, 2N, 2) body-frame beam ends: origin, end, origin, end...
    track: bool  # some sensor reports hit velocities


class _Scan(NamedTuple):
    """One sensor's share of a batched cast, before its range noise."""

    state: np.ndarray  # pose of the carrier the scan was cast from
    origin: np.ndarray  # (2,) sensor position
    heading: float  # world heading of the sensor frame (carrier + offset)
    direction: np.ndarray  # (N, 2) unit beam directions
    distance: np.ndarray  # (N,) distance to the nearest hit, inf for none
    target: np.ndarray  # (N,) index into ``objects`` of that hit, -1 for none
    objects: list  # the environment's objects (for hit velocities)


def _batched_lidars(objects) -> list:
    """``(sensor, object index)`` of every sensor the batch casts: plain
    ``Lidar2D`` (not a subclass) without angular noise."""
    from irsim.world.sensors.lidar2d import Lidar2D

    return [
        (sensor, index)
        for index, obj in enumerate(objects)
        for sensor in obj.sensors
        if type(sensor) is Lidar2D and not (sensor.noise and sensor.angle_std > 0)
    ]


def _group_by_beam_count(found) -> list[_SensorGroup]:
    """``found`` = ``[(sensor, object index)]`` as groups of equal beam count."""
    by_count: dict[int, list] = {}
    for sensor, owner in found:
        by_count.setdefault(sensor.number, []).append((sensor, owner))
    groups = []
    for items in by_count.values():
        sensors = [sensor for sensor, _ in items]
        groups.append(
            _SensorGroup(
                sensors=sensors,
                owner=np.array([owner for _, owner in items], dtype=int),
                max_range=np.array([float(s.range_max) for s in sensors]),
                beams=np.stack(
                    [shapely.get_coordinates(s._original_geometry) for s in sensors]
                ),
                track=any(s.has_velocity for s in sensors),
            )
        )
    return groups


# --------------------------------------------------------------------- scene
class _LastBuilt:
    """The last value built, kept until it is asked for with another key."""

    def __init__(self) -> None:
        self._key = None
        self._value = None

    def get(self, key, build):
        if key != self._key:
            self._value, self._key = build(), key
        return self._value


def _memo(cache: dict, obj, geometry, build):
    """``build(geometry)`` for ``obj``, rebuilt only when its geometry is replaced."""
    entry = cache.get(obj._id)
    if entry is None or entry[0] is not geometry:
        entry = cache[obj._id] = (geometry, build(geometry))
    return entry[1]


def _boundary_arrays(geometry) -> tuple[np.ndarray, np.ndarray]:
    start, end = boundary_segments(geometry)
    return (
        np.ascontiguousarray(start, dtype=float),
        np.ascontiguousarray(end, dtype=float),
    )


def _as_circle_polygon(geometry) -> CirclePolygon | None:
    """``geometry`` as a circle polygon: a convex polygon whose vertices lie on
    one circle at even angles (a circle buffer). ``None`` for anything else,
    which is then cast as plain segments."""
    if geometry.geom_type != "Polygon" or geometry.is_empty or geometry.interiors:
        return None
    coords = np.asarray(geometry.exterior.coords, dtype=float)[:-1]
    center = coords.mean(axis=0)
    dist = np.hypot(*(coords - center).T)
    radius = float(dist.max())
    if radius <= 0 or not np.all(dist >= radius * (1 - _CIRCLE_RTOL)):
        return None
    angles = np.arctan2(coords[:, 1] - center[1], coords[:, 0] - center[0])
    order = np.argsort(angles)
    angles = angles[order]
    gaps = np.diff(np.r_[angles, angles[0] + 2 * np.pi])
    even = np.allclose(gaps, 2 * np.pi / len(coords), rtol=_CIRCLE_RTOL, atol=0)
    if not even or not geometry.equals(geometry.convex_hull):
        return None
    # Edge k joins vertex order[k] and the next vertex by angle; both are
    # neighbours on the ring, which may run either way round.
    following = np.roll(order, -1)
    forward = following == (order + 1) % len(coords)
    edge_start = np.where(forward, order, following)
    edge_end = np.where(forward, following, order)
    return CirclePolygon(angles, edge_start, edge_end, center, radius, coords)


def _placed_circles(items) -> np.ndarray:
    """Which circle sources match their current world geometry, including
    vertex order? Unplaced replacements must be cast as boundary segments.
    All sources in ``items`` have the same vertex count."""
    geometries = np.array([obj._geometry for _, obj, _ in items], dtype=object)
    count = len(items[0][2].vertices)
    placed = shapely.get_num_coordinates(geometries) == count + 1
    matching = np.flatnonzero(placed)
    if matching.size:
        sources = np.stack([items[i][2].vertices for i in matching])
        poses = np.array([_pose(items[i][1]) for i in matching])
        expected = _beams_to_world(sources, poses)
        world = shapely.get_coordinates(geometries[matching]).reshape(-1, count + 1, 2)
        placed[matching] = np.all(world[:, :count] == expected, axis=(1, 2)) & np.all(
            world[:, count] == expected[:, 0], axis=1
        )
    return placed


def _pose(obj) -> np.ndarray:
    """``[x, y, theta]`` of an object as a flat float array."""
    return np.asarray(obj.state, dtype=float).reshape(-1)[:3]


def _to_world(points: np.ndarray, poses: np.ndarray) -> np.ndarray:
    """Move body-frame points ``(..., 2)`` by poses ``[x, y, theta]``: one pose
    ``(3,)``, or one pose ``(C, 3)`` per leading row of points ``(C, M, 2)``."""
    x, y, theta = poses[..., 0:1], poses[..., 1:2], poses[..., 2:3]
    c, s = np.cos(theta), np.sin(theta)
    px, py = points[..., 0], points[..., 1]
    return np.stack((px * c - py * s + x, px * s + py * c + y), axis=-1)


def _beams_to_world(beams: np.ndarray, poses: np.ndarray) -> np.ndarray:
    """World beam ends ``(S, 2N, 2)`` of the sensors, computed as
    :func:`~irsim.util.util.geometry_transform` does for one sensor (one scalar
    ``cos`` / ``sin`` per pose), so batched rays match per-sensor rays bit for
    bit."""
    cos = np.array([np.cos(theta) for theta in poses[:, 2]])[:, None]
    sin = np.array([np.sin(theta) for theta in poses[:, 2]])[:, None]
    px, py = beams[..., 0], beams[..., 1]
    return np.stack(
        (px * cos - py * sin + poses[:, 0:1], px * sin + py * cos + poses[:, 1:2]),
        axis=-1,
    )


# ------------------------------------------------------------- NumPy kernels
# A cast pairs sensors with the primitives within their range, intersects all
# beams of a sensor with a pair's primitive in one array operation, and folds
# the distances into ``hits``. Candidate pairs are generated and consumed in
# bounded blocks, sorted by sensor (``np.nonzero`` order).


def _cast_segments(rays: Rays, segments: Segments, hits: Hits, track: bool) -> None:
    """Intersect every beam with the segments whose box is within its range."""
    for sensor, segment in _segments_in_range(rays, segments):
        for block in _blocks(len(sensor), rays.direction.shape[1]):
            s, e = sensor[block], segment[block]
            distance, ambiguous = _crossings(
                rays.origin[s][:, None],
                rays.direction[s],
                rays.max_range[s][:, None],
                segments.start[e][:, None],
                segments.end[e][:, None],
            )
            hits.ambiguous[s[ambiguous.any(axis=1)]] = True
            _keep_nearest(hits, s, distance, segments.owner[e], track)


def _cast_circles(rays: Rays, circles: CircleBodies, hits: Hits, track: bool) -> None:
    """Intersect every beam with the circle polygons within its range."""
    poses = circles.poses()
    centers = circles.centers(poses)
    vertices = circles.ring_vertices()
    for sensor, circle in _circles_in_range(rays, centers, circles):
        for block in _blocks(len(sensor), rays.direction.shape[1]):
            s, c = sensor[block], circle[block]
            origin, direction = rays.origin[s], rays.direction[s]
            max_range = rays.max_range[s]
            t_in, touches, ambiguous = _circle_crossings(
                origin, direction, max_range, centers[c], circles.radius[c]
            )
            hits.ambiguous[s[ambiguous]] = True
            pair, beam = np.nonzero(touches)
            if len(pair) == 0:
                continue
            near, unsure = _polygon_distances(
                origin[pair],
                direction[pair, beam],
                max_range[pair],
                t_in[pair, beam],
                c[pair],
                poses,
                centers,
                vertices,
                circles,
            )
            hits.ambiguous[s[pair[unsure]]] = True
            distance = np.full(touches.shape, np.inf)
            distance[pair, beam] = near
            _keep_nearest(hits, s, distance, circles.owner[c], track)


def _segments_in_range(rays: Rays, segments: Segments):
    """Yield bounded ``(sensor, segment)`` candidate arrays; a sensor never
    sees its own body."""
    for sensors, items in _pair_blocks(len(rays.origin), len(segments.start)):
        x, y = rays.origin[sensors, 0:1], rays.origin[sensors, 1:2]
        r = rays.max_range[sensors, None]
        box = segments.box[items]
        near = (
            (box[None, :, 0] <= x + r)
            & (box[None, :, 1] >= x - r)
            & (box[None, :, 2] <= y + r)
            & (box[None, :, 3] >= y - r)
            & (segments.owner[None, items] != rays.owner[sensors, None])
        )
        s, i = np.nonzero(near)
        yield s + sensors.start, i + items.start


def _circles_in_range(rays: Rays, centers: np.ndarray, circles: CircleBodies):
    """Yield bounded ``(sensor, circle)`` candidate arrays within range."""
    for sensors, items in _pair_blocks(len(rays.origin), len(centers)):
        dx = centers[None, items, 0] - rays.origin[sensors, 0:1]
        dy = centers[None, items, 1] - rays.origin[sensors, 1:2]
        reach = rays.max_range[sensors, None] + circles.radius[None, items]
        reach += ROUND_OFF * np.maximum(1.0, reach)
        near = (dx * dx + dy * dy <= reach**2) & (
            circles.owner[None, items] != rays.owner[sensors, None]
        )
        s, i = np.nonzero(near)
        yield s + sensors.start, i + items.start


def _pair_blocks(n_sensors: int, n_items: int):
    """Tile the sensor x primitive test, bounding both dimensions. Split the
    primitives too when a single sensor's row exceeds MAX_BLOCK_ELEMENTS."""
    for sensors in _blocks(n_sensors, n_items):
        for items in _blocks(n_items, sensors.stop - sensors.start):
            yield sensors, items


def _blocks(n_rows: int, row_size: int):
    """Slices holding at most MAX_BLOCK_ELEMENTS values, or one row at least."""
    size = max(1, MAX_BLOCK_ELEMENTS // max(1, row_size))
    return (slice(start, min(start + size, n_rows)) for start in range(0, n_rows, size))


def _cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """2D cross product ``a x b`` over the last axis."""
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _crossings(origin, direction, max_range, start, end):
    """Where beams ``origin + t * direction`` cross segments ``start -> end``.

    The inputs broadcast against each other (with a trailing ``xy`` axis, but
    for ``max_range``). Returns ``t`` of each crossing in
    ``(ORIGIN_EPS, max_range]`` (``inf`` where there is none), and whether
    rounding could decide it: the crossing lies within ``ROUND_OFF`` of a
    segment end, the range limit or the origin, or the beam runs along the
    segment.
    """
    edge = end - start
    to_start = start - origin
    denom = _cross(direction, edge)
    num_t = _cross(to_start, edge)
    num_u = _cross(to_start, direction)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = num_t / denom  # position along the beam
        u = num_u / denom  # position along the segment: 0 at start, 1 at end
    tol = ROUND_OFF * np.maximum(1.0, max_range)
    near = (t >= -tol) & (t <= max_range + tol) & (u >= -tol) & (u <= 1 + tol)
    at_a_limit = (
        (u <= tol)
        | (u >= 1 - tol)
        | (np.abs(t - max_range) <= tol)
        | (np.abs(t - ORIGIN_EPS) <= tol)
    )
    along = (np.abs(denom) <= tol) & (np.abs(num_t) <= ORIGIN_EPS + tol)
    hit = (denom != 0) & (u >= 0) & (u <= 1) & (t > ORIGIN_EPS) & (t <= max_range)
    return np.where(hit, t, np.inf), (near & at_a_limit) | along


def _circle_crossings(origin, direction, max_range, center, radius):
    """Where every beam of a pair block meets the circle of its pair.

    Args:
        origin: ``(P, 2)``; direction: ``(P, N, 2)``; max_range: ``(P,)``;
            center: ``(P, 2)``; radius: ``(P,)``.

    Returns:
        ``t_in`` ``(P, N)``, the distance at which each beam enters the circle;
        ``touches`` ``(P, N)``, the beam meets the circle ahead of the sensor
        and within range; ``ambiguous`` ``(P,)``, a beam grazes the circle or
        enters it at the range limit.
    """
    to_origin = origin - center
    # |to_origin + t * direction| = radius  <=>  t^2 + 2 b t + c = 0
    b = to_origin[:, 0:1] * direction[..., 0] + to_origin[:, 1:2] * direction[..., 1]
    c = (to_origin[:, 0] ** 2 + to_origin[:, 1] ** 2 - radius**2)[:, None]
    disc = b * b - c
    root = np.sqrt(np.maximum(disc, 0.0))
    t_in, t_out = -b - root, -b + root
    limit = max_range[:, None]
    touches = (disc >= 0) & (t_out > ORIGIN_EPS) & (t_in <= limit)
    grazing = np.abs(disc) <= ROUND_OFF * np.maximum(1.0, b * b + np.abs(c))
    at_limit = np.abs(t_in - limit) <= ROUND_OFF * np.maximum(1.0, limit)
    return t_in, touches, np.any(grazing | at_limit, axis=1)


def _polygon_distances(
    origin, direction, max_range, t_in, body, poses, centers, vertices, circles
):
    """Exact distances of ``K`` beams to the circle polygons they touch.

    A beam entering the circle from outside can only cross the three polygon
    edges around its entry point; a beam starting inside may cross any edge.

    Args:
        origin, direction: ``(K, 2)`` of each beam; max_range, t_in: ``(K,)``.
        body: ``(K,)`` index of the touched circle body.
        poses, centers, vertices: World poses ``(C, 3)``, centres ``(C, 2)``
            and ring vertices ``(C, M, 2)`` of the circle bodies.
        circles: The :class:`CircleBodies` (vertex angles and edge ends).

    Returns:
        ``(K,)`` distances (``inf`` where the polygon is missed) and ``(K,)``
        ambiguous flags.
    """
    distance = np.full(len(body), np.inf)
    ambiguous = np.zeros(len(body), dtype=bool)

    def cross_edges(rows, start, end):
        t, unsure = _crossings(
            origin[rows][:, None],
            direction[rows][:, None],
            max_range[rows][:, None],
            start,
            end,
        )
        distance[rows] = t.min(axis=1)
        ambiguous[rows] = unsure.any(axis=1)

    entering = t_in > ORIGIN_EPS
    if entering.any():
        entry = origin[entering] + t_in[entering][:, None] * direction[entering]
        edges = _edges_around(entry, body[entering], poses, centers, vertices, circles)
        cross_edges(entering, *edges)
    inside = ~entering
    if inside.any():
        start = vertices[body[inside]]  # every ring edge, in the ring's direction
        cross_edges(inside, start, np.roll(start, -1, axis=1))
    return distance, ambiguous


def _edges_around(entry, body, poses, centers, vertices, circles):
    """``(start, end)``, each ``(K, 3, 2)``: the three polygon edges around
    each entry point, with their ends in the ring's direction."""
    angle = (
        np.arctan2(entry[:, 1] - centers[body, 1], entry[:, 0] - centers[body, 0])
        - poses[body, 2]
    )
    angle = (angle + np.pi) % (2 * np.pi) - np.pi  # body frame, in [-pi, pi)
    # The last vertex at or below the entry angle.
    if circles.shared_angles:
        last = np.searchsorted(circles.angles[0], angle, side="right") - 1
    else:
        last = np.count_nonzero(circles.angles[body] <= angle[:, None], axis=1) - 1
    edge = (last[:, None] + _ENTRY_EDGES) % vertices.shape[1]
    polygon = body[:, None]
    start = circles.edge_start[polygon, edge]
    end = circles.edge_end[polygon, edge]
    return vertices[polygon, start], vertices[polygon, end]


def _keep_nearest(hits: Hits, sensor, distance, target, track: bool) -> None:
    """Fold the distances of a pair block into each sensor's nearest hit.

    Args:
        hits: The running nearest hits, updated in place.
        sensor: ``(P,)`` sensor index of each pair, sorted.
        distance: ``(P, N)`` beam distances to each pair's primitive
            (``inf`` where it is missed).
        target: ``(P,)`` object index of each pair's primitive.
        track: Also keep the hit targets; without it, only distances.
    """
    first = np.empty(len(sensor), dtype=bool)  # first pair of each sensor
    first[0] = True
    np.not_equal(sensor[1:], sensor[:-1], out=first[1:])
    starts = np.flatnonzero(first)
    sensors = sensor[starts]
    nearest = np.minimum.reduceat(distance, starts, axis=0)  # (G, N) per sensor
    current = hits.distance[sensors]
    if not track:
        hits.distance[sensors] = np.minimum(current, nearest)
        return
    group = np.cumsum(first) - 1  # position of each pair's sensor in ``sensors``
    # The per-sensor scan picks among equally near targets by STRtree order;
    # recast such scans rather than let primitive order pick the velocity.
    tied = np.isfinite(distance) & _close(distance, nearest[group])
    ties = np.add.reduceat(tied.astype(int), starts, axis=0) > 1
    ties |= np.isfinite(nearest) & _close(nearest, current)
    hits.ambiguous[sensors[np.any(ties, axis=1)]] = True
    closer = nearest < current
    if not closer.any():
        return
    pair, beam = np.nonzero((distance == nearest[group]) & closer[group])
    hits.distance[sensors] = np.minimum(current, nearest)
    hits.target[sensor[pair], beam] = target[pair]


def _close(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.isclose(a, b, rtol=ROUND_OFF, atol=ROUND_OFF)


# ----------------------------------------------------------- compiled kernel
# With numba installed (``pip install ir-sim[fast]``), ``cast_kernel`` runs the
# cast stage in plain loops: for every sensor, every segment and circle within
# its range, every beam. It uses the formulas and tolerances of the NumPy
# kernels above, so both give the same scans and mark the same kinds of scans
# ambiguous. Compiled code is cached on disk after the first call. The helpers
# are inlined into the kernel and kept pure (scalars in, scalars out): an
# inner-loop helper that wrote into array arguments ran the kernel about 3x
# slower, even when inlined.

if NUMBA_AVAILABLE:
    _jit_inlined = njit(cache=True, nogil=True, inline="always")

    @njit(cache=True, nogil=True)
    def cast_kernel(
        origin, direction, max_range, owner,
        seg_start, seg_end, seg_box, seg_owner,
        circle_xy, circle_theta, circle_center, circle_radius, circle_owner,
        circle_vertices, circle_angles, circle_edge_start, circle_edge_end,
        eps, round_off,
        distance, target, ambiguous,
    ):  # fmt: skip
        """Nearest hit of every beam of every sensor.

        The arguments are the fields of the ``Rays``, ``Segments``,
        ``CircleBodies.kernel_arrays()`` and ``Hits`` of
        this module, in that order:

        Args:
            origin: ``(S, 2)`` sensor positions; direction: ``(S, N, 2)`` unit
                beam directions; max_range, owner: ``(S,)`` range and object
                index of the body carrying each sensor (not seen by it).
            seg_start, seg_end: ``(E, 2)`` boundary segments; seg_box:
                ``(E, 4)`` their boxes ``[min_x, max_x, min_y, max_y]``;
                seg_owner: ``(E,)`` object index.
            circle_xy, circle_theta: ``(C, 2)`` positions and ``(C,)`` headings
                of the circle bodies; circle_center: ``(C, 2)`` polygon centres
                in the body frame; circle_radius, circle_owner: ``(C,)``;
                circle_vertices: ``(C, M, 2)`` world vertices in ring order;
                circle_angles: ``(C, M)`` body-frame vertex angles, ascending;
                circle_edge_start, circle_edge_end: ``(C, M)`` ring indices of
                the ends of edge k (by angle), in the ring's direction.
            eps: Origin epsilon of the per-sensor scan.
            round_off: Relative tolerance that marks scans ambiguous.
            distance, target: ``(S, N)`` outputs, preset to ``inf`` / ``-1``.
            ambiguous: ``(S,)`` output, preset to ``False``.
        """
        for s in range(origin.shape[0]):
            ox, oy = origin[s, 0], origin[s, 1]
            r, own = max_range[s], owner[s]
            _jit_cast_segments(
                s, ox, oy, r, own, direction,
                seg_start, seg_end, seg_box, seg_owner,
                eps, round_off, distance, target, ambiguous,
            )  # fmt: skip
            _jit_cast_circles(
                s, ox, oy, r, own, direction,
                circle_xy, circle_theta, circle_center, circle_radius, circle_owner,
                circle_vertices, circle_angles, circle_edge_start, circle_edge_end,
                eps, round_off, distance, target, ambiguous,
            )  # fmt: skip

    # ---------------------------------------------------------- segments
    @_jit_inlined
    def _jit_cast_segments(
        s, ox, oy, r, own, direction,
        seg_start, seg_end, seg_box, seg_owner,
        eps, round_off, distance, target, ambiguous,
    ):  # fmt: skip
        """Cast every beam of sensor ``s`` at the segments within its range."""
        for j in range(seg_start.shape[0]):
            if seg_owner[j] == own:
                continue
            if _jit_box_out_of_reach(
                seg_box[j, 0], seg_box[j, 1], seg_box[j, 2], seg_box[j, 3], ox, oy, r
            ):
                continue
            ax, ay = seg_start[j, 0], seg_start[j, 1]
            bx, by = seg_end[j, 0], seg_end[j, 1]
            for n in range(direction.shape[1]):
                t, unsure = _jit_crossing(
                    ox, oy, direction[s, n, 0], direction[s, n, 1],
                    ax, ay, bx, by, eps, r, round_off,
                )  # fmt: skip
                closer, unsure = _jit_compare(t, unsure, distance[s, n], r, round_off)
                ambiguous[s] = ambiguous[s] or unsure
                if closer:
                    distance[s, n] = t
                    target[s, n] = seg_owner[j]

    @_jit_inlined
    def _jit_box_out_of_reach(min_x, max_x, min_y, max_y, ox, oy, r):
        """Whether a bounding box lies outside the square of half-side ``r``."""
        return min_x > ox + r or max_x < ox - r or min_y > oy + r or max_y < oy - r

    @_jit_inlined
    def _jit_crossing(ox, oy, dx, dy, ax, ay, bx, by, eps, max_r, round_off):
        """Distance along beam ``(ox, oy) + t (dx, dy)`` to segment ``a -> b``
        (``inf`` when it is not crossed within range), and whether rounding
        could decide it (as :func:`_crossings`)."""
        ex, ey = bx - ax, by - ay  # the segment
        sx, sy = ax - ox, ay - oy  # sensor -> segment start
        denom = dx * ey - dy * ex
        num_t = sx * ey - sy * ex
        tol = round_off * max(1.0, max_r)
        along = abs(denom) <= tol and abs(num_t) <= eps + tol
        if denom == 0.0:
            # Parallel: never crossed; a beam along the segment is ambiguous,
            # so the sensor casts alone.
            return math.inf, along
        t = num_t / denom  # position along the beam
        u = (sx * dy - sy * dx) / denom  # position along the segment: 0 at a, 1 at b
        near = -tol <= t <= max_r + tol and -tol <= u <= 1 + tol
        at_a_limit = (
            u <= tol or u >= 1 - tol or abs(t - max_r) <= tol or abs(t - eps) <= tol
        )
        unsure = along or (near and at_a_limit)
        if eps < t <= max_r and 0.0 <= u <= 1.0:
            return t, unsure
        return math.inf, unsure

    @_jit_inlined
    def _jit_compare(t, unsure, nearest, max_r, round_off):
        """Whether a hit at ``t`` is closer than ``nearest`` so far, and whether
        the scan becomes ambiguous (an unsure hit, or a tie with ``nearest``)."""
        tied = math.isfinite(t) and abs(t - nearest) <= round_off * max(1.0, max_r)
        return t < nearest, unsure or tied

    # ----------------------------------------------------------- circles
    @_jit_inlined
    def _jit_cast_circles(
        s, ox, oy, r, own, direction,
        circle_xy, circle_theta, circle_center, circle_radius, circle_owner,
        circle_vertices, circle_angles, circle_edge_start, circle_edge_end,
        eps, round_off, distance, target, ambiguous,
    ):  # fmt: skip
        """Cast every beam of sensor ``s`` at the circle bodies within its range:
        first at the circle, then at the polygon edges around the entry point."""
        for c in range(circle_xy.shape[0]):
            if circle_owner[c] == own:
                continue
            ct, st = math.cos(circle_theta[c]), math.sin(circle_theta[c])
            px, py = circle_xy[c, 0], circle_xy[c, 1]
            cx = px + (ct * circle_center[c, 0] - st * circle_center[c, 1])
            cy = py + (st * circle_center[c, 0] + ct * circle_center[c, 1])
            radius = circle_radius[c]
            if not _jit_within_reach(cx - ox, cy - oy, r + radius, round_off):
                continue
            ocx, ocy = ox - cx, oy - cy
            cc = ocx * ocx + ocy * ocy - radius * radius
            for n in range(direction.shape[1]):
                dx, dy = direction[s, n, 0], direction[s, n, 1]
                t_in, t_out, unsure = _jit_circle_crossing(
                    ocx, ocy, cc, dx, dy, r, round_off
                )
                if unsure:
                    ambiguous[s] = True
                if not _jit_touches(t_in, t_out, eps, r):
                    continue
                _jit_cast_polygon_beam(
                    s, n, c, ox, oy, dx, dy, r, cx, cy, t_in,
                    circle_theta, circle_vertices, circle_angles,
                    circle_edge_start, circle_edge_end, circle_owner,
                    eps, round_off, distance, target, ambiguous,
                )  # fmt: skip

    @_jit_inlined
    def _jit_within_reach(dx, dy, reach, round_off):
        """Whether a circle centred ``(dx, dy)`` from the sensor can be reached."""
        reach += round_off * max(1.0, reach)
        return dx * dx + dy * dy <= reach * reach

    @_jit_inlined
    def _jit_circle_crossing(ocx, ocy, cc, dx, dy, r, round_off):
        """Entry and exit distances of a beam through a circle (``nan`` on a
        miss), and whether it grazes the circle or enters it at the range limit."""
        b = ocx * dx + ocy * dy
        disc = b * b - cc
        grazing = abs(disc) <= round_off * max(1.0, b * b + abs(cc))
        if disc < 0.0:
            return math.nan, math.nan, grazing
        root = math.sqrt(disc)
        t_in = -b - root
        at_limit = abs(t_in - r) <= round_off * max(1.0, r)
        return t_in, -b + root, grazing or at_limit

    @_jit_inlined
    def _jit_touches(t_in, t_out, eps, r):
        """Whether a beam meets the circle ahead of the sensor and within range
        (``False`` for a miss, whose distances are ``nan``)."""
        return t_out > eps and t_in <= r

    @_jit_inlined
    def _jit_cast_polygon_beam(
        s, n, c, ox, oy, dx, dy, r, cx, cy, t_in,
        circle_theta, circle_vertices, circle_angles,
        circle_edge_start, circle_edge_end, circle_owner,
        eps, round_off, distance, target, ambiguous,
    ):  # fmt: skip
        """Cast beam ``n`` of sensor ``s`` at the polygon of circle ``c``: the
        three edges around the entry point, or all edges from inside, each with
        its ends in the ring's direction."""
        n_vertices = circle_vertices.shape[1]
        if t_in <= eps:
            first, count = 0, n_vertices
        else:
            angle = (
                math.atan2(oy + t_in * dy - cy, ox + t_in * dx - cx) - circle_theta[c]
            )
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            first, count = _jit_first_entry_edge(circle_angles, c, angle), 3
        for k in range(count):
            e = (first + k) % n_vertices
            a, b = circle_edge_start[c, e], circle_edge_end[c, e]
            t, unsure = _jit_crossing(
                ox, oy, dx, dy,
                circle_vertices[c, a, 0], circle_vertices[c, a, 1],
                circle_vertices[c, b, 0], circle_vertices[c, b, 1],
                eps, r, round_off,
            )  # fmt: skip
            closer, unsure = _jit_compare(t, unsure, distance[s, n], r, round_off)
            ambiguous[s] = ambiguous[s] or unsure
            if closer:
                distance[s, n] = t
                target[s, n] = circle_owner[c]

    @_jit_inlined
    def _jit_first_entry_edge(circle_angles, c, angle):
        """First of the three polygon edges around body-frame ``angle``: the one
        before the edge that starts at the last vertex at or below it."""
        lo, hi = 0, circle_angles.shape[1]
        while lo < hi:  # count the vertices at or below ``angle``
            mid = (lo + hi) // 2
            if circle_angles[c, mid] <= angle:
                lo = mid + 1
            else:
                hi = mid
        return lo - 2

else:  # pragma: no cover - optional dependency
    cast_kernel = None


# The caster behind scan(), built last because it uses the helpers above.
# Environments share it: a scan is reused only while its environment's
# geometry tree is unchanged, and the caches check each object's identity.
_caster = RayCast2DBatch()
