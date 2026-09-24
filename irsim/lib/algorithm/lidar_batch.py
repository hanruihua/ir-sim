"""Batched 2D LiDAR ray casting for every plain ``Lidar2D`` sensor of a world.

One call per simulation step replaces the per-sensor Shapely scans with a few
NumPy operations over all beams of all sensors at once, reproducing the
per-sensor result (:func:`~irsim.lib.algorithm.ray_casting_2d.cast_rays`) to
floating-point precision:

* boundaries of static bodies (walls, boxes, polygons, grid maps) are
  flattened into one segment array with bounding boxes and kept until the set
  of static bodies changes; moving polygonal bodies keep their body-frame
  segments cached and are rigidly transformed with NumPy every step;
* every sensor only meets the primitives whose bounding box or circumscribed
  circle lies within its range (a ``(sensors x primitives)`` test), and the
  surviving (sensor, primitive) pairs are intersected with all beams of the
  sensor in one blocked matrix operation;
* ``circle`` bodies (regular buffer polygons) are pre-tested against their
  circumscribed circle; a beam that touches it is then intersected exactly
  with the polygon edges around the entry point, which is where a convex
  polygon inscribed in that circle can be entered (all edges when the beam
  starts inside the circle);
* the nearest hit per beam is folded with ``np.minimum.reduceat`` over the
  pairs of each sensor;
* sensors receive their ranges, hit objects and origin without building beam
  geometries (those are built lazily when plotted).

Numerically ambiguous scans (grazing beams, endpoint/range-limit hits and
velocity ties) use the per-sensor reference path so rounding cannot change a
hit into a miss or select a different target. Noise is applied in the original
object/sensor order, including when FMCW sensors are interleaved.

``analytic_circles=True`` trades exactness for speed by treating ``circle``
bodies as true circles. The polygon's radial sagitta is not a bound on beam
range error: grazing beams can change between a hit and a max-range miss.

The per-sensor :meth:`~irsim.world.sensors.lidar2d.Lidar2D.step` path stays
available (``world.lidar_batch: false``) and is what sensors stepped outside
:class:`~irsim.env.env_base.EnvBase` use.
"""

from __future__ import annotations

import numpy as np
import shapely

from irsim.lib.algorithm.ray_casting_2d import ORIGIN_EPS, boundary_segments

# Largest (pairs x beams) block processed at once, in matrix elements.
MAX_BLOCK_ELEMENTS = 1 << 22
# Relative tolerance under which all polygon vertices count as lying on the
# circumscribed circle (regular buffer polygons).
_INSCRIBED_RTOL = 1e-9
_NEIGHBOURS = np.array([-1, 0, 1])
# Only selects scans for the reference path; never widens a hit predicate.
ROUND_OFF = 1e-12
# Typed placeholders for the compiled kernel when a scene has no segments / circles.
_EMPTY_SEGMENTS = (
    np.empty((0, 2)),
    np.empty((0, 2)),
    np.empty((0, 4)),
    np.empty(0, dtype=np.int64),
)
_EMPTY_CIRCLES = (
    np.empty((0, 2)),
    np.empty(0),
    np.empty((0, 2)),
    np.empty(0),
    np.empty(0, dtype=np.int64),
    np.empty((0, 1, 2)),
    np.empty((0, 1)),
)


def _rotate_translate(points: np.ndarray, state: np.ndarray) -> np.ndarray:
    """Rotate ``(..., 2)`` body-frame points by ``theta``, then translate
    (mirrors :func:`irsim.util.util.geometry_transform` for arrays)."""
    x, y, theta = float(state[0]), float(state[1]), float(state[2])
    c, s = np.cos(theta), np.sin(theta)
    px, py = points[..., 0], points[..., 1]
    return np.stack((px * c - py * s + x, px * s + py * c + y), axis=-1)


def _segment_bboxes(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    """``(S, 4)`` boxes ``[min_x, max_x, min_y, max_y]`` of segments."""
    return np.column_stack(
        (
            np.minimum(start[:, 0], end[:, 0]),
            np.maximum(start[:, 0], end[:, 0]),
            np.minimum(start[:, 1], end[:, 1]),
            np.maximum(start[:, 1], end[:, 1]),
        )
    )


def _ambiguous_edges(denom, num_t, t, u, maxr):
    """Flag endpoint, range-limit and collinear cases sensitive to roundoff."""
    tol = ROUND_OFF * np.maximum(1.0, maxr)
    nearby = (t >= -tol) & (t <= maxr + tol) & (u >= -tol) & (u <= 1 + tol)
    boundary = (
        (u <= tol)
        | (u >= 1 - tol)
        | (np.abs(t - maxr) <= tol)
        | (np.abs(t - ORIGIN_EPS) <= tol)
    )
    collinear = (np.abs(denom) <= tol) & (np.abs(num_t) <= ORIGIN_EPS + tol)
    return np.any((nearby & boundary) | collinear, axis=1)


def _pair_segment_distances(ray_origins, D, maxr, A, B):
    """Beam/segment hit distances for (pair, beam) combinations.

    Args:
        ray_origins: Pair origins ``(P, 2)``.
        D: Beam directions per pair ``(P, N, 2)``.
        maxr: Range per pair ``(P,)``.
        A, B: One segment per pair ``(P, 2)``.

    Returns:
        ``(P, N)`` distances (``inf`` on a miss), and a ``(P,)`` mask of
        numerically ambiguous pairs requiring the reference scan.
    """
    V = B - A
    AO = A - ray_origins
    Vx, Vy = V[:, 0:1], V[:, 1:2]
    Dx, Dy = D[..., 0], D[..., 1]
    denom = Dx * Vy - Dy * Vx  # D x V     (P, N)
    num_t = (AO[:, 0] * V[:, 1] - AO[:, 1] * V[:, 0])[:, None]  # AO x V
    num_u = AO[:, 0:1] * Dy - AO[:, 1:2] * Dx  # AO x D
    raw_denom = denom
    crossing = denom != 0
    if crossing.all():
        parallel = None
    else:
        parallel = ~crossing
        denom = np.where(crossing, denom, 1.0)
    t = num_t / denom
    u = num_u / denom
    ambiguous = _ambiguous_edges(raw_denom, num_t, t, u, maxr[:, None])
    valid = crossing & (u >= 0) & (u <= 1) & (t > ORIGIN_EPS) & (t <= maxr[:, None])
    t = np.where(valid, t, np.inf)
    if parallel is not None:
        coll = parallel & (np.abs(num_t) <= ORIGIN_EPS)
        pi, ni = np.nonzero(coll)
        d = D[pi, ni]
        dA = np.sum(AO[pi] * d, axis=1)
        dB = np.sum((B[pi] - ray_origins[pi]) * d, axis=1)
        lo, hi = np.minimum(dA, dB), np.maximum(dA, dB)
        hit = np.where(lo > ORIGIN_EPS, lo, np.minimum(hi, maxr[pi]))
        ok = (hi > ORIGIN_EPS) & (lo <= maxr[pi]) & (hit <= maxr[pi])
        t[pi[ok], ni[ok]] = hit[ok]
    return t, ambiguous


def _ray_edge_distances(ray_origins, D, maxr, A, B):
    """Return ``(K, E)`` ray/edge distances and a ``(K,)`` ambiguity mask."""
    V = B - A
    AO = A - ray_origins[:, None, :]
    Dx, Dy = D[:, 0:1], D[:, 1:2]
    with np.errstate(all="ignore"):
        denom = Dx * V[..., 1] - Dy * V[..., 0]
        num_t = AO[..., 0] * V[..., 1] - AO[..., 1] * V[..., 0]
        num_u = AO[..., 0] * Dy - AO[..., 1] * Dx
        t = num_t / denom
        u = num_u / denom
    ambiguous = _ambiguous_edges(denom, num_t, t, u, maxr[:, None])
    valid = (denom != 0) & (u >= 0) & (u <= 1) & (t > ORIGIN_EPS) & (t <= maxr[:, None])
    t = np.where(valid, t, np.inf)
    coll = (denom == 0) & (np.abs(num_t) <= ORIGIN_EPS)
    if coll.any():
        ki, ei = np.nonzero(coll)
        dA = np.sum(AO[ki, ei] * D[ki], axis=1)
        dB = np.sum((B[ki, ei] - ray_origins[ki]) * D[ki], axis=1)
        lo, hi = np.minimum(dA, dB), np.maximum(dA, dB)
        hit = np.where(lo > ORIGIN_EPS, lo, np.minimum(hi, maxr[ki]))
        ok = (hi > ORIGIN_EPS) & (lo <= maxr[ki]) & (hit <= maxr[ki])
        t[ki[ok], ei[ok]] = hit[ok]
    return t, ambiguous


def _fold(best, best_obj, ps, t, obj_of_pair, uncertain):
    """Fold ``(pair x beam)`` distances into the per-sensor nearest hit.

    Args:
        best: Running ``(S, N)`` nearest distance.
        best_obj: Running ``(S, N)`` object index of the nearest hit, or
            ``None`` when hit objects are not needed.
        ps: ``(P,)`` sensor index of every pair, non-decreasing.
        t: ``(P, N)`` distances, ``inf`` where the pair's primitive is missed.
        obj_of_pair: ``(P,)`` object index of every pair's primitive.
    """
    if len(ps) == 0:
        return
    new_group = np.empty(len(ps), dtype=bool)
    new_group[0] = True
    np.not_equal(ps[1:], ps[:-1], out=new_group[1:])
    starts = np.flatnonzero(new_group)
    tmin = np.minimum.reduceat(t, starts, axis=0)  # (G, N)
    sensors = ps[starts]
    cur = best[sensors]
    if best_obj is None:
        best[sensors] = np.minimum(cur, tmin)
        return
    group = np.cumsum(new_group) - 1
    # STRtree order decides the reference target on ties. Recast those scans
    # instead of letting primitive order or the backend choose its velocity.
    tied = np.isfinite(t) & np.isclose(t, tmin[group], rtol=ROUND_OFF, atol=ROUND_OFF)
    ties = np.add.reduceat(tied.astype(int), starts, axis=0) > 1
    ties |= np.isfinite(tmin) & np.isclose(tmin, cur, rtol=ROUND_OFF, atol=ROUND_OFF)
    uncertain[sensors[np.any(ties, axis=1)]] = True
    better = tmin < cur
    if not better.any():
        return
    win = (t == tmin[group]) & better[group]
    pi, bi = np.nonzero(win)
    best[sensors] = np.minimum(cur, tmin)
    best_obj[ps[pi], bi] = obj_of_pair[pi]


class LidarBatchCaster:
    """Casts the beams of all ``Lidar2D`` sensors in one vectorized pass per step."""

    def __init__(self, analytic_circles: bool = False, backend: str = "auto") -> None:
        """
        Args:
            analytic_circles: Treat ``circle`` bodies as true circles instead
                of their buffer polygons. Faster, but not identical to the
                per-sensor scan (see the module docstring). Default ``False``.
            backend: ``"auto"`` uses the compiled kernel of
                :mod:`~irsim.lib.algorithm.lidar_batch_numba` when numba is
                installed and the NumPy kernels otherwise; ``"numpy"`` and
                ``"numba"`` force one of them (``"numba"`` raises without numba).
        """
        self.analytic_circles = analytic_circles
        if backend not in ("auto", "numpy", "numba"):
            raise ValueError(
                f"lidar_batch backend must be 'auto', 'numpy' or 'numba', got {backend!r}"
            )
        self._requested_backend = backend
        self._kernel = None
        self._backend = "numpy" if backend == "numpy" else None
        if backend == "numba":
            self._resolve_backend()  # fail early when numba is missing
        self._static_cache: dict[
            int, tuple
        ] = {}  # obj id -> (geometry, start, end, box)
        self._local_cache: dict[
            int, tuple
        ] = {}  # obj id -> (geometry, start, end) body frame
        self._circle_cache: dict[
            int, tuple
        ] = {}  # obj id -> (geometry, arrays or None)
        self._scene_key = None
        self._scene = None
        self._circle_key = None
        self._circles = None
        self._sensor_key = None
        self._sensor_const = None

    def _resolve_backend(self) -> None:
        """Pick the kernel on first use, so worlds without lidars never import numba."""
        if self._backend is not None:
            return
        try:
            from irsim.lib.algorithm import lidar_batch_numba
        except ImportError:
            lidar_batch_numba = None
        if lidar_batch_numba is not None and lidar_batch_numba.AVAILABLE:
            self._kernel = lidar_batch_numba.cast_kernel
            self._backend = "numba"
        elif self._requested_backend == "numba":
            raise ImportError(
                "lidar_batch 'numba' needs numba: pip install ir-sim[fast]"
            )
        else:
            self._backend = "numpy"

    @property
    def backend(self) -> str:
        """``"numba"`` or ``"numpy"``: the kernel in use (resolved on first query or cast)."""
        self._resolve_backend()
        return self._backend

    # ---------------------------------------------------------------- scene
    def _static_arrays(self, obj, geometry):
        key = geometry
        cached = self._static_cache.get(obj._id)
        if cached is None or cached[0] is not key:
            start, end = boundary_segments(geometry)
            start = np.ascontiguousarray(start, dtype=float)
            end = np.ascontiguousarray(end, dtype=float)
            cached = (key, start, end, _segment_bboxes(start, end))
            self._static_cache[obj._id] = cached
        return cached[1:]

    def _local_arrays(self, obj):
        original = obj.gf._original_geometry
        key = original
        cached = self._local_cache.get(obj._id)
        if cached is None or cached[0] is not key:
            start, end = boundary_segments(original)
            cached = (
                key,
                np.ascontiguousarray(start, dtype=float),
                np.ascontiguousarray(end, dtype=float),
            )
            self._local_cache[obj._id] = cached
        return cached[1:]

    def _circle_arrays(self, obj):
        """``(vertices sorted by angle, angles, local center, radius)`` of a regular
        buffer polygon, or ``None`` when the geometry is not inscribed in its
        circumscribed circle (then it is cast as a plain polygon)."""
        original = obj.gf._original_geometry
        key = original
        cached = self._circle_cache.get(obj._id)
        if cached is None or cached[0] is not key:
            if (
                original.geom_type != "Polygon"
                or original.is_empty
                or original.interiors
            ):
                self._circle_cache[obj._id] = (key, None)
                return None
            coords = np.asarray(original.exterior.coords, dtype=float)[:-1]
            center = coords.mean(axis=0)
            dist = np.hypot(*(coords - center).T)
            radius = float(dist.max())
            if radius > 0 and np.all(dist >= radius * (1 - _INSCRIBED_RTOL)):
                angles = np.arctan2(coords[:, 1] - center[1], coords[:, 0] - center[0])
                order = np.argsort(angles)
                angles = angles[order]
                gaps = np.diff(np.r_[angles, angles[0] + 2 * np.pi])
                regular = np.allclose(
                    gaps, 2 * np.pi / len(coords), rtol=_INSCRIBED_RTOL, atol=0
                )
                convex = original.equals(original.convex_hull)
                cached = (
                    key,
                    (np.ascontiguousarray(coords[order]), angles, center, radius)
                    if regular and convex
                    else None,
                )
            else:
                cached = (key, None)
            self._circle_cache[obj._id] = cached
        return cached[1]

    def _gather_scene(self, objects):
        """Segments ``(start, end, box, owner)`` or ``None``, and circle constants or ``None``."""
        static_key, static_items, dynamic, circle_key, circle_items = [], [], [], [], []
        for index, obj in enumerate(objects):
            if not getattr(obj, "_geometry_valid", False) or getattr(
                obj, "unobstructed", False
            ):
                continue
            shape = obj.shape
            if shape == "circle":
                arrays = self._circle_arrays(obj)
                if arrays is not None:
                    circle_key.append((index, obj._id, id(obj.gf._original_geometry)))
                    circle_items.append((obj, arrays))
                elif obj.static:
                    static_key.append((index, obj._id, id(obj._geometry)))
                    static_items.append((index, obj, obj._geometry))
                else:
                    dynamic.append((index, obj))
            elif shape == "map":
                static_key.append((index, obj._id, id(obj.geometry)))
                static_items.append((index, obj, obj.geometry))
            elif obj.static:
                static_key.append((index, obj._id, id(obj._geometry)))
                static_items.append((index, obj, obj._geometry))
            else:
                dynamic.append((index, obj))

        static_key = tuple(static_key)
        if static_key != self._scene_key:
            starts, ends, boxes, owners = [], [], [], []
            for index, obj, geometry in static_items:
                start, end, box = self._static_arrays(obj, geometry)
                if len(start):
                    starts.append(start)
                    ends.append(end)
                    boxes.append(box)
                    owners.append(np.full(len(start), index, dtype=int))
            self._scene = (
                (
                    np.concatenate(starts),
                    np.concatenate(ends),
                    np.concatenate(boxes),
                    np.concatenate(owners),
                )
                if starts
                else None
            )
            self._scene_key = static_key
        segments = self._scene
        if dynamic:
            starts, ends, owners = [], [], []
            for index, obj in dynamic:
                local_start, local_end = self._local_arrays(obj)
                if len(local_start):
                    state = np.asarray(obj.state, dtype=float).reshape(-1)[:3]
                    starts.append(_rotate_translate(local_start, state))
                    ends.append(_rotate_translate(local_end, state))
                    owners.append(np.full(len(local_start), index, dtype=int))
            if starts:
                start, end, owner = (
                    np.concatenate(starts),
                    np.concatenate(ends),
                    np.concatenate(owners),
                )
                box = _segment_bboxes(start, end)
                if segments is not None:
                    start, end = (
                        np.concatenate((segments[0], start)),
                        np.concatenate((segments[1], end)),
                    )
                    box, owner = (
                        np.concatenate((segments[2], box)),
                        np.concatenate((segments[3], owner)),
                    )
                segments = (start, end, box, owner)

        circle_key = tuple(circle_key)
        if circle_key != self._circle_key:
            if circle_items:
                uniform = len({len(a[0]) for _, a in circle_items}) == 1
                angles = np.stack([a[1] for _, a in circle_items]) if uniform else None
                shared = uniform and bool(np.all(angles == angles[0]))
                self._circles = {
                    "owner": np.array([k[0] for k in circle_key], dtype=int),
                    "objects": [obj for obj, _ in circle_items],
                    "center": np.array([a[2] for _, a in circle_items], dtype=float),
                    "radius": np.array([a[3] for _, a in circle_items], dtype=float),
                    "vertices": np.stack([a[0] for _, a in circle_items])
                    if uniform
                    else None,
                    "angles": angles[0] if shared else angles,
                    "angles_all": angles,
                    "shared_angles": shared,
                    "centered": bool(
                        np.all(np.array([a[2] for _, a in circle_items]) == 0)
                    ),
                    "local": [a for _, a in circle_items],
                }
            else:
                self._circles = None
            self._circle_key = circle_key
        return segments, self._circles

    # ----------------------------------------------------------------- rays
    def _gather_sensors(self, objects):
        """Plain ``Lidar2D`` sensors with owner index; per-sensor constants are cached."""
        from irsim.world.sensors.lidar2d import Lidar2D

        found = [
            (sensor, index)
            for index, obj in enumerate(objects)
            for sensor in getattr(obj, "sensors", ())
            if type(sensor) is Lidar2D
        ]
        if not found:
            return found, None
        key = tuple(
            (id(s), i, id(s._original_geometry), s.range_max, s.has_velocity)
            for s, i in found
        )
        if key != self._sensor_key:
            uniform = len({s.number for s, _ in found}) == 1
            self._sensor_const = {
                "uniform": uniform,
                "owner": np.array([i for _, i in found], dtype=int),
                "range": np.array([float(s.range_max) for s, _ in found]),
                "geometry": [s._original_geometry for s, _ in found],
                "coordinates": np.stack(
                    [shapely.get_coordinates(s._original_geometry) for s, _ in found]
                )
                if uniform
                else None,
                "track": any(getattr(s, "has_velocity", False) for s, _ in found),
            }
            self._sensor_key = key
        return found, self._sensor_const

    # -------------------------------------------------------------- kernels
    def _cast_segment_pairs(
        self, origins, D, ranges, owner, segments, best, best_obj, uncertain
    ):
        start, end, box, seg_owner = segments
        n_beams = D.shape[1]
        ox, oy, r = origins[:, 0:1], origins[:, 1:2], ranges[:, None]
        near = (
            (box[None, :, 0] <= ox + r)
            & (box[None, :, 1] >= ox - r)
            & (box[None, :, 2] <= oy + r)
            & (box[None, :, 3] >= oy - r)
            & (seg_owner[None, :] != owner[:, None])
        )
        si, sj = np.nonzero(near)
        if len(si) == 0:
            return
        block = max(1, MAX_BLOCK_ELEMENTS // n_beams)
        for p0 in range(0, len(si), block):
            ps, pj = si[p0 : p0 + block], sj[p0 : p0 + block]
            t, ambiguous = _pair_segment_distances(
                origins[ps], D[ps], ranges[ps], start[pj], end[pj]
            )
            uncertain[ps[ambiguous]] = True
            _fold(best, best_obj, ps, t, seg_owner[pj], uncertain)

    def _cast_circle_pairs(
        self, origins, D, ranges, owner, circles, best, best_obj, uncertain
    ):
        n_beams = D.shape[1]
        c_owner, radius, local_center = (
            circles["owner"],
            circles["radius"],
            circles["center"],
        )
        states = np.array(
            [
                np.asarray(o.state, dtype=float).reshape(-1)[:3]
                for o in circles["objects"]
            ]
        )
        theta = states[:, 2]
        ct, st = np.cos(theta), np.sin(theta)
        if circles["centered"]:
            centers = states[:, :2]
        else:
            centers = np.column_stack(
                (
                    states[:, 0] + ct * local_center[:, 0] - st * local_center[:, 1],
                    states[:, 1] + st * local_center[:, 0] + ct * local_center[:, 1],
                )
            )
        dx = centers[None, :, 0] - origins[:, 0:1]
        dy = centers[None, :, 1] - origins[:, 1:2]
        reach = ranges[:, None] + radius[None, :]
        reach += ROUND_OFF * np.maximum(1.0, reach)
        near = (dx * dx + dy * dy <= reach**2) & (c_owner[None, :] != owner[:, None])
        si, cj = np.nonzero(near)
        if len(si) == 0:
            return
        exact = not self.analytic_circles
        if exact:
            vertices = circles["vertices"]
            if vertices is None:
                # Mixed polygon resolutions: cast their edges as plain segments.
                starts = [
                    _rotate_translate(a[0], s)
                    for a, s in zip(circles["local"], states, strict=True)
                ]
                ends = [np.roll(s, -1, axis=0) for s in starts]
                start, end = np.concatenate(starts), np.concatenate(ends)
                seg = (
                    start,
                    end,
                    _segment_bboxes(start, end),
                    np.repeat(c_owner, [len(s) for s in starts]),
                )
                self._cast_segment_pairs(
                    origins, D, ranges, owner, seg, best, best_obj, uncertain
                )
                return
            M = vertices.shape[1]
            angles, shared = circles["angles"], circles["shared_angles"]
            c3, s3 = ct[:, None], st[:, None]
            world = np.stack(
                (
                    vertices[..., 0] * c3 - vertices[..., 1] * s3 + states[:, 0:1],
                    vertices[..., 0] * s3 + vertices[..., 1] * c3 + states[:, 1:2],
                ),
                axis=-1,
            )
        block = max(1, MAX_BLOCK_ELEMENTS // n_beams)
        for p0 in range(0, len(si), block):
            ps, pc = si[p0 : p0 + block], cj[p0 : p0 + block]
            ray_origins, Dp, mr = origins[ps], D[ps], ranges[ps]  # (P,2) (P,N,2) (P,)
            oc = ray_origins - centers[pc]  # (P, 2)
            b = oc[:, 0:1] * Dp[..., 0] + oc[:, 1:2] * Dp[..., 1]  # (P, N)
            c = (oc[:, 0] ** 2 + oc[:, 1] ** 2 - radius[pc] ** 2)[:, None]
            disc = b * b - c
            if exact:
                tol = ROUND_OFF * np.maximum(1.0, b * b + np.abs(c))
                uncertain[ps[np.any(np.abs(disc) <= tol, axis=1)]] = True
            sq = np.sqrt(np.maximum(disc, 0.0))
            t_in, t_out = -b - sq, -b + sq
            if exact:
                at_limit = np.abs(t_in - mr[:, None]) <= ROUND_OFF * np.maximum(
                    1.0, mr[:, None]
                )
                uncertain[ps[np.any(at_limit, axis=1)]] = True
            touches = (disc >= 0) & (t_out > ORIGIN_EPS) & (t_in <= mr[:, None])
            if not exact:
                t = np.where(t_in > ORIGIN_EPS, t_in, t_out)
                t = np.where(touches & (t > ORIGIN_EPS) & (t <= mr[:, None]), t, np.inf)
                _fold(best, best_obj, ps, t, c_owner[pc], uncertain)
                continue
            kp, kb = np.nonzero(touches)
            if len(kp) == 0:
                continue
            t_dense = np.full((len(ps), n_beams), np.inf)
            circ, Ok, Dk, mk, tin = (
                pc[kp],
                ray_origins[kp],
                Dp[kp, kb],
                mr[kp],
                t_in[kp, kb],
            )
            inside = tin <= ORIGIN_EPS
            out = ~inside
            if out.any():
                ko, Oo, Do, mo = circ[out], Ok[out], Dk[out], mk[out]
                p = Oo + tin[out][:, None] * Do
                phi = (
                    np.arctan2(p[:, 1] - centers[ko, 1], p[:, 0] - centers[ko, 0])
                    - theta[ko]
                )
                phi = (phi + np.pi) % (2 * np.pi) - np.pi
                if shared:
                    j = np.searchsorted(angles, phi, side="right") - 1
                else:
                    j = np.count_nonzero(angles[ko] <= phi[:, None], axis=1) - 1
                idx = (j[:, None] + _NEIGHBOURS[None, :]) % M
                t, ambiguous = _ray_edge_distances(
                    Oo,
                    Do,
                    mo,
                    world[ko[:, None], idx],
                    world[ko[:, None], (idx + 1) % M],
                )
                uncertain[ps[kp[out][ambiguous]]] = True
                t_dense[kp[out], kb[out]] = t.min(axis=1)
            if inside.any():
                ki = circ[inside]
                A = world[ki]
                t, ambiguous = _ray_edge_distances(
                    Ok[inside], Dk[inside], mk[inside], A, np.roll(A, -1, axis=1)
                )
                uncertain[ps[kp[inside][ambiguous]]] = True
                t_dense[kp[inside], kb[inside]] = t.min(axis=1)
            _fold(best, best_obj, ps, t_dense, c_owner[pc], uncertain)

    # ------------------------------------------------------------------ step
    def _cast(self, objects, origins, D, ranges, owner, track):
        """Nearest hit per beam for a sensor set.

        Returns:
            Ranges ``(S, N)``, hit object indices ``(S, N)`` (or ``None``
            without velocity tracking), and a reference-fallback mask ``(S,)``.
        """
        n_sensors, n_beams = D.shape[:2]
        best = np.full((n_sensors, n_beams), np.inf)
        uncertain = np.zeros(n_sensors, dtype=bool)
        segments, circles = self._gather_scene(objects)
        self._resolve_backend()
        if self._kernel is not None:
            best_obj = self._cast_compiled(
                origins, D, ranges, owner, segments, circles, best, uncertain
            )
            if not track:
                best_obj = None
        else:
            best_obj = np.full((n_sensors, n_beams), -1, dtype=int) if track else None
            if segments is not None:
                self._cast_segment_pairs(
                    origins, D, ranges, owner, segments, best, best_obj, uncertain
                )
            if circles is not None:
                self._cast_circle_pairs(
                    origins, D, ranges, owner, circles, best, best_obj, uncertain
                )
        np.minimum(
            best, ranges[:, None], out=best
        )  # misses (inf) read the maximum range
        return best, best_obj, uncertain

    def _cast_compiled(
        self, origins, D, ranges, owner, segments, circles, best, uncertain
    ):
        """Run the numba kernel on the gathered scene; returns the ``(S, N)`` hit objects."""
        best_obj = np.full(best.shape, -1, dtype=np.int64)
        if circles is not None and circles["vertices"] is None:
            # Mixed polygon resolutions: cast the circle polygons as plain segments.
            states = [
                np.asarray(o.state, dtype=float).reshape(-1)[:3]
                for o in circles["objects"]
            ]
            starts = [
                _rotate_translate(a[0], st)
                for a, st in zip(circles["local"], states, strict=True)
            ]
            ends = [np.roll(st, -1, axis=0) for st in starts]
            start, end = np.concatenate(starts), np.concatenate(ends)
            extra = (
                start,
                end,
                _segment_bboxes(start, end),
                np.repeat(circles["owner"], [len(st) for st in starts]),
            )
            segments = (
                extra
                if segments is None
                else tuple(
                    np.concatenate((a, b)) for a, b in zip(segments, extra, strict=True)
                )
            )
            circles = None
        if segments is None:
            segments = _EMPTY_SEGMENTS
        if circles is None:
            c_pos, c_theta, c_lc, c_radius, c_owner, c_local, c_angles = _EMPTY_CIRCLES
        else:
            states = np.array(
                [
                    np.asarray(o.state, dtype=float).reshape(-1)[:3]
                    for o in circles["objects"]
                ]
            )
            c_pos, c_theta = (
                np.ascontiguousarray(states[:, :2]),
                np.ascontiguousarray(states[:, 2]),
            )
            c_lc, c_radius, c_owner = (
                circles["center"],
                circles["radius"],
                circles["owner"],
            )
            c_local, c_angles = circles["vertices"], circles["angles_all"]
        self._kernel(
            origins,
            D,
            ranges,
            owner,
            segments[0],
            segments[1],
            segments[2],
            segments[3],
            c_pos,
            c_theta,
            c_lc,
            c_radius,
            c_owner,
            c_local,
            c_angles,
            self.analytic_circles,
            ORIGIN_EPS,
            ROUND_OFF,
            best,
            best_obj,
            uncertain,
        )
        return best_obj

    def step(self, objects, *, step_other_sensors=False) -> set[int]:
        """Cast plain lidars, then apply scans in object/sensor order.

        Environments set ``step_other_sensors`` to also step custom/FMCW sensors
        in that same order, preserving seeded noise and sensor dependencies.
        Numerically ambiguous scans use the reference sensor step.
        """
        found, const = self._gather_sensors(objects)
        scans = {}
        if found:
            if const["uniform"]:
                scans = self._scan_uniform(found, const, objects)
            else:
                for sensor, index in found:
                    local = dict(const)
                    local["coordinates"] = shapely.get_coordinates(
                        sensor._original_geometry
                    )[None]
                    local["range"] = np.array([float(sensor.range_max)])
                    local["owner"] = np.array([index])
                    local["track"] = sensor.has_velocity
                    scans.update(self._scan_uniform([(sensor, index)], local, objects))
        for obj in objects:
            for sensor in obj.sensors:
                if id(sensor) in scans:
                    result, uncertain = scans[id(sensor)]
                    if uncertain and not self.analytic_circles:
                        sensor.step(obj.state[0:3])
                    else:
                        sensor.apply_batch_result(*result, objects)
                elif step_other_sensors:
                    sensor.step(obj.state[0:3])
        return set(scans)

    def _scan_uniform(self, found, const, objects):
        """Cast equal-sized scans without updating sensors or drawing noise."""
        states = np.array(
            [
                np.asarray(objects[i].state, dtype=float).reshape(-1)[:3]
                for _, i in found
            ]
        )
        coords = const["coordinates"]
        ct, st = np.cos(states[:, 2:3]), np.sin(states[:, 2:3])
        px, py = coords[..., 0], coords[..., 1]
        world = np.stack(
            (px * ct - py * st + states[:, 0:1], px * st + py * ct + states[:, 1:2]),
            axis=-1,
        )
        origins = world[:, 0]
        D = (world[:, 1::2] - origins[:, None]) / const["range"][:, None, None]
        base = states[:, 2] + np.array([sensor.offset[2, 0] for sensor, _ in found])
        result, best_obj, uncertain = self._cast(
            objects, origins, D, const["range"], const["owner"], const["track"]
        )
        return {
            id(sensor): (
                (
                    objects[index].state[0:3],
                    origins[k],
                    base[k],
                    D[k],
                    result[k],
                    None if best_obj is None else best_obj[k],
                ),
                uncertain[k],
            )
            for k, (sensor, index) in enumerate(found)
        }
