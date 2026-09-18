"""
Mass-based contact resolution for the ``contact`` collision mode.

Objects that overlap after a kinematic step are separated along their
contact normal, and the separation is shared in inverse proportion to their
masses, as in a position-based, perfectly inelastic rigid-body contact. A
robot pushing a box therefore moves the box, a heavy box slows the robot
down, and an immovable object (infinite mass) stops it while still letting
it slide along the surface. Bodies translate only: there is no rotational
response and no coasting, so an object stops as soon as nothing pushes it.

The contact normal and depth come from the separating axis theorem (SAT)
evaluated between convex pieces of the two shapes: circles, convex
polygons, and line segments. Non-convex polygons are triangulated with a
constrained Delaunay triangulation, so every shape IR-SIM supports takes
part, including compound bodies, linestring walls, and grid maps.

Author: Ruihua Han
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import shapely
from shapely.geometry import LineString, Polygon
from shapely.geometry.base import BaseGeometry

# Gap left between two bodies after a contact is resolved, so that shapes that
# were just separated do not register as intersecting (colliding) again.
CONTACT_SLOP = 1e-6
# Maximum Gauss-Seidel sweeps over the contact pairs per step. Sweeps stop as
# soon as nothing overlaps; a chain anchored on an immovable object (robot
# pushing a box into a wall) halves its residual every sweep, so a 0.1 m step
# settles below the slop in about twenty sweeps.
CONTACT_ITERATIONS = 32

# Map boundary segments this close to an object's bounding box take part.
_MAP_QUERY_MARGIN = 1e-3

_AREA_TOL = 1e-9
_EPS = 1e-12
_NO_AXES = np.zeros((0, 2))

# A convex piece is ``("circle", center (2,), radius)`` or
# ``("polygon", vertices (N, 2)[, edge normals (K, 2)])``; a line segment is a
# two-vertex polygon. The normals are optional and translation-invariant, so
# pieces built by this module carry them to spare the sweeps recomputing them.
Piece = tuple[str, Any, ...]


@dataclass(slots=True)
class Contact:
    """One resolved contact between two objects.

    Attributes:
        a: First object; ``normal`` points from ``b`` toward it.
        b: Second object.
        normal: Unit contact normal ``(2,)`` pointing from ``b`` to ``a``.
        depth: Penetration depth that was removed, in meters.
    """

    a: Any
    b: Any
    normal: np.ndarray
    depth: float


# ---------------------------------------------------------------------------
# Convex decomposition
# ---------------------------------------------------------------------------


def _is_convex(polygon: Polygon) -> bool:
    return abs(polygon.convex_hull.area - polygon.area) <= _AREA_TOL * max(
        polygon.area, 1.0
    )


def _exterior_vertices(polygon: Polygon) -> np.ndarray:
    return np.asarray(polygon.exterior.coords, dtype=float)[:-1]


def _polygon_normals(vertices: np.ndarray) -> np.ndarray:
    """Unit edge normals of a convex polygon; one for a two-vertex segment."""
    edges = np.roll(vertices, -1, axis=0) - vertices
    if vertices.shape[0] == 2:
        edges = edges[:1]
    normals = np.stack([edges[:, 1], -edges[:, 0]], axis=1)
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > _EPS
    return normals[valid] / lengths[valid, None]


def _polygon_piece(vertices: np.ndarray) -> Piece:
    return ("polygon", vertices, _polygon_normals(vertices))


def _polygon_pieces(polygon: Polygon, convex: bool | None = None) -> list[Piece]:
    if polygon.is_empty:
        return []
    if convex is None:
        convex = not polygon.interiors and _is_convex(polygon)
    if convex:
        return [_polygon_piece(_exterior_vertices(polygon))]
    triangles = shapely.constrained_delaunay_triangles(polygon)
    return [
        _polygon_piece(_exterior_vertices(triangle))
        for triangle in getattr(triangles, "geoms", [triangles])
        if isinstance(triangle, Polygon) and not triangle.is_empty
    ]


def _line_pieces(line: LineString, bounds: tuple | None = None) -> list[Piece]:
    coords = np.asarray(line.coords, dtype=float)
    if coords.shape[0] < 2:
        return []
    starts, ends = coords[:-1], coords[1:]
    if bounds is not None:
        minx, miny, maxx, maxy = bounds
        lo, hi = np.minimum(starts, ends), np.maximum(starts, ends)
        keep = (
            (hi[:, 0] >= minx)
            & (lo[:, 0] <= maxx)
            & (hi[:, 1] >= miny)
            & (lo[:, 1] <= maxy)
        )
        starts, ends = starts[keep], ends[keep]
    edges = ends - starts
    normals = np.stack([edges[:, 1], -edges[:, 0]], axis=1)
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > _EPS
    normals = np.where(
        valid[:, None], normals / np.where(valid, lengths, 1.0)[:, None], 0.0
    )
    return [
        ("polygon", np.array([start, end]), normal[None, :] if ok else _NO_AXES)
        for start, end, normal, ok in zip(starts, ends, normals, valid, strict=True)
    ]


def geometry_pieces(geometry: BaseGeometry, bounds: tuple | None = None) -> list[Piece]:
    """Decompose a shapely geometry into convex pieces.

    Polygons become themselves when convex, otherwise their constrained
    Delaunay triangles. Lines become their segments, restricted to those whose
    bounding box meets ``bounds`` when given. Points contribute nothing.

    Args:
        geometry: Any shapely geometry or collection.
        bounds: Optional ``(minx, miny, maxx, maxy)`` filter for line segments.

    Returns:
        list: Convex pieces as ``("polygon", vertices (N, 2))`` tuples.
    """
    pieces: list[Piece] = []
    for part in getattr(geometry, "geoms", [geometry]):
        if isinstance(part, Polygon):
            pieces.extend(_polygon_pieces(part))
        elif isinstance(part, LineString):
            pieces.extend(_line_pieces(part, bounds))
        elif hasattr(part, "geoms"):
            pieces.extend(geometry_pieces(part, bounds))
    return pieces


def object_pieces(obj: Any, other: Any | None = None) -> list[Piece]:
    """Convex pieces of an object at its current pose.

    A ``circle`` is kept exact. A ``compound`` uses its parts. A ``map`` only
    contributes the boundary segments near ``other`` (the object it is tested
    against), found through its own spatial index.

    Args:
        obj: The object to decompose.
        other: The object it is in contact with, used to restrict map segments.

    Returns:
        list: Convex pieces.
    """
    if obj.shape == "circle":
        center = np.asarray(obj.centroid, dtype=float).reshape(-1)[:2]
        return [("circle", center, float(obj.radius))]

    if obj.shape == "map":
        if other is None:
            return geometry_pieces(obj.geometry)
        return _map_pieces_near(obj, other, _MAP_QUERY_MARGIN)

    part_geometries = getattr(obj.gf, "part_geometries", None)
    if part_geometries:
        return [piece for part in part_geometries for piece in geometry_pieces(part)]

    vertices = obj.vertices
    if (
        obj.shape in {"polygon", "rectangle"}
        and obj.convex_flag
        and vertices is not None
        and vertices.shape[1] >= 3
    ):
        # the object already knows it is convex: skip the hull test
        return [_polygon_piece(np.ascontiguousarray(vertices.T, dtype=float))]

    return geometry_pieces(obj.geometry)


# ---------------------------------------------------------------------------
# Separating axis theorem
# ---------------------------------------------------------------------------


def _axes(piece: Piece, other: Piece) -> np.ndarray:
    """Candidate separating axes contributed by ``piece`` against ``other``."""
    if piece[0] == "polygon":
        if len(piece) > 2:
            return piece[2]
        return _polygon_normals(piece[1])

    center = piece[1]
    if other[0] == "circle":
        direction = center - other[1]
    else:
        vertices = other[1]
        closest = vertices[np.argmin(np.linalg.norm(vertices - center, axis=1))]
        direction = center - closest
    length = np.linalg.norm(direction)
    if length <= _EPS:
        return np.zeros((0, 2))
    return direction[None, :] / length


def _project(piece: Piece, axes: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Projection intervals ``(low, high)`` of ``piece`` on each axis."""
    if piece[0] == "circle":
        center, radius = piece[1], piece[2]
        projection = axes @ center
        return projection - radius, projection + radius
    projection = piece[1] @ axes.T
    return projection.min(axis=0), projection.max(axis=0)


def piece_mtv(a: Piece, b: Piece) -> tuple[np.ndarray, float] | None:
    """Minimum translation that separates convex piece ``a`` from ``b``.

    Args:
        a: Convex piece to move.
        b: Convex piece it overlaps.

    Returns:
        tuple | None: ``(normal, depth)`` where the unit ``normal`` points from
        ``b`` toward ``a`` and moving ``a`` by ``normal * depth`` separates the
        pieces, or ``None`` when they do not overlap. Exactly touching pieces
        count as a zero-depth contact, since shapely reports them as
        intersecting.
    """
    if a[0] == "circle" and b[0] == "circle":
        offset = a[1] - b[1]
        distance = float(np.hypot(offset[0], offset[1]))
        depth = a[2] + b[2] - distance
        if depth < 0:
            return None
        if distance <= _EPS:
            return np.array([1.0, 0.0]), depth
        return offset / distance, depth

    axes = np.concatenate([_axes(a, b), _axes(b, a)], axis=0)
    if axes.shape[0] == 0:
        return None

    low_a, high_a = _project(a, axes)
    low_b, high_b = _project(b, axes)
    # Distance ``a`` must travel along -axis (its top past b's bottom) or along
    # +axis (its bottom past b's top) to clear ``b``. Unlike the length of the
    # interval intersection this is right when one interval contains the
    # other, e.g. a circle straddling a zero-width wall segment.
    push_down = high_a - low_b
    push_up = high_b - low_a
    if np.any(push_down < 0) or np.any(push_up < 0):
        return None

    depths = np.minimum(push_down, push_up)
    best = int(np.argmin(depths))
    normal = axes[best] if push_up[best] <= push_down[best] else -axes[best]
    return normal, float(depths[best])


def object_mtv(
    a: Any, b: Any, cache: dict[Any, list[Piece]] | None = None
) -> tuple[np.ndarray, float] | None:
    """Deepest minimum translation between two objects.

    Every convex piece of ``a`` is tested against every piece of ``b``, and the
    deepest overlap wins; shallower overlaps with other pieces are resolved by
    later sweeps or steps.

    Args:
        a: Object to move along the returned normal.
        b: Object it overlaps.
        cache: Optional pieces cache keyed by ``id(obj)``, shared across the
            sweeps of one resolution; map pieces depend on the partner and
            are never cached.

    Returns:
        tuple | None: ``(normal, depth)`` with the normal pointing from ``b``
        to ``a``, or ``None`` when no pieces overlap.
    """
    best: tuple[np.ndarray, float] | None = None
    for piece_a in _cached_pieces(a, b, cache):
        for piece_b in _cached_pieces(b, a, cache):
            mtv = piece_mtv(piece_a, piece_b)
            if mtv is not None and (best is None or mtv[1] > best[1]):
                best = mtv
    return best


def _cached_pieces(
    obj: Any, other: Any, cache: dict[Any, list[Piece]] | None
) -> list[Piece]:
    if cache is None or obj.shape == "map":
        return object_pieces(obj, other)
    key = id(obj)
    if key not in cache:
        cache[key] = object_pieces(obj, other)
    return cache[key]


# ---------------------------------------------------------------------------
# Resolution
# ---------------------------------------------------------------------------


def resolve_contacts(
    pairs: list[tuple[Any, Any]],
    iterations: int = CONTACT_ITERATIONS,
    slop: float = CONTACT_SLOP,
    margin: float = 0.0,
) -> list[Contact]:
    """Separate overlapping object pairs, sharing the correction by mass.

    For each overlapping pair the penetration is removed along the contact
    normal. Object ``a`` moves by ``depth * w_a / (w_a + w_b)`` and ``b`` by
    the rest, where ``w`` is the inverse mass (zero for static or
    infinite-mass objects, which never move).

    Pairs are swept in order of their distance from immovable objects, and an
    object that was just pressed against something it cannot move is treated
    as blocked in that direction for the rest of the sweep: it only responds
    tangentially, and its partner takes the remaining correction. A robot
    pushing a box into a wall therefore settles in a single sweep, and a box
    pushed obliquely into a wall slides along it. The sweep repeats until no
    pair overlaps or ``iterations`` is reached, so free chains settle too.

    The sweeps work on translated copies of each object's convex pieces and
    accumulate one displacement per object, which is applied once at the end
    through
    :py:meth:`~irsim.world.object_base.ObjectBase.apply_contact_displacement`;
    that call also folds the displacement into the object's velocity. Both
    objects of every resolved pair are marked as in contact with each other.

    Args:
        pairs: Candidate ``(a, b)`` object pairs. They should include every
            pair that a correction could bring into contact, e.g. all pairs
            within one step's travel of each other, not only the pairs that
            intersect after the kinematic step; pairs that never overlap are
            skipped.
        iterations: Maximum sweeps over the pairs.
        slop: Extra gap left between separated bodies.
        margin: How far an object may be moved by the sweeps, used to fetch
            enough grid-map boundary around each candidate up front.

    Returns:
        list[Contact]: The contacts resolved in the first sweep that touched
        each pair, in sweep order.
    """
    contacts: list[Contact] = []
    seen: set[tuple[int, int]] = set()
    sweep = _SweepState(margin)
    # (a, b, key_a, key_b, w_a, w_b) for every pair that something can move
    order = [
        (a, b, id(a), id(b), w_a, w_b)
        for a, b in _anchored_order(pairs)
        if (w_a := a.inv_mass) + (w_b := b.inv_mass) > 0
    ]

    for _ in range(iterations):
        blocked: dict[int, list[np.ndarray]] = {}
        moved = False
        for a, b, key_a, key_b, w_a, w_b in order:
            mtv = sweep.mtv(a, b)
            if mtv is None:
                continue
            normal, depth = mtv
            depth += slop

            # directions each body can actually move in, and how much of the
            # separation that motion buys along the normal
            if blocked:
                move_a, gain_a = _free_motion(blocked.get(key_a), normal)
                move_b, gain_b = _free_motion(blocked.get(key_b), -normal)
            else:
                move_a, gain_a, move_b, gain_b = normal, 1.0, -normal, 1.0
            eff_a, eff_b = w_a * gain_a, w_b * gain_b
            total = eff_a + eff_b
            if total <= 0:
                continue  # squeezed between blockers; the status check reports it

            scale = depth / total
            if eff_a > 0:
                sweep.move(key_a, scale * w_a * move_a)
            if eff_b > 0:
                sweep.move(key_b, scale * w_b * move_b)
            moved = True

            # whoever could not yield now blocks its partner along the normal
            if eff_b <= 0 and w_a > 0:
                blocked.setdefault(key_a, []).append(-normal)
            if eff_a <= 0 and w_b > 0:
                blocked.setdefault(key_b, []).append(normal)

            key = (key_a, key_b)
            if key not in seen:
                seen.add(key)
                contacts.append(Contact(a, b, normal, depth))
                _mark_contact(a, b)

        if not moved:
            break

    sweep.apply()
    return contacts


class _SweepState:
    """Pieces and accumulated displacements of the objects in one resolution.

    Objects are not touched while sweeping: their pieces are decomposed once
    and translated with numpy as displacements accumulate, and the total is
    applied to each moved object when the sweeps are over.
    """

    def __init__(self, margin: float) -> None:
        self.margin = margin
        self.objects: dict[int, Any] = {}
        self.base: dict[int, list[Piece]] = {}
        self.current: dict[int, list[Piece]] = {}
        self.offset: dict[int, np.ndarray] = {}
        # map boundary segments near a partner, with their bounding boxes
        self.map_segments: dict[
            tuple[int, int], tuple[list[Piece], np.ndarray, np.ndarray]
        ] = {}

    def pieces(self, obj: Any, other: Any) -> list[Piece]:
        """Current pieces of ``obj``; for a map, the segments near ``other``."""
        key = id(obj)
        current = self.current.get(key)
        if current is not None:
            return current
        if obj.shape == "map":
            segments, lo, hi = self._map_segments(obj, other)
            if not segments:
                return segments
            box_lo, box_hi = _pieces_bounds(self.pieces(other, obj))
            near = np.flatnonzero(
                np.all(hi >= box_lo - _MAP_QUERY_MARGIN, axis=1)
                & np.all(lo <= box_hi + _MAP_QUERY_MARGIN, axis=1)
            )
            return [segments[i] for i in near]
        self.objects[key] = obj
        self.base[key] = object_pieces(obj)
        self.current[key] = self.base[key]
        return self.current[key]

    def _map_segments(
        self, grid: Any, other: Any
    ) -> tuple[list[Piece], np.ndarray, np.ndarray]:
        map_key = (id(grid), id(other))
        if map_key not in self.map_segments:
            segments = _map_pieces_near(grid, other, self.margin + _MAP_QUERY_MARGIN)
            if segments:
                vertices = np.stack([piece[1] for piece in segments])
                lo, hi = vertices.min(axis=1), vertices.max(axis=1)
            else:
                lo = hi = np.zeros((0, 2))
            self.map_segments[map_key] = (segments, lo, hi)
        return self.map_segments[map_key]

    def mtv(self, a: Any, b: Any) -> tuple[np.ndarray, float] | None:
        pieces_a = self.pieces(a, b)
        pieces_b = self.pieces(b, a)
        if len(pieces_a) == 1 and len(pieces_b) == 1:
            return piece_mtv(pieces_a[0], pieces_b[0])
        best: tuple[np.ndarray, float] | None = None
        for piece_a in pieces_a:
            for piece_b in pieces_b:
                mtv = piece_mtv(piece_a, piece_b)
                if mtv is not None and (best is None or mtv[1] > best[1]):
                    best = mtv
        return best

    def move(self, key: int, delta: np.ndarray) -> None:
        total = self.offset.get(key)
        total = delta if total is None else total + delta
        self.offset[key] = total
        self.current[key] = [_translate(piece, total) for piece in self.base[key]]

    def apply(self) -> None:
        for key, delta in self.offset.items():
            self.objects[key].apply_contact_displacement(delta)


def _translate(piece: Piece, delta: np.ndarray) -> Piece:
    if piece[0] == "circle":
        return ("circle", piece[1] + delta, piece[2])
    return ("polygon", piece[1] + delta, *piece[2:])


def _pieces_bounds(pieces: list[Piece]) -> tuple[np.ndarray, np.ndarray]:
    """Axis-aligned bounds ``(lo, hi)`` of a list of pieces."""
    lo = np.full(2, np.inf)
    hi = np.full(2, -np.inf)
    for piece in pieces:
        if piece[0] == "circle":
            lo = np.minimum(lo, piece[1] - piece[2])
            hi = np.maximum(hi, piece[1] + piece[2])
        else:
            lo = np.minimum(lo, piece[1].min(axis=0))
            hi = np.maximum(hi, piece[1].max(axis=0))
    return lo, hi


def _map_pieces_near(grid: Any, other: Any, margin: float) -> list[Piece]:
    minx, miny, maxx, maxy = other.geometry.bounds
    bounds = (minx - margin, miny - margin, maxx + margin, maxy + margin)
    indices = grid.geometry_tree.query(shapely.box(*bounds))
    return [
        piece
        for index in np.sort(indices)
        for piece in _line_pieces(grid.linestrings[index], bounds)
    ]


def _free_motion(
    blocks: list[np.ndarray] | None, direction: np.ndarray
) -> tuple[np.ndarray, float]:
    """Project a wanted move onto what the object's blockers allow.

    Returns the allowed motion (the tangential part along the most opposing
    blocker, or ``direction`` itself when nothing blocks it) and its
    component along ``direction``, which is the separation one unit of that
    motion produces.
    """
    if not blocks:
        return direction, 1.0
    into = [float(direction @ u) for u in blocks]
    worst = int(np.argmax(into))
    if into[worst] <= 0:
        return direction, 1.0
    motion = direction - into[worst] * blocks[worst]
    gain = float(motion @ direction)
    if gain <= _EPS:
        return np.zeros(2), 0.0
    return motion, gain


def _anchored_order(pairs: list[tuple[Any, Any]]) -> list[tuple[Any, Any]]:
    """Sort pairs by their contact-graph distance from immovable objects."""
    adjacency: dict[int, set[int]] = {}
    objects: dict[int, Any] = {}
    for a, b in pairs:
        objects[id(a)] = a
        objects[id(b)] = b
        adjacency.setdefault(id(a), set()).add(id(b))
        adjacency.setdefault(id(b), set()).add(id(a))

    level = {key: 0 for key, obj in objects.items() if obj.inv_mass <= 0}
    frontier = list(level)
    while frontier:
        nxt = []
        for key in frontier:
            for other in adjacency[key]:
                if other not in level:
                    level[other] = level[key] + 1
                    nxt.append(other)
        frontier = nxt

    unreached = len(objects) + 1

    def rank(pair: tuple[Any, Any]) -> tuple[int, int]:
        la = level.get(id(pair[0]), unreached)
        lb = level.get(id(pair[1]), unreached)
        return min(la, lb), max(la, lb)

    return sorted(pairs, key=rank)


def _mark_contact(a: Any, b: Any) -> None:
    for obj, other in ((a, b), (b, a)):
        obj.contact_flag = True
        if other not in obj.contact_obj:
            obj.contact_obj.append(other)
