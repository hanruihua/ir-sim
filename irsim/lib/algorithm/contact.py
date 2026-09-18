"""
Rigid-body contact resolution for the ``contact`` collision mode.

Objects that overlap after a kinematic step are separated along their
contact normal. Between two passive bodies, or two driven ones, the
separation is shared in inverse proportion to their masses, which is the
perfectly inelastic collision of position-based dynamics. A driven object
(one with kinematics) pushes a passive body without yielding to it, as a
velocity-controlled robot in a physics engine does, provided its traction,
``friction * mass``, is at least the body's ground friction; a body it
cannot overcome stalls it. An immovable object (static or infinite mass)
stops whatever presses on it, which then slides along the surface: a robot
always, a passive body only when the push leaves the friction cone of the
two surfaces. Passive bodies keep the velocity a push gave them and coast to
a stop under ground friction, which their kinematics model integrates.
A push that misses a body's center also turns it: the contact point and
normal give a torque, shared through each body's moment of inertia as in
position-based rigid-body dynamics, and a spinning passive body slows under
friction like a sliding one. Driven objects keep their heading, which their
drive holds.

The contact normal and depth come from the separating axis theorem (SAT)
evaluated between convex pieces of the two shapes: circles, convex
polygons, and line segments. Non-convex polygons are triangulated with a
constrained Delaunay triangulation, so every shape IR-SIM supports takes
part, including compound bodies, linestring walls, and grid maps.

The resolution follows position-based dynamics: each overlap is a
unilateral distance constraint, projected one pair at a time in
Gauss-Seidel sweeps with the correction weighted by inverse mass, and the
velocity of a body is recovered from its position change. Pairs are swept
outward from immovable objects, and a body that was just pressed against
something it cannot move only yields tangentially for the rest of the
sweep, a simplified form of shock propagation, so that a chain pushed into
a wall settles in one sweep and slides along it.

References:

    Müller, M., Heidelberger, B., Hennix, M., Ratcliff, J. (2007),
    *Position based dynamics.* J. Vis. Commun. Image Represent. 18(2):109.
    Mass-weighted constraint projection, Gauss-Seidel iteration over the
    constraints, and velocities derived from the position update.

    Macklin, M., Müller, M., Chentanez, N. (2016), *XPBD: position-based
    simulation of compliant constrained dynamics.* Proc. Motion in Games,
    p. 49. Müller, M., Macklin, M., Chentanez, N., Jeschke, S., Kim, T.-Y.
    (2020), *Detailed rigid body simulation with extended position based
    dynamics.* Comput. Graph. Forum 39(8):101. The rigid-body form of the
    same projection.

    Guendelman, E., Bridson, R., Fedkiw, R. (2003), *Nonconvex rigid bodies
    with stacking.* ACM Trans. Graph. 22(3):871. Shock propagation: contacts
    are processed outward from immovable objects and settled bodies are
    treated as immovable, which the anchored sweep order and the blocked
    directions here simplify to a single pass.

    Gottschalk, S., Lin, M. C., Manocha, D. (1996), *OBBTree: a hierarchical
    structure for rapid interference detection.* Proc. SIGGRAPH, p. 171.
    Ericson, C. (2005), *Real-Time Collision Detection.* Morgan Kaufmann,
    chapter 5. The separating axis theorem that gives the contact normal
    and depth between two convex pieces.

    Catto, E. (2005), *Iterative dynamics with temporal coherence.* Game
    Developers Conference; and Box2D (https://box2d.org). Sequential
    per-pair resolution, the linear slop left between separated bodies,
    and the fattened bounding boxes of the broad phase.

    Chew, L. P. (1989), *Constrained Delaunay triangulations.* Algorithmica
    4:97, as implemented by GEOS and exposed through
    :func:`shapely.constrained_delaunay_triangles`, for the convex
    decomposition of non-convex polygons.

Author: Ruihua Han assisted by claude
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import shapely
from shapely.geometry import LineString, Polygon
from shapely.geometry.base import BaseGeometry

from irsim.util.util import is_convex_polygon

# Gap left between two bodies after a contact is resolved, so that shapes that
# were just separated do not register as intersecting (colliding) again.
CONTACT_SLOP = 1e-6
# Bodies closer than this rest against each other: such a pair needs no
# correction, but an immovable or driving partner still blocks the body, so a
# box resting on a wall cannot be pushed into the wall in the same sweep.
REST_TOLERANCE = 10 * CONTACT_SLOP
# Maximum Gauss-Seidel sweeps over the contact pairs per step. Sweeps stop as
# soon as nothing overlaps. A chain anchored on an immovable object (robot
# pushing a box into a wall) settles in one sweep thanks to the anchored
# order and blocked directions; the cap only bounds free chains and closed
# loops of contacts, whose residual shrinks from sweep to sweep.
CONTACT_ITERATIONS = 32

# Map boundary segments this close to an object's bounding box take part.
_MAP_QUERY_MARGIN = 1e-3

_EPS = 1e-12
_FEATURE_TOL = 1e-7  # vertices this close along the normal form one edge
_LEVER_TOL = 1e-6  # contact arms shorter than this are a centered push
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
        depth: Penetration depth that was removed, in meters; zero or
            negative for a resting contact, which moved nothing.
        point: World-frame contact point ``(2,)``, midway through the overlap.
        force: Contact force along ``normal`` in newtons, the constraint
            impulse over the squared step time as XPBD estimates it, capped
            at the traction of a driven object pushing into the contact;
            ``0`` when the step time was not given.
    """

    a: Any
    b: Any
    normal: np.ndarray
    depth: float
    point: np.ndarray | None = None
    force: float = 0.0


# ---------------------------------------------------------------------------
# Convex decomposition
# ---------------------------------------------------------------------------


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
        convex = not polygon.interiors and is_convex_polygon(polygon)
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


Mtv = tuple[np.ndarray, float, np.ndarray]


def piece_mtv(a: Piece, b: Piece, tolerance: float = 0.0) -> Mtv | None:
    """Minimum translation that separates convex piece ``a`` from ``b``.

    Args:
        a: Convex piece to move.
        b: Convex piece it overlaps.
        tolerance: Gap up to which separated pieces still count as a contact,
            reported with a negative depth.

    Returns:
        tuple | None: ``(normal, depth, point)`` where the unit ``normal``
        points from ``b`` toward ``a``, moving ``a`` by ``normal * depth``
        separates the pieces, and ``point`` is the contact point midway
        through the overlap; ``None`` when they do not overlap. Exactly
        touching pieces count as a zero-depth contact, since shapely reports
        them as intersecting.
    """
    if a[0] == "circle" and b[0] == "circle":
        offset = a[1] - b[1]
        distance = float(np.hypot(offset[0], offset[1]))
        depth = a[2] + b[2] - distance
        if depth < -tolerance:
            return None
        normal = np.array([1.0, 0.0]) if distance <= _EPS else offset / distance
        return normal, depth, b[1] + normal * (b[2] - 0.5 * depth)

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
    if np.any(push_down < -tolerance) or np.any(push_up < -tolerance):
        return None

    depths = np.minimum(push_down, push_up)
    best = int(np.argmin(depths))
    normal = axes[best] if push_up[best] <= push_down[best] else -axes[best]
    return normal, float(depths[best]), _contact_point(a, b, normal)


def _contact_point(a: Piece, b: Piece, normal: np.ndarray) -> np.ndarray:
    """Contact point of two overlapping pieces along a known normal.

    The deepest feature of ``a`` toward ``b`` and of ``b`` toward ``a`` (a
    vertex, an edge, or a circle's rim point) are found along the normal, and
    the point is placed midway through the overlap, at the middle of the
    stretch where the two features overlap along the tangent. A vertex on a
    face therefore acts at the vertex, two parallel faces act at the middle of
    their overlap, and a circle acts at its rim.
    """
    tangent = np.array([-normal[1], normal[0]])
    low_a, t_a = _support_extent(a, normal, -1.0, tangent)
    high_b, t_b = _support_extent(b, normal, 1.0, tangent)
    t_low, t_high = max(t_a[0], t_b[0]), min(t_a[1], t_b[1])
    if t_low <= t_high:
        t_mid = 0.5 * (t_low + t_high)
    else:  # features do not overlap along the tangent: split the difference
        t_mid = 0.25 * (t_a[0] + t_a[1] + t_b[0] + t_b[1])
    return normal * (0.5 * (low_a + high_b)) + tangent * t_mid


def _support_extent(
    piece: Piece, normal: np.ndarray, sign: float, tangent: np.ndarray
) -> tuple[float, tuple[float, float]]:
    """Extreme feature of ``piece`` in direction ``sign * normal``.

    Returns its coordinate along ``normal`` and its extent along ``tangent``.
    """
    if piece[0] == "circle":
        center, radius = piece[1], piece[2]
        t = float(center @ tangent)
        return float(center @ normal) + sign * radius, (t, t)
    vertices = piece[1]
    along = vertices @ normal
    extreme = float(np.max(sign * along))
    feature = vertices[sign * along >= extreme - _FEATURE_TOL]
    t = feature @ tangent
    return sign * extreme, (float(t.min()), float(t.max()))


def object_mtv(
    a: Any, b: Any, cache: dict[Any, list[Piece]] | None = None
) -> Mtv | None:
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
        tuple | None: ``(normal, depth, point)`` with the normal pointing from
        ``b`` to ``a``, or ``None`` when no pieces overlap.
    """
    best: Mtv | None = None
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


def contact_step(
    pairs: list[tuple[Any, Any]],
    iterations: int = CONTACT_ITERATIONS,
    slop: float = CONTACT_SLOP,
    margin: float = 0.0,
    step_time: float | None = None,
) -> list[Contact]:
    """Separate overlapping object pairs, sharing each correction by mass.

    One call resolves the contacts of one simulation step, in three stages:

    1. Order: pairs that nothing can move are dropped, and the rest are
       sorted by their contact-graph distance from immovable objects
       (:func:`_anchored_order`), so a wall contact is handled before the
       robot contact behind it.
    2. Sweep: Gauss-Seidel passes over the pairs. Each overlapping pair is
       separated along its contact normal: ``a`` moves by
       ``depth * w_a / (w_a + w_b)`` and ``b`` by the rest, where ``w`` is
       the inverse mass (zero for static or infinite-mass objects, which
       never move). Within a sweep, an object that was just pressed against
       something it cannot move only yields tangentially afterwards and its
       partner takes the rest, so a chain pushed into a wall settles in one
       sweep and a box pushed obliquely into a wall slides along it. Sweeps
       repeat until one moves nothing, or ``iterations`` is reached.
       A pair resting within :data:`REST_TOLERANCE` of each other needs no
       correction but still blocks: a box resting on a wall is not pushed
       into it and back out again within a sweep.
    3. Apply: the sweeps work on translated copies of each object's convex
       pieces and accumulate one displacement per object, applied at the end
       through
       :py:meth:`~irsim.world.object_base.ObjectBase.apply_contact_displacement`,
       which also folds it into the object's velocity.

    Both objects of every resolved pair are marked as in contact with each
    other.

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
        step_time: Simulation step, needed to report contact forces and to
            apply restitution; without it forces stay ``0`` and nothing
            bounces.

    Returns:
        list[Contact]: One entry per pair that touched, in the order of the
        first sweep that found it, carrying its deepest push; a positive
        ``depth`` means the pair was pushed apart, otherwise the two only
        rest against each other.
    """
    solver = _ContactSolver(pairs, slop=slop, margin=margin, step_time=step_time)
    for _ in range(iterations):
        if not solver.sweep():
            break
    solver.apply()
    return solver.contacts


@dataclass(slots=True)
class _Pair:
    """A candidate pair and how its corrections are shared.

    ``w_a`` / ``w_b`` are the inverse masses. ``driver_a`` says that ``a`` is
    a driven object touching the pushable passive body ``b``: each sweep
    decides whether its traction covers ``b`` and everything ``b`` pushes
    ahead, in which case it yields to ``b`` only what ``b`` cannot take, and
    otherwise stalls against it; ``driver_b`` the reverse.
    ``cone_a`` / ``cone_b`` are the friction coefficients of passive bodies,
    which stick to a blocker when pushed into its friction cone; ``None``
    for driven objects, which slide. ``i_a`` / ``i_b`` are the inverse
    moments of inertia of the bodies a contact may turn. ``bounce`` is the
    pair's restitution, the mean of the two materials' values.
    """

    a: Any
    b: Any
    key_a: int
    key_b: int
    w_a: float
    w_b: float
    driver_a: bool
    driver_b: bool
    cone_a: float | None
    cone_b: float | None
    mu_a: float
    mu_b: float
    i_a: float
    i_b: float
    bounce: float


def _make_pair(a: Any, b: Any) -> _Pair | None:
    """Classify a candidate pair; ``None`` when nothing in it can move."""
    w_a, w_b = a.inv_mass, b.inv_mass
    if w_a + w_b <= 0:
        return None
    return _Pair(
        a,
        b,
        id(a),
        id(b),
        w_a,
        w_b,
        not a.passive and b.passive and w_b > 0,
        not b.passive and a.passive and w_a > 0,
        a.friction if a.passive else None,
        b.friction if b.passive else None,
        a.friction,
        b.friction,
        a.inv_inertia,
        b.inv_inertia,
        0.5 * (a.restitution + b.restitution),
    )


class _ContactSolver:
    """The state of one :func:`contact_step`: its pairs, pieces, and moves.

    Objects are not touched while sweeping. Their convex pieces are
    decomposed once and moved with numpy as displacements and rotations
    accumulate, and each moved object receives its totals in :meth:`apply`.
    """

    def __init__(
        self,
        pairs: list[tuple[Any, Any]],
        slop: float,
        margin: float,
        step_time: float | None = None,
    ):
        self.slop = slop
        self.margin = margin
        self.step_time = step_time
        # the pairs that something can move, those touching walls first
        self.pairs = [
            pair
            for a, b in _anchored_order(pairs)
            if (pair := _make_pair(a, b)) is not None
        ]
        self.contacts: list[Contact] = []
        self._by_pair: dict[tuple[int, int], Contact] = {}
        # constraint impulse (sum of the multipliers) per pair, and the
        # separation speed a bouncy pair must be given after its push, with
        # which of its two bodies the contact was free to move
        self._impulse: dict[tuple[int, int], float] = {}
        self._bounce: dict[
            tuple[int, int], tuple[np.ndarray, float, _Pair, bool, bool]
        ] = {}
        # directions an object may not move in for the rest of the sweep,
        # each with the friction coefficient of the blocker
        self._blocked: dict[int, list[tuple[np.ndarray, float]]] = {}
        # touching pushable passive bodies: key -> [(other key, normal from
        # other to key)], the graph a push's load travels along
        self._touching: dict[int, list[tuple[int, np.ndarray]]] = {}
        # each moved object, its pieces at rest, its pieces moved by the
        # accumulated offset and rotation, and those totals; bodies rotate
        # about their center of mass, so that is also where lever arms start
        self._objects: dict[int, Any] = {}
        self._base: dict[int, list[Piece]] = {}
        self._current: dict[int, list[Piece]] = {}
        self._offset: dict[int, np.ndarray] = {}
        self._angle: dict[int, float] = {}
        self._pivot: dict[int, np.ndarray] = {}
        # grid-map boundary segments near a partner, with their bounding boxes
        self._map_cache: dict[
            tuple[int, int], tuple[list[Piece], np.ndarray, np.ndarray]
        ] = {}

    # -- sweeping ---------------------------------------------------------

    def sweep(self) -> bool:
        """One Gauss-Seidel pass over the pairs; ``True`` when anything moved."""
        self._blocked = {}
        self._touching = self._passive_contacts()
        moved = False
        for pair in self.pairs:
            moved = self._separate(pair) or moved
        return moved

    def _passive_contacts(self) -> dict[int, list[tuple[int, np.ndarray]]]:
        """Which pushable passive bodies touch, with the normal between them."""
        touching: dict[int, list[tuple[int, np.ndarray]]] = {}
        for pair in self.pairs:
            if pair.cone_a is None or pair.cone_b is None or pair.w_a * pair.w_b <= 0:
                continue
            mtv = self.mtv(pair.a, pair.b)
            if mtv is None:
                continue
            normal = mtv[0]
            touching.setdefault(pair.key_a, []).append((pair.key_b, normal))
            touching.setdefault(pair.key_b, []).append((pair.key_a, -normal))
        return touching

    def _load(self, key: int, direction: np.ndarray) -> float:
        """Ground friction a push along ``direction`` on this body must overcome.

        The body's own friction force plus that of every touching passive
        body lying ahead of it in the push direction, recursively, so a
        light box does not let a robot push a heavy one behind it.
        """
        total = 0.0
        stack, seen = [key], {key}
        while stack:
            current = stack.pop()
            total += self._objects[current].friction_force
            for other, normal in self._touching.get(current, ()):
                # ``normal`` points from ``other`` to ``current``: pushing
                # ``current`` along ``direction`` drives it into ``other``
                # when the two oppose
                if other not in seen and float(direction @ normal) < -_EPS:
                    seen.add(other)
                    stack.append(other)
        return total

    def _separate(self, pair: _Pair) -> bool:
        """Remove the overlap of one pair; ``True`` when an object moved."""
        mtv = self.mtv(pair.a, pair.b)
        if mtv is None:
            return False
        normal, depth, point = mtv
        depth += self.slop
        moved = depth > 0  # otherwise the pair only rests against each other

        # 1. What each object can do about it: the motion its blockers allow,
        #    the separation along the normal one unit of that motion buys, and
        #    the turn the contact's lever arm allows through its inertia.
        move_a, gain_a = _free_motion(
            self._blocked.get(pair.key_a), normal, pair.cone_a
        )
        move_b, gain_b = _free_motion(
            self._blocked.get(pair.key_b), -normal, pair.cone_b
        )
        share_a, share_b = pair.w_a * gain_a, pair.w_b * gain_b
        lever_a = _lever(point - self._center(pair.key_a), normal)
        lever_b = _lever(point - self._center(pair.key_b), normal)
        turn_a, turn_b = pair.i_a * lever_a**2, pair.i_b * lever_b**2
        # a driver whose traction covers the body it pushes, and everything
        # that body pushes ahead of it, keeps its motion and heading and lets
        # the body take the whole correction; it yields only when the body
        # cannot move away (a body that can merely turn still makes it yield,
        # and turns as much as its inertia allows). Otherwise it stalls: the
        # body acts immovable to it, and an off-center contact deflects it as
        # slipping wheels would.
        if pair.driver_a:
            if pair.a.friction_force >= self._load(pair.key_b, -normal):
                if share_b > 0:
                    share_a = turn_a = 0.0
            else:
                share_b = turn_b = 0.0
        if pair.driver_b:
            if pair.b.friction_force >= self._load(pair.key_a, normal):
                if share_a > 0:
                    share_b = turn_b = 0.0
            else:
                share_a = turn_a = 0.0
        total = share_a + turn_a + share_b + turn_b
        if total <= 0:
            return False  # squeezed between blockers; the status check reports it

        # 2. Split the depth in proportion to those generalized inverse masses:
        #    each body translates by its share and turns by its lever arm.
        if moved:
            scale = depth / total
            key = (pair.key_a, pair.key_b)
            self._impulse[key] = self._impulse.get(key, 0.0) + scale
            if pair.bounce > 0 and key not in self._bounce:
                approach = float(
                    (pair.a.velocity_xy - pair.b.velocity_xy).ravel() @ normal
                )
                if approach < 0:
                    # only a body this contact could move may bounce: one a
                    # driver cannot push, or one pressed on a blocker, is
                    # not hammered forward by repeated kicks
                    self._bounce[key] = (
                        normal,
                        -pair.bounce * approach,
                        pair,
                        share_a + turn_a > 0,
                        share_b + turn_b > 0,
                    )
            if share_a > 0:
                self.move(pair.key_a, scale * pair.w_a * move_a)
            if turn_a > 0:
                self.rotate(pair.key_a, scale * pair.i_a * lever_a)
            if share_b > 0:
                self.move(pair.key_b, scale * pair.w_b * move_b)
            if turn_b > 0:
                self.rotate(pair.key_b, -scale * pair.i_b * lever_b)

        # 3. Whoever could not yield now blocks its partner along the normal.
        if share_b <= 0 and pair.w_a > 0:
            self._blocked.setdefault(pair.key_a, []).append((-normal, pair.mu_b))
        if share_a <= 0 and pair.w_b > 0:
            self._blocked.setdefault(pair.key_b, []).append((normal, pair.mu_a))

        self._record(pair, normal, depth, point)
        return moved

    def _record(
        self, pair: _Pair, normal: np.ndarray, depth: float, point: np.ndarray
    ) -> None:
        """Report the pair's contact, resting or pushed, keeping its deepest push."""
        key = (pair.key_a, pair.key_b)
        contact = self._by_pair.get(key)
        if contact is None:
            contact = Contact(pair.a, pair.b, normal, depth, point)
            self._by_pair[key] = contact
            self.contacts.append(contact)
            _mark_contact(pair.a, pair.b)
        elif depth > contact.depth:
            contact.normal, contact.depth, contact.point = normal, depth, point

    # -- geometry ---------------------------------------------------------

    def pieces(self, obj: Any, other: Any) -> list[Piece]:
        """Current pieces of ``obj``; for a map, the segments near ``other``."""
        key = id(obj)
        current = self._current.get(key)
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
        self._objects[key] = obj
        self._base[key] = object_pieces(obj)
        self._current[key] = self._base[key]
        self._pivot[key] = np.asarray(obj.centroid, dtype=float).reshape(2)
        return self._current[key]

    def _center(self, key: int) -> np.ndarray:
        """Current center of mass of an object, the point it rotates about."""
        pivot = self._pivot.get(key)
        if pivot is None:
            return np.zeros(2)  # a map: never moves, never turns
        offset = self._offset.get(key)
        return pivot if offset is None else pivot + offset

    def _map_segments(
        self, grid: Any, other: Any
    ) -> tuple[list[Piece], np.ndarray, np.ndarray]:
        """Boundary segments of ``grid`` within the margin of ``other``, cached per pair."""
        map_key = (id(grid), id(other))
        if map_key not in self._map_cache:
            segments = _map_pieces_near(grid, other, self.margin + _MAP_QUERY_MARGIN)
            if segments:
                vertices = np.stack([piece[1] for piece in segments])
                lo, hi = vertices.min(axis=1), vertices.max(axis=1)
            else:
                lo = hi = np.zeros((0, 2))
            self._map_cache[map_key] = (segments, lo, hi)
        return self._map_cache[map_key]

    def mtv(self, a: Any, b: Any) -> Mtv | None:
        """Deepest minimum translation between the current pieces of ``a`` and ``b``."""
        pieces_a = self.pieces(a, b)
        pieces_b = self.pieces(b, a)
        if len(pieces_a) == 1 and len(pieces_b) == 1:
            return piece_mtv(pieces_a[0], pieces_b[0], REST_TOLERANCE)
        best: Mtv | None = None
        for piece_a in pieces_a:
            for piece_b in pieces_b:
                mtv = piece_mtv(piece_a, piece_b, REST_TOLERANCE)
                if mtv is not None and (best is None or mtv[1] > best[1]):
                    best = mtv
        return best

    def move(self, key: int, delta: np.ndarray) -> None:
        """Add ``delta`` to an object's displacement and move its pieces."""
        total = self._offset.get(key)
        self._offset[key] = delta if total is None else total + delta
        self._refresh(key)

    def rotate(self, key: int, angle: float) -> None:
        """Add ``angle`` to an object's rotation and move its pieces."""
        self._angle[key] = self._angle.get(key, 0.0) + angle
        self._refresh(key)

    def _refresh(self, key: int) -> None:
        offset = self._offset.get(key)
        angle = self._angle.get(key, 0.0)
        pivot = self._pivot[key]
        self._current[key] = [
            _transform(piece, offset, angle, pivot) for piece in self._base[key]
        ]

    def apply(self) -> None:
        """Hand every moved object its displacement, rotation, force and bounce."""
        for key in self._offset.keys() | self._angle.keys():
            self._objects[key].apply_contact_displacement(
                self._offset.get(key, np.zeros(2)), self._angle.get(key, 0.0)
            )
        if self.step_time is None or self.step_time <= 0:
            return
        # forces: the constraint multiplier over the squared step, as in XPBD;
        # a driven object pushing into the contact cannot transmit more than
        # its wheels' grip, so its force is capped at its traction, which also
        # keeps a stalled robot's force independent of the step size
        for key, impulse in self._impulse.items():
            contact = self._by_pair[key]
            contact.force = impulse / self.step_time**2
            for obj, into in (
                (contact.a, -contact.normal),
                (contact.b, contact.normal),
            ):
                pushing = not obj.passive and obj.pushable
                if pushing and float(obj.drive_velocity_xy.ravel() @ into) > 0:
                    contact.force = min(contact.force, obj.friction_force)
            push = contact.force * contact.normal
            contact.a.add_contact_force(push)
            contact.b.add_contact_force(-push)
        # restitution: a bouncy pair must separate at the pair's restitution
        # times its approach speed; whatever the position fold left short of
        # that is added to the passive bodies, shared by inverse mass
        for normal, target, pair, free_a, free_b in self._bounce.values():
            w_a = pair.w_a if pair.a.passive and free_a else 0.0
            w_b = pair.w_b if pair.b.passive and free_b else 0.0
            if w_a + w_b <= 0:
                continue
            separating = float(
                (pair.a.velocity_xy - pair.b.velocity_xy).ravel() @ normal
            )
            missing = target - separating
            if missing <= 0:
                continue
            if w_a > 0:
                pair.a.add_contact_velocity(missing * w_a / (w_a + w_b) * normal)
            if w_b > 0:
                pair.b.add_contact_velocity(-missing * w_b / (w_a + w_b) * normal)


def _lever(r: np.ndarray, n: np.ndarray) -> float:
    """Lever arm of a push along ``n`` applied at ``r`` from the pivot: ``r x n``.

    Arms below a micrometre are floating-point noise of a centered contact and
    count as zero, so a centered push never turns a body.
    """
    lever = float(r[0] * n[1] - r[1] * n[0])
    return lever if abs(lever) > _LEVER_TOL else 0.0


def _transform(
    piece: Piece, offset: np.ndarray | None, angle: float, pivot: np.ndarray
) -> Piece:
    """Rotate a piece by ``angle`` about ``pivot`` and translate it by ``offset``."""
    shift = pivot if offset is None else pivot + offset
    if angle == 0.0:
        rotation = None
    else:
        c, s = np.cos(angle), np.sin(angle)
        rotation = np.array([[c, -s], [s, c]])
    if piece[0] == "circle":
        center = piece[1] - pivot
        if rotation is not None:
            center = rotation @ center
        return ("circle", center + shift, piece[2])
    vertices = piece[1] - pivot
    if rotation is None:
        return ("polygon", vertices + shift, *piece[2:])
    vertices = vertices @ rotation.T + shift
    if len(piece) > 2:
        return ("polygon", vertices, piece[2] @ rotation.T)
    return ("polygon", vertices)


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
    blocks: list[tuple[np.ndarray, float]] | None,
    direction: np.ndarray,
    cone: float | None,
) -> tuple[np.ndarray, float]:
    """Project a wanted move onto what the object's blockers allow.

    Returns the allowed motion (the tangential part along the most opposing
    blocker, or ``direction`` itself when nothing blocks it) and its
    component along ``direction``, which is the separation one unit of that
    motion produces. ``cone`` is the friction coefficient of a passive body:
    pressed into a blocker, it slides only when the push leaves the friction
    cone of the two surfaces (mean coefficient, as physics engines combine
    materials by default), and sticks otherwise. A driven object (``None``)
    always slides, since its drive can point along the surface.
    """
    if not blocks:
        return direction, 1.0
    into = [float(direction @ u) for u, _ in blocks]
    worst = int(np.argmax(into))
    if into[worst] <= 0:
        return direction, 1.0
    normal, mu_blocker = blocks[worst]
    motion = direction - into[worst] * normal
    gain = float(motion @ direction)
    if gain <= _EPS:
        return np.zeros(2), 0.0
    if cone is not None:
        mu = 0.5 * (cone + mu_blocker)
        if float(np.linalg.norm(motion)) <= mu * into[worst]:
            return np.zeros(2), 0.0  # inside the friction cone: it sticks
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
