"""Vectorized 2D ray casting against simulation objects.

Shared by the lidar sensors (:class:`~irsim.world.sensors.lidar2d.Lidar2D` and
:class:`~irsim.world.sensors.fmcw_lidar2d.FMCWLidar2D`). The high-level
:func:`cast_rays` function flattens already-detected object boundaries and
returns the nearest object hit by every beam. The lower-level
:func:`cast_ray_segments` function performs the vectorized numerical kernel.

For a sensor origin in free space, this reproduces a GEOS ``difference`` scan
to floating-point precision while avoiding the expensive overlay. FMCW exit
hits also remain compatible when the origin is inside an obstacle.

Typical use::

    ranges, hit_object_indices, origin, directions = cast_rays(
        lidar_geometry, detected_objects, range_max
    )
"""

import numpy as np
import shapely

# Beam hits closer than this are treated as the origin itself (e.g. the sensor
# sitting exactly on an obstacle edge) and ignored, matching GEOS's difference,
# which drops such coincident intersections topologically.
ORIGIN_EPS = 1e-9

# Limit the ray/segment matrix to a predictable size. Large point-based maps
# can contribute tens of thousands of boundary segments; processing all of
# them at once would make every temporary matrix scale with segments * beams.
SEGMENT_CHUNK_SIZE = 1024


def _empty_scan(number: int, max_range: float) -> tuple[np.ndarray, np.ndarray]:
    """Return the range and hit-index arrays for a scan with no hits."""
    # dtype=float is important when max_range comes from YAML as an integer;
    # otherwise later assignments would truncate fractional hit distances.
    ranges = np.full(number, max_range, dtype=float)
    hit_index = np.full(number, -1, dtype=int)
    return ranges, hit_index


def _nonparallel_hit_distances(
    directions: np.ndarray,
    segment_vectors: np.ndarray,
    start_to_origin: np.ndarray,
    max_range: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Calculate valid distances for every non-parallel ray/segment pair.

    For each pair, solve

    ``origin + ray_distance * direction``
    ``= segment_start + segment_position * segment_vector``.

    A valid hit is in front of the ray origin and between the two segment
    endpoints. Invalid pairs are represented by infinity so that a later
    minimum operation naturally ignores them.

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray]: The ``(segments, rays)``
        distance matrix, its intersection denominator, and the per-segment
        cross product used to identify collinear segments.
    """
    perpendicular_directions = np.column_stack((-directions[:, 1], directions[:, 0]))
    with np.errstate(all="ignore"):
        denominator = segment_vectors @ perpendicular_directions.T
        segment_position = (start_to_origin @ perpendicular_directions.T) / denominator
        line_cross = (
            segment_vectors[:, 0] * start_to_origin[:, 1]
            - segment_vectors[:, 1] * start_to_origin[:, 0]
        )
        ray_distance = line_cross[:, np.newaxis] / denominator

    # Ignore intersections at the sensor origin. For example, a sensor on a
    # grid-cell edge would otherwise report zero for every crossing beam.
    valid_hit = (
        (denominator != 0)
        & (segment_position >= 0)
        & (segment_position <= 1)
        & (ray_distance > ORIGIN_EPS)
        & (ray_distance <= max_range)
    )
    return np.where(valid_hit, ray_distance, np.inf), denominator, line_cross


def _empty_collinear_hits() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return empty arrays matching :func:`_find_collinear_hits`."""
    empty_index = np.empty(0, dtype=int)
    return empty_index, empty_index.copy(), np.empty(0, dtype=float)


def _find_collinear_hits(
    origin: np.ndarray,
    directions: np.ndarray,
    seg_start: np.ndarray,
    seg_end: np.ndarray,
    denominator: np.ndarray,
    line_cross: np.ndarray,
    max_range: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Find hits where a ray overlaps a collinear segment.

    Parallel pairs have a zero intersection denominator. Most are separate
    lines and therefore misses. A pair is collinear only when the segment's
    supporting line also passes through the ray origin. For such pairs, project
    both endpoints onto the ray and return the nearest positive endpoint.

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray]: Matching segment indices,
        ray indices, and hit distances. All arrays are one-dimensional.
    """
    collinear_segments = np.flatnonzero(np.abs(line_cross) <= ORIGIN_EPS)
    if len(collinear_segments) == 0:
        return _empty_collinear_hits()

    local_segment_index, ray_index = np.nonzero(denominator[collinear_segments] == 0)
    segment_index = collinear_segments[local_segment_index]
    if len(segment_index) == 0:
        return _empty_collinear_hits()

    matching_directions = directions[ray_index]
    start_distance = np.sum(
        (seg_start[segment_index] - origin) * matching_directions, axis=1
    )
    end_distance = np.sum(
        (seg_end[segment_index] - origin) * matching_directions,
        axis=1,
    )
    overlap_start = np.minimum(start_distance, end_distance)
    overlap_end = np.maximum(start_distance, end_distance)

    # When an overlap begins at the sensor origin, ignore distance zero and use
    # its other positive endpoint, matching the previous scanner behavior.
    hit_distance = np.where(
        overlap_start > ORIGIN_EPS,
        overlap_start,
        np.minimum(overlap_end, max_range),
    )
    valid_hit = (
        (overlap_end > ORIGIN_EPS)
        & (overlap_start <= max_range)
        & (hit_distance <= max_range)
    )
    return (
        segment_index[valid_hit],
        ray_index[valid_hit],
        hit_distance[valid_hit],
    )


def _nearest_hits(
    hit_distances: np.ndarray, max_range: float
) -> tuple[np.ndarray, np.ndarray]:
    """Select the nearest segment for each ray from a distance matrix."""
    number = hit_distances.shape[1]
    ranges, hit_index = _empty_scan(number, max_range)

    nearest_segment = np.argmin(hit_distances, axis=0)
    nearest_distance = hit_distances[nearest_segment, np.arange(number)]
    has_hit = np.isfinite(nearest_distance)
    ranges[has_hit] = nearest_distance[has_hit]
    hit_index[has_hit] = nearest_segment[has_hit]
    return ranges, hit_index


def boundary_segments(geometry):
    """Flatten a geometry's boundary into ``(start, end)`` segment arrays.

    Handles every obstacle geometry a lidar may encounter: polygons (with
    holes), rectangles, circles (buffer polygons), linestrings, map segments,
    and compound ``MultiPolygon`` / ``GeometryCollection`` shapes. Points and
    empty geometries contribute no edges.

    Polygonal parts are reduced to their ring linestrings with
    :func:`shapely.boundary` (which covers the outer ring and any holes);
    linestrings are used directly. Note ``shapely.boundary`` of a *linestring*
    returns its endpoints, not its segments, so it must not be applied to line
    obstacles.

    Args:
        geometry: Any Shapely geometry.

    Returns:
        tuple[np.ndarray, np.ndarray]: ``(start, end)`` endpoint arrays, each of
        shape ``(M, 2)`` (``(0, 2)`` when the geometry has no edges).
    """
    # Collect boundary linestrings: polygon rings via shapely.boundary, lines
    # as-is. shapely.get_parts flattens Multi* / GeometryCollection one level.
    lines = []
    for part in shapely.get_parts(geometry):
        if part.geom_type in ("Polygon", "MultiPolygon"):
            lines.append(shapely.boundary(part))
        elif part.geom_type in ("LineString", "LinearRing", "MultiLineString"):
            lines.append(part)
    if not lines:
        empty = np.empty((0, 2))
        return empty, empty

    # Split every linestring into (start, end) segments in one vectorized pass.
    # ``return_index`` tags each coordinate with its linestring, so masking on
    # equal neighbours keeps segments from spanning two different linestrings.
    parts = shapely.get_parts(lines)
    coords, index = shapely.get_coordinates(parts, return_index=True)
    if len(coords) < 2:
        empty = np.empty((0, 2))
        return empty, empty
    same = index[:-1] == index[1:]
    return coords[:-1][same], coords[1:][same]


def _empty_segments() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return empty segment endpoints and their owning-object indices."""
    empty = np.empty((0, 2))
    return empty, empty.copy(), np.empty(0, dtype=int)


def _gather_obstacle_edges(
    lidar_geometry,
    detected_objects,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Flatten detected-object boundaries into line segments.

    The caller has already selected detectable scene objects. Map objects keep
    their internal spatial query here so only map lines touched by the scan are
    expanded. Owner indices refer to ``detected_objects``.

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray]: Segment starts, segment ends,
        and the owning object index for every segment, with shapes ``(M, 2)``,
        ``(M, 2)``, and ``(M,)``.
    """
    starts, ends, owners = [], [], []
    for object_index, obj in enumerate(detected_objects):
        if obj.shape == "map":
            hits = obj.geometry_tree.query(lidar_geometry, predicate="intersects")
            geometries = [obj.linestrings[index] for index in hits]
        elif lidar_geometry.intersects(obj._geometry):
            geometries = [obj._geometry]
        else:
            continue

        if geometries:
            start, end = boundary_segments(geometries)
            if len(start):
                starts.append(start)
                ends.append(end)
                owners.append(np.full(len(start), object_index))

    if not starts:
        return _empty_segments()
    return np.concatenate(starts), np.concatenate(ends), np.concatenate(owners)


def _ray_parameters(lidar_geometry, max_range: float) -> tuple[np.ndarray, np.ndarray]:
    """Extract the shared origin and unit directions from max-range beams."""
    coordinates = shapely.get_coordinates(lidar_geometry)
    origin = coordinates[0]
    directions = (coordinates[1::2] - origin) / max_range
    return origin, directions


def cast_ray_segments(
    origin: np.ndarray,
    directions: np.ndarray,
    seg_start: np.ndarray,
    seg_end: np.ndarray,
    max_range: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Nearest ray-segment hit distance and segment index per ray (vectorized).

    Solves, for every ray ``origin + t * direction`` against every segment
    ``seg_start + u * (seg_end - seg_start)``, the intersection in vectorized
    segment blocks and keeps the nearest valid hit (``0 < t <= max_range``,
    ``0 <= u <= 1``) per ray. Blocking bounds peak matrix memory by
    ``SEGMENT_CHUNK_SIZE * number_of_rays``. Collinear overlaps are handled
    separately because their standard intersection denominator is zero.

    Args:
        origin (np.ndarray): Shared ray origin ``(2,)``.
        directions (np.ndarray): Unit ray directions ``(N, 2)``.
        seg_start (np.ndarray): Segment start points ``(M, 2)``.
        seg_end (np.ndarray): Segment end points ``(M, 2)``.
        max_range (float): Maximum ray length; misses return this.

    Returns:
        tuple[np.ndarray, np.ndarray]: ``ranges`` ``(N,)`` clamped to
        ``max_range``, and ``hit_index`` ``(N,)`` giving the index into the
        segment arrays that each ray hit (``-1`` on a miss).
    """
    if len(seg_start) == 0:
        return _empty_scan(len(directions), max_range)

    ranges, hit_index = _empty_scan(len(directions), max_range)

    for chunk_start in range(0, len(seg_start), SEGMENT_CHUNK_SIZE):
        chunk_end = min(chunk_start + SEGMENT_CHUNK_SIZE, len(seg_start))
        chunk_segment_start = seg_start[chunk_start:chunk_end]
        chunk_segment_end = seg_end[chunk_start:chunk_end]
        segment_vectors = chunk_segment_end - chunk_segment_start
        start_to_origin = origin - chunk_segment_start

        # 1. Solve ordinary intersections for this bounded segment block.
        hit_distances, denominator, line_cross = _nonparallel_hit_distances(
            directions,
            segment_vectors,
            start_to_origin,
            max_range,
        )

        # 2. Fill in overlapping pairs that the zero denominator cannot solve.
        segment_index, ray_index, distance = _find_collinear_hits(
            origin,
            directions,
            chunk_segment_start,
            chunk_segment_end,
            denominator,
            line_cross,
            max_range,
        )
        hit_distances[segment_index, ray_index] = distance

        # 3. Merge this block's nearest hits into the per-beam result. A strict
        # comparison preserves np.argmin's original first-segment tie behavior.
        chunk_ranges, chunk_hit_index = _nearest_hits(hit_distances, max_range)
        nearer = (chunk_hit_index >= 0) & ((hit_index < 0) | (chunk_ranges < ranges))
        ranges[nearer] = chunk_ranges[nearer]
        hit_index[nearer] = chunk_start + chunk_hit_index[nearer]

    return ranges, hit_index


def cast_rays(
    lidar_geometry,
    detected_objects,
    max_range: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Cast a 2D lidar geometry against already-detected objects.

    This geometry-only operation derives ray parameters, gathers boundary
    segments from the supplied objects, runs the numerical kernel, and maps
    segment hits back to indices in ``detected_objects``. Scene lookup remains
    the sensor's responsibility.

    Args:
        lidar_geometry: Max-range beams in world coordinates as a Shapely
            multiline geometry.
        detected_objects: Objects selected by the sensor's scene query.
        max_range: Maximum ray length; misses return this value.

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: Ranges,
        indices into ``detected_objects``, origin, and directions.
    """
    shapely.prepare(lidar_geometry)
    origin, directions = _ray_parameters(lidar_geometry, max_range)
    segment_start, segment_end, segment_owner = _gather_obstacle_edges(
        lidar_geometry,
        detected_objects,
    )
    ranges, hit_segments = cast_ray_segments(
        origin,
        directions,
        segment_start,
        segment_end,
        max_range,
    )

    hit_object_indices = np.full(len(directions), -1, dtype=int)
    has_hit = hit_segments >= 0
    hit_object_indices[has_hit] = segment_owner[hit_segments[has_hit]]
    return ranges, hit_object_indices, origin, directions
