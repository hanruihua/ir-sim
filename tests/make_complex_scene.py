"""Structured complex-scene generator for ray-casting benchmarks.

Instead of random scatter, this builds a realistic indoor-like environment:
  - Outer boundary walls
  - A grid of rooms separated by corridor walls
  - Additional furniture/pillar circles
  - Diagonal partition walls

Segments are returned as (start, end) NumPy arrays ready for cast_ray_segments.
"""

from __future__ import annotations

import math

import numpy as np


def _rect_segments(cx, cy, w, h, theta=0.0):
    """Return the 4 wall segments of an axis-aligned or rotated rectangle."""
    hw, hh = w / 2, h / 2
    corners = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]], dtype=float)
    if theta != 0.0:
        c, s = math.cos(theta), math.sin(theta)
        R = np.array([[c, -s], [s, c]])
        corners = corners @ R.T
    corners += [cx, cy]
    starts = corners
    ends = np.roll(corners, -1, axis=0)
    return starts, ends


def _linestring_segments(pts):
    pts = np.asarray(pts, dtype=float)
    return pts[:-1], pts[1:]


def make_complex_scene(
    n_beams: int,
    range_max: float,
    angle_range: float,
    seed: int = 42,
    target_segments: int = 500,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build a structured indoor-like scene.

    Args:
        n_beams: Number of scan beams.
        range_max: Sensor max range (world extends to ~1.1x this).
        angle_range: FoV in radians.
        seed: RNG seed for minor randomisation.
        target_segments: Approximate number of boundary segments to generate.

    Returns:
        origin (2,), directions (N,2), seg_start (M,2), seg_end (M,2)
    """
    rng = np.random.default_rng(seed)
    R = range_max * 0.9  # world half-size

    all_starts: list[np.ndarray] = []
    all_ends: list[np.ndarray] = []

    def add(starts, ends):
        all_starts.append(np.asarray(starts, dtype=float))
        all_ends.append(np.asarray(ends, dtype=float))

    # 1. Outer boundary wall (closed square)
    s, e = _linestring_segments([
        [-R, -R], [R, -R], [R, R], [-R, R], [-R, -R]
    ])
    add(s, e)

    # 2. Room grid: divide space into cells
    #    Number of grid lines depends on target_segments
    n_lines = max(3, int(math.sqrt(target_segments / 8)))
    step = 2 * R / (n_lines + 1)
    # Horizontal walls with doorways
    for k in range(1, n_lines + 1):
        y = -R + k * step
        # Make ~2 doorways per wall (remove segments in those ranges)
        door_centers = rng.uniform(-R * 0.7, R * 0.7, 2)
        door_w = step * 0.35
        cur_start = -R
        for xv in sorted(door_centers):
            d0, d1 = xv - door_w, xv + door_w
            if cur_start < d0:
                add(*_linestring_segments([[cur_start, y], [d0, y]]))
            cur_start = d1
        if cur_start < R:
            add(*_linestring_segments([[cur_start, y], [R, y]]))

    # Vertical walls with doorways
    for k in range(1, n_lines + 1):
        x = -R + k * step
        door_centers = rng.uniform(-R * 0.7, R * 0.7, 2)
        door_w = step * 0.35
        cur_start = -R
        for yv in sorted(door_centers):
            d0, d1 = yv - door_w, yv + door_w
            if cur_start < d0:
                add(*_linestring_segments([[x, cur_start], [x, d0]]))
            cur_start = d1
        if cur_start < R:
            add(*_linestring_segments([[x, cur_start], [x, R]]))

    # 3. Furniture / pillars: small circles approximated as octagons
    n_pillars = max(4, target_segments // 40)
    pillar_r = R * rng.uniform(0.02, 0.05, n_pillars)
    pillar_c = rng.uniform(-R * 0.8, R * 0.8, (n_pillars, 2))
    for (cx, cy), r in zip(pillar_c, pillar_r, strict=False):
        n_sides = 8
        angles = np.linspace(0, 2 * math.pi, n_sides, endpoint=False)
        verts = np.column_stack([cx + r * np.cos(angles), cy + r * np.sin(angles)])
        verts = np.vstack([verts, verts[0]])  # close
        add(*_linestring_segments(verts))

    # 4. Diagonal partitions for variety
    n_diag = max(2, target_segments // 80)
    for _ in range(n_diag):
        cx, cy = rng.uniform(-R * 0.5, R * 0.5, 2)
        length = rng.uniform(R * 0.15, R * 0.35)
        theta = rng.uniform(0, math.pi)
        dx, dy = length / 2 * math.cos(theta), length / 2 * math.sin(theta)
        add(
            np.array([[cx - dx, cy - dy]]),
            np.array([[cx + dx, cy + dy]]),
        )

    # 5. Top up with random segments to hit target_segments
    current_M = sum(len(s) for s in all_starts)
    shortfall = target_segments - current_M
    if shortfall > 0:
        centers = rng.uniform(-R * 0.8, R * 0.8, (shortfall, 2))
        ang_r = rng.uniform(0, math.pi, shortfall)
        L_r = rng.uniform(0.05, R * 0.08, shortfall)
        hv = (L_r[:, None] / 2) * np.column_stack([np.cos(ang_r), np.sin(ang_r)])
        add(centers - hv, centers + hv)

    seg_start = np.concatenate(all_starts)
    seg_end = np.concatenate(all_ends)

    # Beams spanning the FoV
    half = angle_range / 2
    angles_b = np.linspace(-half, half, n_beams)
    directions = np.column_stack([np.cos(angles_b), np.sin(angles_b)])
    origin = np.zeros(2)

    return origin, directions, seg_start, seg_end
