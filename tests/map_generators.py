"""Structured environment map generators for ray-casting benchmarks.

Three industrial/campus map types, each returning (seg_start, seg_end) arrays
of boundary segments ready for cast_ray_segments.  All coordinates are in
[-range_max*0.9, range_max*0.9] so the sensor origin at (0,0) can reach
the entire scene.

Maps
----
warehouse : parallel shelving aisles, loading bays, perimeter wall
factory   : heavy machinery blocks, conveyor linestrings, support pillars
campus    : building footprints, paved walkways, planting beds
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------------------
# Geometry primitives
# ---------------------------------------------------------------------------

def _rect(cx: float, cy: float, w: float, h: float, theta: float = 0.0):
    """Four wall segments of a rectangle (returns start, end arrays)."""
    hw, hh = w / 2, h / 2
    c_ = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]], dtype=float)
    if theta:
        c, s = math.cos(theta), math.sin(theta)
        c_ = c_ @ np.array([[c, s], [-s, c]])
    c_ += [cx, cy]
    return c_, np.roll(c_, -1, axis=0)


def _line(*pts):
    """Segments along a polyline."""
    p = np.asarray(pts, dtype=float)
    return p[:-1], p[1:]


def _octagon(cx: float, cy: float, r: float):
    """Pillar approximated as regular octagon."""
    a = np.linspace(0, 2 * math.pi, 8, endpoint=False)
    v = np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a)])
    v2 = np.vstack([v, v[0]])
    return v2[:-1], v2[1:]


@dataclass
class SceneBuffer:
    starts: list = field(default_factory=list)
    ends:   list = field(default_factory=list)

    def add(self, s, e):
        s = np.asarray(s, dtype=float)
        e = np.asarray(e, dtype=float)
        if s.ndim == 1:
            s, e = s[None], e[None]
        self.starts.append(s)
        self.ends.append(e)

    def arrays(self):
        if not self.starts:
            return np.empty((0, 2)), np.empty((0, 2))
        return np.concatenate(self.starts), np.concatenate(self.ends)


# ---------------------------------------------------------------------------
# Warehouse map
# ---------------------------------------------------------------------------

def warehouse(range_max: float, seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """Warehouse: parallel shelving aisles, loading bays, forklift paths.

    Features
    --------
    * Outer perimeter wall with two loading-bay openings
    * 4-6 pairs of shelving racks forming parallel aisles
    * Cross-aisle at one third from the top
    * Corner staging area enclosed by low walls
    * Support pillars every other aisle
    """
    rng = np.random.default_rng(seed)
    R = range_max * 0.88
    buf = SceneBuffer()

    # Outer wall with two bay openings on the south side
    bay_w = R * 0.18
    buf.add(*_line([-R, R], [-R, -R]))                              # west
    buf.add(*_line([-R, R], [R, R]))                                # north
    buf.add(*_line([R, R], [R, -R]))                                # east
    # south wall with two bays
    buf.add(*_line([-R, -R], [-R * 0.45 - bay_w / 2, -R]))
    buf.add(*_line([-R * 0.45 + bay_w / 2, -R], [R * 0.45 - bay_w / 2, -R]))
    buf.add(*_line([R * 0.45 + bay_w / 2, -R], [R, -R]))

    # Shelving racks: rows of narrow rectangles in parallel north-south aisles
    n_rows = 5
    row_x = np.linspace(-R * 0.75, R * 0.75, n_rows)
    shelf_h = R * 0.14
    shelf_w = R * 0.08
    n_shelves = 7
    shelf_y = np.linspace(-R * 0.72, R * 0.55, n_shelves)
    for rx in row_x:
        for sy in shelf_y:
            buf.add(*_rect(rx, sy, shelf_w, shelf_h))

    # Cross-aisle wall (partial, one gap)
    cross_y = R * 0.22
    buf.add(*_line([-R, cross_y], [-R * 0.22, cross_y]))
    buf.add(*_line([R * 0.22, cross_y], [R, cross_y]))

    # Staging area in NW corner
    buf.add(*_line([-R, R * 0.55], [-R * 0.38, R * 0.55]))
    buf.add(*_line([-R * 0.38, R * 0.55], [-R * 0.38, R]))

    # Pillars
    for px in row_x[::2]:
        buf.add(*_octagon(px, cross_y + R * 0.32, R * 0.04))

    # Stacked pallets (small rectangles near bays)
    for bx in [-R * 0.45, R * 0.45]:
        buf.add(*_rect(bx, -R * 0.75, R * 0.14, R * 0.1,
                       rng.uniform(-0.2, 0.2)))

    return buf.arrays()


# ---------------------------------------------------------------------------
# Factory map
# ---------------------------------------------------------------------------

def factory(range_max: float, seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """Factory floor: machinery, conveyor lines, CNC zones, utility rooms.

    Features
    --------
    * Perimeter wall with two access doors
    * Large machinery blocks (presses, milling machines)
    * Three conveyor belt linestrings
    * CNC machining zone with partial enclosure
    * Utility room in one corner
    * Support columns on a grid
    """
    R = range_max * 0.88
    buf = SceneBuffer()

    door_w = R * 0.16
    # Perimeter
    buf.add(*_line([-R, R], [-R, -R], [-R + door_w, -R]))          # W, SW
    buf.add(*_line([-R + door_w * 2.2, -R], [R, -R], [R, R], [-R, R]))  # S-E-N
    # East door
    buf.add(*_line([R, R * 0.22], [R, R * 0.22 + door_w]))         # door gap (omit)
    buf.add(*_line([R, R * 0.22 + door_w * 1.8], [R, R]))

    # Large machinery (presses, lathes)
    machines = [
        (-R * 0.6,  R * 0.55, R * 0.28, R * 0.22,  0.15),
        ( R * 0.55, R * 0.55, R * 0.32, R * 0.18, -0.1),
        (-R * 0.6, -R * 0.35, R * 0.22, R * 0.30,  0.0),
        ( R * 0.55,-R * 0.35, R * 0.30, R * 0.25,  0.05),
        ( R * 0.05, R * 0.58, R * 0.20, R * 0.14,  0.0),
    ]
    for mx, my, mw, mh, mt in machines:
        buf.add(*_rect(mx, my, mw, mh, mt))

    # Conveyor belts (linestrings)
    conveyors = [
        [(-R * 0.75, R * 0.05), (R * 0.32, R * 0.05)],
        [(-R * 0.3, -R * 0.55), (-R * 0.3, R * 0.25)],
        [(R * 0.22, -R * 0.7), (R * 0.22, -R * 0.1)],
    ]
    for pts in conveyors:
        buf.add(*_line(*pts))

    # CNC zone enclosure (partial walls)
    cx, cy = R * 0.05, -R * 0.38
    buf.add(*_line([cx - R * 0.22, cy + R * 0.24], [cx - R * 0.22, cy - R * 0.24]))
    buf.add(*_line([cx - R * 0.22, cy - R * 0.24], [cx + R * 0.22, cy - R * 0.24]))
    buf.add(*_line([cx + R * 0.22, cy - R * 0.24], [cx + R * 0.22, cy + R * 0.12]))

    # Utility room NE corner
    buf.add(*_line([R * 0.42, R], [R * 0.42, R * 0.45], [R, R * 0.45]))

    # Support columns
    col_xs = np.linspace(-R * 0.7, R * 0.7, 4)
    col_ys = np.linspace(-R * 0.65, R * 0.65, 3)
    for cx2 in col_xs:
        for cy2 in col_ys:
            if abs(cx2) < R * 0.15 and abs(cy2) < R * 0.15:
                continue  # keep centre clear for robot
            buf.add(*_octagon(cx2, cy2, R * 0.038))

    return buf.arrays()


# ---------------------------------------------------------------------------
# Campus map
# ---------------------------------------------------------------------------

def campus(range_max: float, seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """University/corporate campus: buildings, walkways, planting beds.

    Features
    --------
    * Four main building footprints (large rectangles with entrance notches)
    * Covered walkway connecting two buildings (paired linestrings)
    * Open plaza with benches (small rectangles) and planting beds (circles)
    * Parking structure outline in one corner
    * Fountain in plaza centre (octagon)
    * Low perimeter hedge (dashed linestrings)
    """
    R = range_max * 0.88
    buf = SceneBuffer()

    # ---- Buildings ----
    buildings = [
        (-R * 0.52,  R * 0.55, R * 0.44, R * 0.38, 0.0),   # NW science hall
        ( R * 0.52,  R * 0.52, R * 0.40, R * 0.42, 0.05),   # NE admin
        (-R * 0.50, -R * 0.52, R * 0.42, R * 0.36, 0.0),   # SW library
        ( R * 0.50, -R * 0.50, R * 0.38, R * 0.38, -0.05),  # SE engineering
    ]
    for bx, by, bw, bh, bt in buildings:
        # Full rectangle minus one entrance notch (remove one wall section)
        hw, hh = bw / 2, bh / 2
        cnrs = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]], dtype=float)
        if bt:
            c_, s_ = math.cos(bt), math.sin(bt)
            cnrs = cnrs @ np.array([[c_, s_], [-s_, c_]])
        cnrs += [bx, by]
        # three full walls + south wall with door gap
        notch = R * 0.08
        for i in range(4):
            a, b = cnrs[i], cnrs[(i + 1) % 4]
            if i == 0:  # south wall: split with notch
                mid = (a + b) / 2
                buf.add(a[None], (mid - (b - a) / np.linalg.norm(b - a) * notch)[None])
                buf.add((mid + (b - a) / np.linalg.norm(b - a) * notch)[None], b[None])
            else:
                buf.add(a[None], b[None])

    # ---- Covered walkway between NW and NE buildings ----
    wy = R * 0.58
    buf.add(*_line([-R * 0.3, wy + R * 0.04], [R * 0.3, wy + R * 0.04]))
    buf.add(*_line([-R * 0.3, wy - R * 0.04], [R * 0.3, wy - R * 0.04]))
    # Support posts
    for px in np.linspace(-R * 0.25, R * 0.25, 5):
        buf.add(*_octagon(px, wy, R * 0.025))

    # ---- Plaza ----
    # Fountain (central octagon)
    buf.add(*_octagon(0, 0, R * 0.08))

    # Benches (small rectangles radiating from centre)
    bench_r = R * 0.22
    for ang in np.linspace(0, 2 * math.pi, 6, endpoint=False):
        bx2 = bench_r * math.cos(ang)
        by2 = bench_r * math.sin(ang)
        buf.add(*_rect(bx2, by2, R * 0.12, R * 0.03, ang + math.pi / 2))

    # Planting beds (circular raised borders)
    for ang in np.linspace(math.pi / 6, 2 * math.pi + math.pi / 6, 4, endpoint=False):
        px2 = R * 0.42 * math.cos(ang)
        py2 = R * 0.42 * math.sin(ang)
        buf.add(*_octagon(px2, py2, R * 0.065))

    # ---- Parking structure SW-ish ----
    pk_cx, pk_cy = -R * 0.1, -R * 0.72
    buf.add(*_line(
        [pk_cx - R * 0.3, pk_cy - R * 0.12],
        [pk_cx - R * 0.3, pk_cy + R * 0.12],
        [pk_cx + R * 0.3, pk_cy + R * 0.12],
        [pk_cx + R * 0.3, pk_cy - R * 0.12],
        [pk_cx - R * 0.3, pk_cy - R * 0.12],
    ))
    # Parking divider rows
    for px3 in np.linspace(pk_cx - R * 0.2, pk_cx + R * 0.2, 3):
        buf.add(*_line([px3, pk_cy - R * 0.12], [px3, pk_cy + R * 0.12]))

    # ---- Perimeter hedge (partial segments) ----
    hedge_R = R * 0.96
    gap = R * 0.14
    for y_seg in [hedge_R, -hedge_R]:
        buf.add(*_line([-hedge_R, y_seg], [-gap, y_seg]))
        buf.add(*_line([gap, y_seg], [hedge_R, y_seg]))
    for x_seg in [hedge_R, -hedge_R]:
        buf.add(*_line([x_seg, -hedge_R], [x_seg, -gap]))
        buf.add(*_line([x_seg, gap], [x_seg, hedge_R]))

    return buf.arrays()


# ---------------------------------------------------------------------------
# Dynamic object wrapper
# ---------------------------------------------------------------------------

@dataclass
class DynamicSegments:
    """A subset of segments that moves each timestep.

    Models moving forklifts / AGVs (warehouse), conveyors (factory),
    or campus shuttles.  The same segment array is translated and rotated
    by a velocity vector each ``step()`` call.
    """
    seg_start: np.ndarray   # (K, 2)
    seg_end:   np.ndarray   # (K, 2)
    center:    np.ndarray   # pivot point (2,)
    vel:       np.ndarray   # translation velocity (2,) m/step
    omega:     float        # rotation rate rad/step
    bounds:    float        # bounce if |x| or |y| exceeds this

    @classmethod
    def make_forklift(cls, cx: float, cy: float, heading: float, size: float,
                      speed: float, bounds: float, rng: np.random.Generator):
        """Rectangular forklift."""
        hw, hh = size, size * 0.4
        pts = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]], dtype=float)
        c_, s_ = math.cos(heading), math.sin(heading)
        R_mat = np.array([[c_, -s_], [s_, c_]])
        pts = pts @ R_mat.T + [cx, cy]
        ss = pts
        se = np.roll(pts, -1, axis=0)
        vel = speed * np.array([math.cos(heading), math.sin(heading)])
        return cls(ss, se, np.array([cx, cy], dtype=float),
                   vel, omega=0.0, bounds=bounds)

    @classmethod
    def make_agv(cls, cx: float, cy: float, bounds: float,
                 rng: np.random.Generator):
        """Small circular AGV."""
        r = bounds * 0.05
        a = np.linspace(0, 2 * math.pi, 6, endpoint=False)
        v = np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a)])
        ss = v
        se = np.roll(v, -1, axis=0)
        angle = rng.uniform(0, 2 * math.pi)
        speed = bounds * 0.012
        vel = speed * np.array([math.cos(angle), math.sin(angle)])
        return cls(ss, se, np.array([cx, cy], dtype=float),
                   vel, omega=rng.uniform(-0.08, 0.08), bounds=bounds)

    def step(self, dt: float = 1.0):
        """Advance one timestep.  Bounce off bounds."""
        disp = self.vel * dt
        new_cx = self.center[0] + disp[0]
        new_cy = self.center[1] + disp[1]
        if abs(new_cx) > self.bounds:
            self.vel[0] *= -1
        if abs(new_cy) > self.bounds:
            self.vel[1] *= -1
        self.seg_start += disp
        self.seg_end   += disp
        self.center    += disp
        if self.omega != 0.0:
            c_, s_ = math.cos(self.omega * dt), math.sin(self.omega * dt)
            R_mat = np.array([[c_, -s_], [s_, c_]])
            self.seg_start = (self.seg_start - self.center) @ R_mat.T + self.center
            self.seg_end   = (self.seg_end   - self.center) @ R_mat.T + self.center


def make_dynamic_objects(map_type: str, range_max: float, n: int = 4,
                         seed: int = 0) -> list[DynamicSegments]:
    """Return ``n`` dynamic objects suitable for the given map type."""
    rng = np.random.default_rng(seed + 77)
    R = range_max * 0.6
    objects: list[DynamicSegments] = []
    if map_type == "warehouse":
        for _ in range(n):
            cx = rng.uniform(-R, R)
            cy = rng.uniform(-R, R)
            heading = rng.uniform(0, 2 * math.pi)
            objects.append(DynamicSegments.make_forklift(
                cx, cy, heading, R * 0.09, R * 0.015, R * 0.85, rng))
    elif map_type == "factory":
        for _ in range(n):
            objects.append(DynamicSegments.make_agv(
                rng.uniform(-R, R), rng.uniform(-R, R), R * 0.85, rng))
    elif map_type == "campus":
        for _ in range(n):
            cx = rng.uniform(-R * 0.35, R * 0.35)
            cy = rng.uniform(-R * 0.35, R * 0.35)
            objects.append(DynamicSegments.make_agv(cx, cy, R * 0.85, rng))
    return objects


def scene_with_dynamics(
    static_start: np.ndarray,
    static_end:   np.ndarray,
    dynamics:     list[DynamicSegments],
) -> tuple[np.ndarray, np.ndarray]:
    """Merge static + current dynamic segments."""
    ss = [static_start] + [d.seg_start for d in dynamics]
    se = [static_end]   + [d.seg_end   for d in dynamics]
    return np.concatenate(ss), np.concatenate(se)
