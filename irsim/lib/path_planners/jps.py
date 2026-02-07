"""
Jump Point Search (JPS) grid planning.

An optimization of A* for uniform-cost grids that prunes symmetric paths
and expands "jump points" only, preserving optimality while reducing nodes expanded.

Collision precedence:
  1. Grid lookup when ``env_map.grid`` is not ``None``; if occupied, collision.
  2. When the grid reports free or is unavailable, Shapely vs. obstacle_list.
  (Grid and obstacle_list are combined when both are present.)

References
----------
- D. Harabor and A. Grastien. Online Graph Pruning for Pathfinding on Grid Maps.
  In AAAI, 2011. https://en.wikipedia.org/wiki/Jump_point_search
- 2D implementation reference: KumarRobotics/jps3d (C++ JPS on 2D/3D maps).
  https://github.com/KumarRobotics/jps3d

"""

from __future__ import annotations

import math
from dataclasses import dataclass
import matplotlib.pyplot as plt
import numpy as np

from irsim.lib.handler.geometry_handler import GeometryFactory
from irsim.world.map import EnvGridMap

# Type alias: ((jx, jy, dx, dy), cost) for each jump successor
JpsSuccessor = tuple[tuple[int, int, int, int], float]


# --- JPS 2D neighbor tables (aligned with jps3d JPS2DNeib) ---
# Direction id: (dx+1) + 3*(dy+1) for dx,dy in {-1,0,1}
# nsz[norm1][0] = number of natural neighbors, nsz[norm1][1] = number of forced checks
# norm1 = |dx| + |dy|  =>  0: start, 1: cardinal, 2: diagonal
_JPS2D_NSZ = ((8, 0), (1, 2), (3, 2))

# Natural neighbors ns[id][0/1][dev]: id -> (list of dx, list of dy) to try
# Precomputed like JPS2DNeib::Neib: norm1=0 -> 8 dirs; norm1=1 -> (dx,dy); norm1=2 -> (dx,0),(0,dy),(dx,dy)
def _build_jps2d_ns() -> list[list[list[int]]]:
    ns: list[list[list[int]]] = [[[] for _ in range(2)] for _ in range(9)]
    for dy in range(-1, 2):
        for dx in range(-1, 2):
            id_ = (dx + 1) + 3 * (dy + 1)
            norm1 = abs(dx) + abs(dy)
            n = _JPS2D_NSZ[norm1][0]
            for dev in range(n):
                if norm1 == 0:
                    tbl = [(1, 0), (-1, 0), (0, 1), (1, 1), (-1, 1), (0, -1), (1, -1), (-1, -1)]
                    tx, ty = tbl[dev]
                elif norm1 == 1:
                    tx, ty = dx, dy
                else:
                    tbl = [(dx, 0), (0, dy), (dx, dy)]
                    tx, ty = tbl[dev]
                ns[id_][0].append(tx)
                ns[id_][1].append(ty)
    return ns


def _build_jps2d_f1_f2() -> tuple[list[list[list[int]]], list[list[list[int]]]]:
    """f1 = offset to check if occupied; f2 = direction to jump if forced. Same as JPS2DNeib::FNeib."""
    f1: list[list[list[int]]] = [[[] for _ in range(2)] for _ in range(9)]
    f2: list[list[list[int]]] = [[[] for _ in range(2)] for _ in range(9)]
    for dy in range(-1, 2):
        for dx in range(-1, 2):
            id_ = (dx + 1) + 3 * (dy + 1)
            norm1 = abs(dx) + abs(dy)
            nf = _JPS2D_NSZ[norm1][1]
            for dev in range(nf):
                if norm1 == 1:
                    if dev == 0:
                        fx, fy = 0, 1
                    else:
                        fx, fy = 0, -1
                    if dx == 0:
                        fx, fy = fy, 0
                    nx, ny = dx + fx, dy + fy
                else:  # norm1 == 2
                    if dev == 0:
                        fx, fy = -dx, 0
                        nx, ny = -dx, dy
                    else:
                        fx, fy = 0, -dy
                        nx, ny = dx, -dy
                f1[id_][0].append(fx)
                f1[id_][1].append(fy)
                f2[id_][0].append(nx)
                f2[id_][1].append(ny)
    return f1, f2


_JPS2D_NS = _build_jps2d_ns()
_JPS2D_F1, _JPS2D_F2 = _build_jps2d_f1_f2()


@dataclass
class _JpsNode:
    """Grid node for JPS: (x, y), cost, parent index, and direction (dx, dy) for pruning."""

    x: int
    y: int
    cost: float
    parent_index: int
    dx: int = 0
    dy: int = 0


def _dir_id(dx: int, dy: int) -> int:
    """Direction id: (dx+1) + 3*(dy+1) for dx, dy in {-1, 0, 1}."""
    return (dx + 1) + 3 * (dy + 1)


class JPSPlanner:
    """Jump Point Search planner for uniform-cost 8-connected grids.

    When the environment map carries an occupancy grid (``env_map.grid``),
    collision checks use a fast O(1) grid lookup.  Otherwise, the planner
    falls back to Shapely geometry intersection (same as :class:`AStarPlanner`).
    """

    def __init__(self, env_map: EnvGridMap) -> None:
        """
        Initialize JPS planner.

        Args:
            env_map: Environment map (any :class:`~irsim.world.map.EnvGridMap`
                compatible object).  Resolution and bounds are taken from the
                map (same as :class:`AStarPlanner`).
        """
        self._map = env_map
        self.resolution = env_map.resolution
        self.origin_x = float(env_map.world_offset[0])
        self.origin_y = float(env_map.world_offset[1])
        self.min_x, self.min_y = 0, 0  # grid indices are 0-based
        self.max_x = self.origin_x + env_map.width
        self.max_y = self.origin_y + env_map.height
        self.x_width = round((self.max_x - self.origin_x) / self.resolution)
        self.y_width = round((self.max_y - self.origin_y) / self.resolution)
        self.obstacle_list = env_map.obstacle_list[:]

    def planning(
        self,
        start_pose: np.ndarray,
        goal_pose: np.ndarray,
        show_animation: bool = True,
    ) -> np.ndarray | None:
        """
        JPS path search.

        Args:
            start_pose (np.ndarray): start pose [x, y]
            goal_pose (np.ndarray): goal pose [x, y]
            show_animation (bool): If true, shows the animation of planning process

        Returns:
            np.ndarray | None: shape (2, N) array [rx, ry] of the final path, or None
            if the start or goal cell is not walkable, or if no path exists (open set
            exhausted).
        """
        sx = self.calc_xy_index(start_pose[0].item(), self.origin_x)
        sy = self.calc_xy_index(start_pose[1].item(), self.origin_y)
        start_node = _JpsNode(sx, sy, 0.0, -1, 0, 0)
        gx = self.calc_xy_index(goal_pose[0].item(), self.origin_x)
        gy = self.calc_xy_index(goal_pose[1].item(), self.origin_y)
        goal_node = _JpsNode(gx, gy, 0.0, -1)

        if not self._is_walkable(start_node.x, start_node.y):
            return None
        if not self._is_walkable(goal_node.x, goal_node.y):
            return None

        open_set: dict[int, _JpsNode] = {}
        closed_set: dict[int, _JpsNode] = {}
        open_set[self.calc_grid_index_from_xy(start_node.x, start_node.y)] = start_node

        while open_set:
            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost
                + self._heuristic(goal_node.x, goal_node.y, open_set[o].x, open_set[o].y),
            )
            current = open_set[c_id]

            if show_animation:  # pragma: no cover
                plt.plot(
                    self.calc_grid_position(current.x, self.origin_x),
                    self.calc_grid_position(current.y, self.origin_y),
                    "xc",
                )
                plt.gcf().canvas.mpl_connect(
                    "key_release_event",
                    lambda event: plt.close(event.canvas.figure) if event.key == "escape" else None,
                )
                if len(closed_set) % 10 == 0:
                    plt.pause(0.01)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for (jx, jy, dx, dy), move_cost in self._get_jps_successors(
                current, goal_node.x, goal_node.y
            ):
                node = _JpsNode(jx, jy, current.cost + move_cost, c_id, dx, dy)
                n_id = self.calc_grid_index_from_xy(jx, jy)

                if n_id in closed_set:
                    continue
                if n_id not in open_set or open_set[n_id].cost > node.cost:
                    open_set[n_id] = node

        if goal_node.parent_index == -1:
            print("Open set is empty..")
            return None
        rx, ry = self._calc_final_path(goal_node, closed_set)
        return np.array([rx, ry])

    def _get_jps_successors(self, current: _JpsNode, gx: int, gy: int) -> list[JpsSuccessor]:
        """Return list of ((jx, jy, dx, dy), cost) for each jump point successor (jps3d getJpsSucc style)."""
        dx, dy = current.dx, current.dy
        norm1 = abs(dx) + abs(dy)
        num_neib, num_fneib = _JPS2D_NSZ[norm1]
        id_ = _dir_id(dx, dy)
        x, y = current.x, current.y
        out: list[JpsSuccessor] = []

        for dev in range(num_neib + num_fneib):
            if dev < num_neib:
                ddx = _JPS2D_NS[id_][0][dev]
                ddy = _JPS2D_NS[id_][1][dev]
                if (jp := self._jump(x, y, ddx, ddy, gx, gy)) is None:
                    continue
                jx, jy = jp
            else:
                fn = dev - num_neib
                nx = x + _JPS2D_F1[id_][0][fn]
                ny = y + _JPS2D_F1[id_][1][fn]
                if not self._is_occupied(nx, ny):
                    continue
                ddx = _JPS2D_F2[id_][0][fn]
                ddy = _JPS2D_F2[id_][1][fn]
                if (jp := self._jump(x, y, ddx, ddy, gx, gy)) is None:
                    continue
                jx, jy = jp
            cost = math.hypot(jx - x, jy - y)
            out.append(((jx, jy, ddx, ddy), cost))
        return out

    def _jump(
        self,
        x: int,
        y: int,
        dx: int,
        dy: int,
        gx: int,
        gy: int,
    ) -> tuple[int, int] | None:
        """Jump from (x,y) in direction (dx,dy); return (jx, jy) or None. Uses iteration along the primary direction to avoid recursion depth limits."""
        nx, ny = x + dx, y + dy
        while True:
            if not self._is_walkable(nx, ny):
                return None
            # Match A*: no corner-cutting check (A* only verifies the target cell).
            if (nx, ny) == (gx, gy) or self._has_forced(nx, ny, dx, dy):
                return (nx, ny)

            id_ = _dir_id(dx, dy)
            norm1 = abs(dx) + abs(dy)
            num_neib = _JPS2D_NSZ[norm1][0]
            for k in range(num_neib - 1):
                ddx = _JPS2D_NS[id_][0][k]
                ddy = _JPS2D_NS[id_][1][k]
                if self._jump(nx, ny, ddx, ddy, gx, gy) is not None:
                    return (nx, ny)

            next_nx, next_ny = nx + dx, ny + dy
            if not self._is_walkable(next_nx, next_ny):
                return (nx, ny)
            nx, ny = next_nx, next_ny

    def _has_forced(self, x: int, y: int, dx: int, dy: int) -> bool:
        """True if (x,y) has a forced neighbor when approached along (dx,dy); uses f1 table."""
        id_ = _dir_id(dx, dy)
        for fn in range(_JPS2D_NSZ[abs(dx) + abs(dy)][1]):
            nx = x + _JPS2D_F1[id_][0][fn]
            ny = y + _JPS2D_F1[id_][1][fn]
            if self._is_occupied(nx, ny):
                return True
        return False

    def _is_occupied(self, ix: int, iy: int) -> bool:
        """True if grid cell (ix, iy) is in bounds and occupied."""
        if ix < 0 or iy < 0 or ix >= self.x_width or iy >= self.y_width:
            return False
        px = self.calc_grid_position(ix, self.origin_x)
        py = self.calc_grid_position(iy, self.origin_y)
        return self.is_collision(px, py)

    def _is_walkable(self, ix: int, iy: int) -> bool:
        """True if grid cell (ix, iy) is in bounds and not in collision."""
        if ix < 0 or iy < 0 or ix >= self.x_width or iy >= self.y_width:
            return False
        px = self.calc_grid_position(ix, self.origin_x)
        py = self.calc_grid_position(iy, self.origin_y)
        return not self.is_collision(px, py)

    def _heuristic(self, gx: int, gy: int, x: int, y: int) -> float:
        """Octile heuristic for 8-directional grid."""
        dx = abs(gx - x)
        dy = abs(gy - y)
        return (dx + dy) + (math.sqrt(2) - 1) * min(dx, dy)

    def _calc_final_path(self, goal_node: _JpsNode, closed_set: dict[int, _JpsNode]) -> tuple[list[float], list[float]]:
        """Build the final path with intermediate cells between jump points.

        JPS only stores jump points in ``closed_set``.  Between consecutive
        jump points, the path follows one of the 8 grid directions, so we
        interpolate all intermediate grid cells to produce a dense trajectory
        that stays on verified-walkable cells.
        """
        waypoints: list[tuple[int, int]] = [(goal_node.x, goal_node.y)]
        idx = goal_node.parent_index
        while idx != -1:
            n = closed_set[idx]
            waypoints.append((n.x, n.y))
            idx = n.parent_index

        rx, ry = [], []
        for (cx, cy), (px, py) in zip(waypoints[:-1], waypoints[1:]):
            ddx = 0 if px == cx else (1 if px > cx else -1)
            ddy = 0 if py == cy else (1 if py > cy else -1)
            ix, iy = cx, cy
            while (ix, iy) != (px, py):
                rx.append(self.calc_grid_position(ix, self.origin_x))
                ry.append(self.calc_grid_position(iy, self.origin_y))
                ix += ddx
                iy += ddy
        lx, ly = waypoints[-1]
        rx.append(self.calc_grid_position(lx, self.origin_x))
        ry.append(self.calc_grid_position(ly, self.origin_y))
        return rx, ry

    def calc_grid_position(self, index: int, min_position: float) -> float:
        return index * self.resolution + min_position

    def calc_xy_index(self, position: float, min_pos: float) -> int:
        return round((position - min_pos) / self.resolution)

    def calc_grid_index_from_xy(self, x: int, y: int) -> int:
        return y * self.x_width + x

    def is_collision(self, x: float, y: float) -> bool:
        """True if world position ``(x, y)`` is in collision.
        """
        shape = {"name": "rectangle", "length": self.resolution, "width": self.resolution}
        geometry = GeometryFactory.create_geometry(**shape).step(np.array([[x, y]]).T)
        return self._map.is_collision(geometry)
