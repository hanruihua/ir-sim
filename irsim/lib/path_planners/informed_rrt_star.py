"""
Informed RRT* path planner.

Collision precedence (inherited from :class:`RRT`):
  1. Grid lookup (O(1) per cell) when ``env_map.grid`` is not ``None``.
  2. Shapely geometry intersection when the grid is unavailable.

Informed RRT* improves upon RRT* by constraining the sampling region to an
ellipsoidal subset of the planning space once an initial solution has been
found.  The ellipse is defined by the start and goal positions as foci, and
the current best path cost determines its size.  As the cost decreases, the
ellipse shrinks, focusing exploration on the region that can actually produce
shorter paths.

Reference:
    J. D. Gammell, S. S. Srinivasa, and T. D. Barfoot,
    "Informed RRT*: Optimal Sampling-based Path Planning Focused via Direct
    Sampling of an Admissible Ellipsoidal Heuristic," in Proc. IEEE/RSJ
    Int. Conf. Intelligent Robots and Systems (IROS), 2014.

Implementation reference:
    ZJU-FAST-Lab/sampling-based-path-finding
    https://github.com/ZJU-FAST-Lab/sampling-based-path-finding
    (C++ implementation of RRT* with informed sampling and ellipsoidal
    heuristic.)

Adapted for ir-sim.
"""

from __future__ import annotations

import logging
import math
import random
from typing import TYPE_CHECKING, Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon as MplPolygon

from irsim.lib.path_planners.rrt import TreeNode
from irsim.lib.path_planners.rrt_star import RRTStar
from irsim.world.map import EnvGridMap

if TYPE_CHECKING:
    from matplotlib.lines import Line2D

logger = logging.getLogger(__name__)


class InformedRRTStar(RRTStar):
    """Informed RRT* path planner.

    After finding an initial feasible path the sampler switches from uniform
    random sampling to sampling inside an informed ellipsoidal set whose
    foci are the start and goal.  This dramatically speeds up convergence
    towards the optimal path.
    """

    # Share the same node type
    Node = TreeNode

    def __init__(
        self,
        env_map: EnvGridMap,
        robot: Any,
        expand_dis: float = 1.5,
        path_resolution: float = 0.25,
        goal_sample_rate: int = 10,
        max_iter: int = 500,
        connect_circle_dist: float = 50.0,
        search_until_max_iter: bool = True,
    ) -> None:
        """Initialise the Informed RRT* planner."""
        super().__init__(
            env_map,
            robot,
            expand_dis,
            path_resolution,
            goal_sample_rate,
            max_iter,
            connect_circle_dist,
            search_until_max_iter=search_until_max_iter,
        )

        # -- informed-sampling state --
        self._best_cost: float = float("inf")
        self._best_path: np.ndarray | None = None
        self._c_min: float = 0.0
        self._x_center: np.ndarray = np.zeros(2)
        self._rotation_matrix: np.ndarray = np.eye(2)

        # -- visualisation (persistent artists) --
        self._vis_setup_done: bool = False
        self._start_marker: Line2D | None = None
        self._goal_marker: Line2D | None = None
        self._ellipse_fill: MplPolygon | None = None
        self._ellipse_line: Line2D | None = None
        self._best_path_line: Line2D | None = None
        self._cost_history: list[float] = []

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def planning(
        self,
        start_pose: list[float],
        goal_pose: list[float],
        show_animation: bool = True,
    ) -> np.ndarray | None:
        """Informed RRT* path planning.

        Args:
            start_pose: Start position ``[x, y]``.
            goal_pose: Goal position ``[x, y]``.
            show_animation: Render tree, ellipse and best path during
                planning.

        Returns:
            ``(2, N)`` waypoint array or *None*.
        """
        sx, sy = float(start_pose[0]), float(start_pose[1])
        gx, gy = float(goal_pose[0]), float(goal_pose[1])

        self.start = TreeNode(x=sx, y=sy, cost=0.0)
        self.end = TreeNode(x=gx, y=gy, cost=float("inf"))

        # -- ellipse invariants --
        self._c_min = math.hypot(gx - sx, gy - sy)
        self._x_center = np.array([(sx + gx) / 2.0, (sy + gy) / 2.0])
        theta = math.atan2(gy - sy, gx - sx)
        self._rotation_matrix = np.array(
            [
                [math.cos(theta), -math.sin(theta)],
                [math.sin(theta), math.cos(theta)],
            ]
        )

        self._best_cost = float("inf")
        self._best_path = None
        self._cost_history = []

        # -- reset vis (new run: re-do one-time setup for new start/goal) --
        self._tree_line = None
        self._start_marker = None
        self._goal_marker = None
        self._ellipse_fill = None
        self._ellipse_line = None
        self._best_path_line = None
        self._vis_setup_done = False

        # -- reset tree --
        self.node_list = [self.start]
        self._kd_dirty = True

        goal_found = False

        for i in range(self.max_iter):
            # 1. Informed sample
            rnd = self._informed_sample()

            # 2. Nearest
            nearest_ind = self._nearest(rnd.x, rnd.y)
            nearest_node = self.node_list[nearest_ind]

            # 3. Steer
            new_node = self.steer(nearest_node, rnd, self.expand_dis)

            # 4. Check
            if not self._check_bounds(new_node.x, new_node.y):
                continue
            if not self.is_collision(new_node):
                continue

            # 5. Choose parent
            cost_from_nearest = math.hypot(
                new_node.x - nearest_node.x,
                new_node.y - nearest_node.y,
            )
            near_inds = self._find_near_nodes(new_node.x, new_node.y)
            best_parent, best_cost_fp = self._choose_parent(
                new_node,
                nearest_node,
                nearest_ind,
                cost_from_nearest,
                near_inds,
            )

            # 6. Add
            added = self._add_tree_node(
                best_parent,
                new_node.x,
                new_node.y,
                best_cost_fp,
                path_x=new_node.path_x,
                path_y=new_node.path_y,
            )
            if best_parent is not nearest_node:
                edge = self.steer(best_parent, added)
                added.path_x = edge.path_x
                added.path_y = edge.path_y

            # 7. Rewire
            self._rewire(added, near_inds)

            # 8. Try goal connection
            dist_to_goal = math.hypot(
                added.x - self.end.x, added.y - self.end.y,
            )
            if dist_to_goal <= self.expand_dis:
                goal_edge = self.steer(added, self.end, self.expand_dis)
                if self.is_collision(goal_edge):
                    new_goal_cost = added.cost + dist_to_goal
                    if new_goal_cost < self.end.cost:
                        if not goal_found:
                            self.end.cost = new_goal_cost
                            self.end.cost_from_parent = dist_to_goal
                            self.end.parent = added
                            self.end.path_x = goal_edge.path_x
                            self.end.path_y = goal_edge.path_y
                            self.end.children = []
                            added.children.append(self.end)
                            self.node_list.append(self.end)
                            self._kd_dirty = True
                            goal_found = True
                        else:
                            self._change_node_parent(
                                self.end, added, dist_to_goal,
                            )
                            self.end.path_x = goal_edge.path_x
                            self.end.path_y = goal_edge.path_y

            # 9. Track best path
            if goal_found:
                path = self._fill_path(self.end)
                cost = self._path_cost(path)
                if cost < self._best_cost:
                    old_cost = self._best_cost
                    self._best_cost = cost
                    self._best_path = path
                    self._cost_history.append(cost)
                    if old_cost == float("inf"):
                        logger.info(
                            "[Informed RRT*] iter %d: first path, "
                            "cost=%.3f (c_min=%.3f)",
                            i,
                            cost,
                            self._c_min,
                        )
                    else:
                        logger.info(
                            "[Informed RRT*] iter %d: improved "
                            "%.3f -> %.3f (delta=%.3f)",
                            i,
                            old_cost,
                            cost,
                            old_cost - cost,
                        )

            # 10. Draw (throttled)
            if show_animation and (i % 10 == 0 or i == self.max_iter - 1):
                self._draw_informed(i)

        # -- summary --
        if self._best_path is not None:
            logger.info(
                "[Informed RRT*] done — cost=%.3f, improvements=%d, nodes=%d",
                self._best_cost,
                len(self._cost_history),
                len(self.node_list),
            )
            return self._best_path

        if goal_found:
            return self._fill_path(self.end)

        logger.info("[Informed RRT*] no feasible path found")
        return None

    # ------------------------------------------------------------------
    # Informed sampling
    # ------------------------------------------------------------------

    def _informed_sample(self) -> TreeNode:
        """Sample from the ellipse if a path exists, else goal-biased uniform."""
        if self._best_cost < float("inf"):
            return self._sample_from_ellipse()
        return self.get_random_node()

    def _sample_from_ellipse(self) -> TreeNode:
        """Uniform sample from the 2-D informed ellipse."""
        a = self._best_cost / 2.0
        b = math.sqrt(max(a * a - (self._c_min / 2.0) ** 2, 0.0))

        # Uniform sample inside the unit disk
        ang = random.uniform(0, 2 * math.pi)
        r = math.sqrt(random.uniform(0, 1))
        x_ball = np.array([r * math.cos(ang), r * math.sin(ang)])

        # Scale -> rotate -> translate
        sample = (
            self._rotation_matrix @ np.array([a * x_ball[0], b * x_ball[1]])
            + self._x_center
        )

        # Clamp to play area
        pa = self.play_area
        sample[0] = max(pa.xmin, min(sample[0], pa.xmax))
        sample[1] = max(pa.ymin, min(sample[1], pa.ymax))

        return TreeNode(x=float(sample[0]), y=float(sample[1]))

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------

    def _draw_informed(self, iteration: int) -> None:
        """Draw tree, ellipse and best path (persistent artists)."""
        ax = plt.gca()

        # --- one-time setup ---
        if not self._vis_setup_done:
            ax.figure.canvas.mpl_connect(
                "key_release_event",
                lambda event: plt.close(event.canvas.figure) if event.key == "escape" else None,
            )
            (self._start_marker,) = ax.plot(
                self.start.x,
                self.start.y,
                "o",
                color="tab:green",
                markersize=8,
                zorder=5,
                label="start",
            )
            (self._goal_marker,) = ax.plot(
                self.end.x,
                self.end.y,
                "o",
                color="tab:red",
                markersize=8,
                zorder=5,
                label="goal",
            )
            self._vis_setup_done = True

        # --- tree edges ---
        xs: list[float] = []
        ys: list[float] = []
        for node in self.node_list:
            if node.parent and node.path_x:
                xs.extend(node.path_x)
                ys.extend(node.path_y)
                xs.append(float("nan"))
                ys.append(float("nan"))

        if self._tree_line is None and xs:
            (self._tree_line,) = ax.plot(
                xs,
                ys,
                "-",
                color="tab:green",
                alpha=0.35,
                linewidth=0.6,
            )
        elif self._tree_line is not None:
            self._tree_line.set_data(xs, ys)

        # --- informed sampling ellipse ---
        if self._best_cost < float("inf"):
            self._update_ellipse(ax)
        else:
            if self._ellipse_line is not None:
                self._ellipse_line.set_data([], [])
            if self._ellipse_fill is not None:
                self._ellipse_fill.set_xy(np.empty((0, 2)))

        # --- best path ---
        if self._best_path is not None:
            if self._best_path_line is None:
                (self._best_path_line,) = ax.plot(
                    self._best_path[0],
                    self._best_path[1],
                    "-",
                    color="tab:red",
                    linewidth=2.0,
                    zorder=4,
                    label="best path",
                )
            else:
                self._best_path_line.set_data(
                    self._best_path[0],
                    self._best_path[1],
                )

        # --- title & legend ---
        title = f"Informed RRT*  —  iter {iteration}/{self.max_iter}"
        if self._best_cost < float("inf"):
            title += f"  |  cost = {self._best_cost:.2f}"
            title += f"  |  c_min = {self._c_min:.2f}"
        else:
            title += "  |  searching …"
        ax.set_title(title, fontsize=9)
        ax.legend(loc="upper left", fontsize=7)

        plt.pause(0.001)

    def _update_ellipse(self, ax: plt.Axes) -> None:
        """Create or update the informed sampling ellipse on *ax*."""
        a = self._best_cost / 2.0
        b = math.sqrt(max(a * a - (self._c_min / 2.0) ** 2, 0.0))

        t = np.linspace(0, 2 * np.pi, 80)
        pts = (
            self._rotation_matrix @ np.vstack([a * np.cos(t), b * np.sin(t)])
            + self._x_center[:, None]
        )
        xy = pts.T  # (N, 2)

        if self._ellipse_line is None:
            (self._ellipse_line,) = ax.plot(
                pts[0],
                pts[1],
                "--",
                color="tab:blue",
                linewidth=1.2,
                label="informed set",
            )
            self._ellipse_fill = MplPolygon(
                xy,
                closed=True,
                color="tab:blue",
                alpha=0.08,
            )
            ax.add_patch(self._ellipse_fill)
        else:
            self._ellipse_line.set_data(pts[0], pts[1])
            self._ellipse_fill.set_xy(xy)

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    @staticmethod
    def _path_cost(path: np.ndarray) -> float:
        """Total Euclidean length of a ``(2, N)`` path array."""
        diffs = np.diff(path, axis=1)
        return float(np.sum(np.hypot(diffs[0], diffs[1])))
