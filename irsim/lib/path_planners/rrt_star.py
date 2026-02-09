"""
RRT* (RRT-Star) path planner.

Collision precedence (inherited from :class:`RRT`):
  1. Grid lookup (O(1) per cell) when ``env_map.grid`` is not ``None``.
  2. Shapely geometry intersection when the grid is unavailable.

Reference:
    S. Karaman and E. Frazzoli, "Sampling-based Algorithms for Optimal
    Motion Planning," Int. J. Robotics Research, 2011.

Implementation reference:
    ZJU-FAST-Lab/sampling-based-path-finding
    https://github.com/ZJU-FAST-Lab/sampling-based-path-finding
    (C++ implementation with kd-tree nearest-neighbour and range queries,
    optimised parent selection, and BFS cost propagation.)

Adapted for ir-sim.
"""

from __future__ import annotations

import logging
import math
from typing import Any

import numpy as np

from irsim.lib.path_planners.rrt import RRT, TreeNode
from irsim.world.map import EnvGridMap

logger = logging.getLogger(__name__)


class RRTStar(RRT):
    """RRT* path planner with asymptotic optimality.

    Extends :class:`RRT` with:

    * **Neighbour search** — after steering, all nodes within a shrinking
      radius are considered as potential parents (``_choose_parent``).
    * **Rewiring** — after inserting a new node, nearby nodes are checked
      to see whether their cost can be reduced by re-parenting through
      the new node (``_rewire``).
    * **Efficient cost propagation** — ``_change_node_parent`` uses BFS
      through the ``children`` list for O(subtree) updates.
    """

    # Share the same node type
    Node = TreeNode

    def __init__(
        self,
        env_map: EnvGridMap,
        robot: Any,
        expand_dis: float = 1.5,
        path_resolution: float = 0.25,
        goal_sample_rate: int = 5,
        max_iter: int = 500,
        connect_circle_dist: float = 0.5,
        search_until_max_iter: bool = False,
    ) -> None:
        """Initialise the RRT* planner."""
        super().__init__(
            env_map,
            robot,
            expand_dis,
            path_resolution,
            goal_sample_rate,
            max_iter,
        )
        self.connect_circle_dist = connect_circle_dist
        self.search_until_max_iter = search_until_max_iter

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def planning(
        self,
        start_pose: list[float],
        goal_pose: list[float],
        show_animation: bool = True,
    ) -> np.ndarray | None:
        """RRT* path planning.

        Args:
            start_pose: Start position ``[x, y]``.
            goal_pose: Goal position ``[x, y]``.
            show_animation: Render the tree during planning.

        Returns:
            ``(2, N)`` waypoint array or *None*.
        """
        start_pose = np.asarray(start_pose, dtype=float).flatten()
        goal_pose = np.asarray(goal_pose, dtype=float).flatten()
        sx, sy = float(start_pose[0]), float(start_pose[1])
        gx, gy = float(goal_pose[0]), float(goal_pose[1])

        self.start = TreeNode(x=sx, y=sy, cost=0.0)
        self.end = TreeNode(x=gx, y=gy, cost=float("inf"))

        # Reset
        self.node_list = [self.start]
        self._kd_dirty = True
        self._vis_setup_done = False
        self._tree_line = None
        self._vis_temp = []

        goal_found = False

        for _i in range(self.max_iter):
            # 1. Sample
            rnd = self.get_random_node()

            # 2. Nearest node
            nearest_ind = self._nearest(rnd.x, rnd.y)
            nearest_node = self.node_list[nearest_ind]

            # 3. Steer
            new_node = self.steer(nearest_node, rnd, self.expand_dis)

            # 4. Bounds + collision
            if not self._check_bounds(new_node.x, new_node.y):
                continue
            if not self.is_collision(new_node):
                continue

            # 5. Choose best parent from neighbours
            cost_from_nearest = math.hypot(
                new_node.x - nearest_node.x,
                new_node.y - nearest_node.y,
            )
            near_inds = self._find_near_nodes(new_node.x, new_node.y)

            best_parent, best_cost_fp = self._choose_parent(
                new_node, nearest_node, nearest_ind, cost_from_nearest, near_inds,
            )

            # 6. Add to tree
            added = self._add_tree_node(
                best_parent,
                new_node.x,
                new_node.y,
                best_cost_fp,
                path_x=new_node.path_x,
                path_y=new_node.path_y,
            )
            # If parent changed, re-steer for the correct edge path
            if best_parent is not nearest_node:
                edge = self.steer(best_parent, added)
                added.path_x = edge.path_x
                added.path_y = edge.path_y

            # 7. Rewire
            self._rewire(added, near_inds)

            if show_animation:
                self.draw_graph(added)

            # 8. Try to connect to goal
            dist_to_goal = math.hypot(
                added.x - self.end.x, added.y - self.end.y,
            )
            if dist_to_goal <= self.expand_dis:
                goal_edge = self.steer(added, self.end, self.expand_dis)
                if self.is_collision(goal_edge):
                    new_goal_cost = added.cost + dist_to_goal
                    if new_goal_cost < self.end.cost:
                        if not goal_found:
                            # First connection — wire goal into the tree
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
                            # Cheaper route — re-parent the goal node
                            self._change_node_parent(
                                self.end, added, dist_to_goal,
                            )
                            self.end.path_x = goal_edge.path_x
                            self.end.path_y = goal_edge.path_y

            # Early termination
            if goal_found and not self.search_until_max_iter:
                return self._fill_path(self.end)

        if goal_found:
            logger.info(
                "[RRT*] path cost = %.3f, nodes = %d",
                self.end.cost,
                len(self.node_list),
            )
            return self._fill_path(self.end)

        logger.info(
            "[RRT*] no feasible path found after %d iterations", self.max_iter,
        )
        return None

    # ------------------------------------------------------------------
    # Choose parent (cost check before collision — reference optimisation)
    # ------------------------------------------------------------------

    def _choose_parent(
        self,
        new_node: TreeNode,
        nearest_node: TreeNode,
        nearest_ind: int,
        cost_from_nearest: float,
        near_inds: list[int],
    ) -> tuple[TreeNode, float]:
        """Select the lowest-cost parent for *new_node* from *near_inds*.

        The potential cost is checked **before** the expensive collision
        test so that unpromising candidates are skipped early.

        Returns:
            ``(best_parent, best_cost_from_parent)``
        """
        best_parent = nearest_node
        best_cost = nearest_node.cost + cost_from_nearest
        best_cost_fp = cost_from_nearest

        for idx in near_inds:
            if idx == nearest_ind:
                continue
            candidate = self.node_list[idx]
            d = math.hypot(candidate.x - new_node.x, candidate.y - new_node.y)
            potential_cost = candidate.cost + d

            # Potential cost first (cheap), collision second (expensive)
            if potential_cost >= best_cost:
                continue

            edge = self.steer(candidate, new_node)
            if self.is_collision(edge):
                best_parent = candidate
                best_cost = potential_cost
                best_cost_fp = d

        return best_parent, best_cost_fp

    # ------------------------------------------------------------------
    # Rewire
    # ------------------------------------------------------------------

    def _rewire(self, new_node: TreeNode, near_inds: list[int]) -> None:
        """Try to rewire nearby nodes through *new_node* for a lower cost.

        Uses :meth:`_change_node_parent` for efficient BFS cost propagation.
        """
        for idx in near_inds:
            candidate = self.node_list[idx]
            if candidate is new_node or candidate is self.start:
                continue

            d = math.hypot(
                new_node.x - candidate.x, new_node.y - candidate.y,
            )
            new_cost = new_node.cost + d
            if new_cost >= candidate.cost:
                continue

            edge = self.steer(new_node, candidate)
            if not self.is_collision(edge):
                continue

            self._change_node_parent(candidate, new_node, d)
            candidate.path_x = edge.path_x
            candidate.path_y = edge.path_y

    # ------------------------------------------------------------------
    # Find near nodes
    # ------------------------------------------------------------------

    def _find_near_nodes(self, x: float, y: float) -> list[int]:
        """Indices of nodes within the RRT* connection radius."""
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        r = min(r, self.expand_dis)
        return self._near_in_radius(x, y, r)
