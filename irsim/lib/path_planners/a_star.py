"""
A* grid planning.

Collision precedence:
  1. Grid lookup when ``env_map.grid`` is not ``None``; if occupied, collision.
  2. When the grid reports free or is unavailable, Shapely vs. obstacle_list.
  (Grid and obstacle_list are combined when both are present.)

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

adapted by: Reinis Cimurs

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

from __future__ import annotations

import math

import matplotlib.pyplot as plt
import numpy as np

from irsim.lib.handler.geometry_handler import GeometryFactory
from irsim.world.map import EnvGridMap


class AStarPlanner:
    def __init__(self, env_map: EnvGridMap) -> None:
        """
        Initialize A* planner.

        Args:
            env_map: Environment map (any :class:`~irsim.world.map.EnvGridMap`
                compatible object).
        """

        self._map = env_map
        self.resolution = env_map.resolution  # m/cell
        self.obstacle_list = env_map.obstacle_list[:]
        self.origin_x = float(env_map.world_offset[0])
        self.origin_y = float(env_map.world_offset[1])
        self.min_x, self.min_y = 0, 0  # grid indices are 0-based
        self.max_x = self.origin_x + env_map.width
        self.max_y = self.origin_y + env_map.height
        self.x_width = round((self.max_x - self.origin_x) / self.resolution)
        self.y_width = round((self.max_y - self.origin_y) / self.resolution)
        self.motion = self.get_motion_model()

    class Node:
        """Node class"""

        def __init__(self, x: int, y: int, cost: float, parent_index: int) -> None:
            """
            Initialize Node

            Args:
                x (float): x position of the node
                y (float): y position of the node
                cost (float): heuristic cost of the node
                parent_index (int): Nodes parent index
            """

            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self) -> str:
            """str function for Node class"""
            return (
                str(self.x)
                + ","
                + str(self.y)
                + ","
                + str(self.cost)
                + ","
                + str(self.parent_index)
            )

    def planning(
        self,
        start_pose: np.ndarray,
        goal_pose: np.ndarray,
        show_animation: bool = True,
    ) -> tuple[list[float], list[float]]:
        """
        A star path search

        Args:
            start_pose (np.array): start pose [x,y]
            goal_pose (np.array): goal pose [x,y]
            show_animation (bool): If true, shows the animation of planning process

        Returns:
            (np.array): xy position array of the final path
        """
        start_node = self.Node(
            self.calc_xy_index(start_pose[0].item(), self.origin_x),
            self.calc_xy_index(start_pose[1].item(), self.origin_y),
            0.0,
            -1,
        )
        goal_node = self.Node(
            self.calc_xy_index(goal_pose[0].item(), self.origin_x),
            self.calc_xy_index(goal_pose[1].item(), self.origin_y),
            0.0,
            -1,
        )

        open_set, closed_set = {}, {}
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost
                + self.calc_heuristic(goal_node, open_set[o]),
            )
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(
                    self.calc_grid_position(current.x, self.origin_x),
                    self.calc_grid_position(current.y, self.origin_y),
                    "xc",
                )
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    "key_release_event",
                    lambda event: plt.close(event.canvas.figure) if event.key == "escape" else None,
                )
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(
                    current.x + self.motion[i][0],
                    current.y + self.motion[i][1],
                    current.cost + self.motion[i][2],
                    c_id,
                )
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return np.array([rx, ry])

    def calc_final_path(
        self, goal_node: "Node", closed_set: dict
    ) -> tuple[list[float], list[float]]:
        """Generate the final path

        Args:
            goal_node (Node): final goal node
            closed_set (dict): dict of closed nodes

        Returns:
            rx (list): list of x positions of final path
            ry (list): list of y positions of final path
        """
        rx, ry = (
            [self.calc_grid_position(goal_node.x, self.origin_x)],
            [self.calc_grid_position(goal_node.y, self.origin_y)],
        )
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.origin_x))
            ry.append(self.calc_grid_position(n.y, self.origin_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1: "Node", n2: "Node") -> float:
        w = 1.0  # weight of heuristic
        return w * math.hypot(n1.x - n2.x, n1.y - n2.y)

    def calc_grid_position(self, index: int, min_position: float) -> float:
        """
        calc grid position

        Args:
            index (int): index of a node
            min_position (float): min value of search space

        Returns:
            (float): position of coordinates along the given axis
        """
        return index * self.resolution + min_position

    def calc_xy_index(self, position: float, min_pos: float) -> int:
        """
        calc xy index of node

        Args:
            position (float): position of a node
            min_pos (float): min value of search space

        Returns:
            (int): index of position along the given axis
        """
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node: "Node") -> int:
        """
        calc grid index of node

        Args:
            node (Node): node to calculate the index for

        Returns:
            (float): grid index of the node
        """
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node: "Node") -> bool:
        """
        Check if node is acceptable - within limits of search space and free of collisions

        Args:
            node (Node): node to check

        Returns:
            (bool): True if node is acceptable. False otherwise
        """
        px = self.calc_grid_position(node.x, self.origin_x)
        py = self.calc_grid_position(node.y, self.origin_y)

        if (
            px < self.origin_x
            or py < self.origin_y
            or px >= self.max_x
            or py >= self.max_y
        ):
            return False

        # collision check
        return not self.check_node(px, py)

    def check_node(self, x: float, y: float) -> bool:
        """Check position for a collision.

        Args:
            x: World x coordinate of the cell centre.
            y: World y coordinate of the cell centre.

        Returns:
            ``True`` if a collision is detected.
        """
        node_position = [x, y]
        shape = {
            "name": "rectangle",
            "length": self.resolution,
            "width": self.resolution,
        }
        gf = GeometryFactory.create_geometry(**shape)
        geometry = gf.step(np.c_[node_position])
        return self._map.is_collision(geometry)

    @staticmethod
    def get_motion_model() -> list[list[float]]:
        # dx, dy, cost
        return [
            [1, 0, 1],
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
            [-1, -1, math.sqrt(2)],
            [-1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)],
            [1, 1, math.sqrt(2)],
        ]
