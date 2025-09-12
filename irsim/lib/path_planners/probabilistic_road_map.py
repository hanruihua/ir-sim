"""

Probabilistic Road Map (PRM) Planner

author: Atsushi Sakai (@Atsushi_twi)

adapted by: Reinis Cimurs

"""

import math
from typing import Any, Optional

import matplotlib.pyplot as plt
import numpy as np
import shapely
from scipy.spatial import KDTree

from irsim.lib.handler.geometry_handler import GeometryFactory
from irsim.world.map import Map


class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x: float, y: float, cost: float, parent_index: int) -> None:
        """
        Initialize Node

        Args:
            x (float): x position of the node
            y (float): y position of the node
            cost (float): heuristic cost of the node
            parent_index (int): Nodes parent index
        """
        self.x = x
        self.y = y
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


class PRMPlanner:
    def __init__(
        self,
        env_map: Map,
        robot_radius: float,
        n_sample: int = 500,
        n_knn: int = 10,
        max_edge_len: float = 30.0,
    ) -> None:
        """
        Initialize the PRM planner.

        Args:
            env_map (Map): Environment map where planning takes place.
            robot_radius (float): Robot radius modeled as a circle.
            n_sample (int): Number of sampled points.
            n_knn (int): Number of nearest neighbors per node.
            max_edge_len (float): Maximum allowed edge length.
        """

        self.rr = robot_radius
        self.obstacle_list = env_map.obstacle_list[:]
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = (
            env_map.width,
            env_map.height,
        )
        self.n_sample = n_sample
        self.n_knn = n_knn
        self.max_edge_len = max_edge_len

    def planning(
        self,
        start_pose: np.ndarray,
        goal_pose: np.ndarray,
        rng: Optional[Any] = None,
        show_animation: bool = True,
    ) -> Optional[tuple[list[float], list[float]]]:
        """
        Plan a path from start to goal using the PRM method.

        Args:
            start_pose (np.array): start pose [x,y]
            goal_pose (np.array): goal pose [x,y]
            rng (Optional): Random generator
            show_animation (bool): If true, shows the animation of planning process

        Returns:
            (np.array): xy position array of the final path
        """
        start_x, start_y, goal_x, goal_y = (
            start_pose[0].item(),
            start_pose[1].item(),
            goal_pose[0].item(),
            goal_pose[1].item(),
        )
        sample_x, sample_y = self.sample_points(start_x, start_y, goal_x, goal_y, rng)
        if show_animation:
            plt.plot(sample_x, sample_y, ".b")

        road_map = self.generate_road_map(sample_x, sample_y)

        rx, ry = self.dijkstra_planning(
            start_x,
            start_y,
            goal_x,
            goal_y,
            road_map,
            sample_x,
            sample_y,
            show_animation,
        )

        return np.array([rx, ry])

    def check_node(self, x: float, y: float, rr: float) -> bool:
        """
        Check positon for a collision

        Args:
            x (float): x value of the position
            y (float): y value of the position

        Returns:
            (bool): True if there is a collision. False otherwise
        """
        node_position = [x, y]
        shape = {"name": "circle", "radius": rr}
        gf = GeometryFactory.create_geometry(**shape)
        geometry = gf.step(np.c_[node_position])
        return any(
            shapely.intersects(geometry, obj._geometry) for obj in self.obstacle_list
        )

    def is_collision(self, sx: float, sy: float, gx: float, gy: float) -> bool:
        """
        Check if line between points is acceptable - within edge limits and free of collisions

        Args:
            sx (float): start x position
            sy (float): start y position
            gx (float): goal x position
            gy (float): goal y position

        Returns:
            result (bool): True if node is not acceptable. False otherwise
        """
        x = sx
        y = sy
        dx = gx - sx
        dy = gy - sy
        yaw = math.atan2(gy - sy, gx - sx)
        d = math.hypot(dx, dy)

        if d >= self.max_edge_len:
            return True

        D = self.rr
        n_step = round(d / D)

        for _i in range(n_step):
            if self.check_node(x, y, self.rr):
                return True  # collision
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)

        # goal point check
        return self.check_node(gx, gy, self.rr)

    def generate_road_map(
        self, sample_x: list[float], sample_y: list[float]
    ) -> list[list[int]]:
        """
        Road map generation

        Args:
            sample_x (List): [m] x positions of sampled points
            sample_y (List): [m] y positions of sampled points

        Returns:
            road_map (List): list of edge ids
        """

        road_map = []
        n_sample = len(sample_x)
        sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

        for _i, ix, iy in zip(range(n_sample), sample_x, sample_y):
            _, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
            edge_id = []

            for ii in range(1, len(indexes)):
                nx = sample_x[indexes[ii]]
                ny = sample_y[indexes[ii]]

                if not self.is_collision(ix, iy, nx, ny):
                    edge_id.append(indexes[ii])

                if len(edge_id) >= self.n_knn:
                    break

            road_map.append(edge_id)

        # self.plot_road_map(road_map, sample_x, sample_y)

        return road_map

    @staticmethod
    def dijkstra_planning(
        sx: float,
        sy: float,
        gx: float,
        gy: float,
        road_map: list[list[int]],
        sample_x: list[float],
        sample_y: list[float],
        show_animation: bool,
    ) -> Optional[tuple[list[float], list[float]]]:
        """
        Args:
            sx (float): start x position [m]
            sy (float): start y position [m]
            gx (float): goal x position [m]
            gy (float): goal y position [m]
            road_map (list): list of edge ids
            sample_x (float): ??? [m]
            sample_y (float): ??? [m]

        Returns:
            (tuple(list, list)): Two lists of path coordinates ([x1, x2, ...], [y1, y2, ...]), empty list when no path was found
        """

        start_node = Node(sx, sy, 0.0, -1)
        goal_node = Node(gx, gy, 0.0, -1)

        open_set, closed_set = {}, {}
        open_set[len(road_map) - 2] = start_node

        path_found = True

        while True:
            if not open_set:
                print("Cannot find path")
                path_found = False
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # show graph
            if show_animation and len(closed_set.keys()) % 2 == 0:
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    "key_release_event",
                    lambda event: [exit(0) if event.key == "escape" else None],
                )
                plt.plot(current.x, current.y, "xg")
                plt.pause(0.001)

            if c_id == (len(road_map) - 1):
                print("goal is found!")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]
            # Add it to the closed set
            closed_set[c_id] = current

            # expand search grid based on motion model
            for i in range(len(road_map[c_id])):
                n_id = road_map[c_id][i]
                dx = sample_x[n_id] - current.x
                dy = sample_y[n_id] - current.y
                d = math.hypot(dx, dy)
                node = Node(sample_x[n_id], sample_y[n_id], current.cost + d, c_id)

                if n_id in closed_set:
                    continue
                # Otherwise if it is already in the open set
                if n_id in open_set:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id].cost = node.cost
                        open_set[n_id].parent_index = c_id
                else:
                    open_set[n_id] = node

        if path_found is False:
            return [], []

        # generate final course
        rx, ry = [goal_node.x], [goal_node.y]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x)
            ry.append(n.y)
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def plot_road_map(
        road_map: list[list[int]], sample_x: list[float], sample_y: list[float]
    ) -> None:  # pragma: no cover
        for i, _ in enumerate(road_map):
            for ii in range(len(road_map[i])):
                ind = road_map[i][ii]

                plt.plot(
                    [sample_x[i], sample_x[ind]], [sample_y[i], sample_y[ind]], "-k"
                )

    def sample_points(
        self, sx: float, sy: float, gx: float, gy: float, rng: Optional[Any]
    ) -> tuple[list[float], list[float]]:
        """
        Generate sample points

        Args:
            sx (float): start x position [m]
            sy (float): start y position [m]
            gx (float): goal x position [m]
            gy (float): goal y position [m]
            rng: Random generator

        Returns:
            sample (tuple (list, list)): sample positions
        """
        sample_x, sample_y = [], []

        if rng is None:
            rng = np.random.default_rng()

        while len(sample_x) <= self.n_sample:
            tx = (rng.random() * (self.max_x - self.min_x)) + self.min_x
            ty = (rng.random() * (self.max_y - self.min_y)) + self.min_y

            if not self.check_node(tx, ty, self.rr):
                sample_x.append(tx)
                sample_y.append(ty)

        sample_x.append(sx)
        sample_y.append(sy)
        sample_x.append(gx)
        sample_y.append(gy)

        return sample_x, sample_y
