"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

adapted by: Reinis Cimurs

"""

import math
import random
from typing import Any, Optional

import matplotlib.pyplot as plt
import numpy as np
import shapely

from irsim.lib.handler.geometry_handler import GeometryFactory


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x: float, y: float) -> None:
            """
            Initialize Node

            Args:
                x (float): x position of the node
                y (float): y position of the node
            """
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:
        """
        Area Bounds
        """

        def __init__(self, env_map: Any) -> None:
            """
            Initialize AreaBounds

            Args:
                env_map (EnvBase): environment where the planning will take place
            """
            self.xmin, self.ymin = 0, 0
            self.xmax, self.ymax = (
                env_map.width,
                env_map.height,
            )

    def __init__(
        self,
        env_map: Any,
        robot_radius: float,
        expand_dis: float = 1.0,
        path_resolution: float = 0.25,
        goal_sample_rate: int = 5,
        max_iter: int = 500,
    ) -> None:
        """
        Initialize RRT planner

        Args:
            env_map (Env): environment map where the planning will take place
            robot_radius (float): robot body modeled as circle with given radius
            expand_dis (float): expansion distance
            path_resolution (float): resolution of the path
            goal_sample_rate (int): goal sample rate
            max_iter (int): max iteration count
        """
        self.obstacle_list = env_map.obstacle_list[:]
        self.max_x, self.max_y = (
            env_map.width,
            env_map.height,
        )
        self.play_area = self.AreaBounds(env_map)
        self.min_rand = 0.0
        self.max_rand = max(self.max_x, self.max_y)
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = []
        self.robot_radius = robot_radius

    def planning(
        self,
        start_pose: list[float],
        goal_pose: list[float],
        show_animation: bool = True,
    ) -> Optional[tuple[list[float], list[float]]]:
        """
        rrt path planning

        Args:
            start_pose (np.array): start pose [x,y]
            goal_pose (np.array): goal pose [x,y]
            show_animation (bool): If true, shows the animation of planning process

        Returns:
            (np.array): xy position array of the final path
        """
        self.start = self.Node(float(start_pose[0]), float(start_pose[1]))
        self.end = self.Node(float(goal_pose[0]), float(goal_pose[1]))

        self.node_list = [self.start]
        for _i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_if_outside_play_area(
                new_node, self.play_area
            ) and self.check_collision(new_node, self.robot_radius):
                self.node_list.append(new_node)

                if show_animation:
                    self.draw_graph(new_node)

            if (
                self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y)
                <= self.expand_dis
            ):
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node, self.robot_radius):
                    return self.generate_final_course(len(self.node_list) - 1)

        return None  # cannot find path

    def steer(
        self, from_node: "Node", to_node: "Node", extend_length: float = float("inf")
    ) -> "Node":
        """
        Generate a new node by steering from `from_node` towards `to_node`.

        This method incrementally moves from `from_node` in the direction of `to_node`,
        using a fixed step size (`self.path_resolution`) and not exceeding the
        specified `extend_length`. The result is a new node that approximates a path
        from the start node toward the goal, constrained by resolution and maximum
        step distance.

        If the final position is within one resolution step of `to_node`, it snaps the
        new node exactly to `to_node`.

        Args:
            from_node (Node): The node from which to begin extending.
            to_node (Node): The target node to steer toward.
            extend_length (float, optional): The maximum length to extend. Defaults to infinity.

        Returns:
            (Node): A new node with updated position, path history (path_x, path_y),
        """
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind: int) -> tuple[list[float], list[float]]:
        """
        Generate the final path

        Args:
            goal_ind (int): index of the final goal

        Returns:
            (np.array): xy position array of the final path
        """
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        rx = [node[0] for node in path]
        ry = [node[1] for node in path]
        return np.array([rx, ry])

    def calc_dist_to_goal(self, x: float, y: float) -> float:
        """
        Calculate distance to goal

        Args:
            x (float): x coordinate of the position
            y (float): y coordinate of the position

        Returns:
            (float): distance to the goal
        """
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self) -> "Node":
        """
        Create random node

        Returns:
            (Node): new random node
        """
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand),
            )
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd: Optional["Node"] = None) -> None:
        plt.gcf().canvas.mpl_connect(
            "key_release_event",
            lambda event: [exit(0) if event.key == "escape" else None],
        )
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, "-r")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        if self.play_area is not None:
            plt.plot(
                [
                    self.play_area.xmin,
                    self.play_area.xmax,
                    self.play_area.xmax,
                    self.play_area.xmin,
                    self.play_area.xmin,
                ],
                [
                    self.play_area.ymin,
                    self.play_area.ymin,
                    self.play_area.ymax,
                    self.play_area.ymax,
                    self.play_area.ymin,
                ],
                "-k",
            )

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(
        x: float, y: float, size: float, color: str = "-b"
    ) -> None:  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list: list["Node"], rnd_node: "Node") -> int:
        dlist = [
            (node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2
            for node in node_list
        ]
        return dlist.index(min(dlist))

    @staticmethod
    def check_if_outside_play_area(node: "Node", play_area: "AreaBounds") -> bool:
        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        return not (
            node.x < play_area.xmin
            or node.x > play_area.xmax
            or node.y < play_area.ymin
            or node.y > play_area.ymax
        )

    def check_collision(self, node: "Node", robot_radius: float) -> bool:
        """
        Check if node is acceptable - free of collisions

        Args:
            node (Node): node to check
            robot_radius (float): robot radius

        Returns:
            (bool): True if there is no collision. False otherwise
        """

        if node is None:
            return False

        for i in range(len(node.path_x)):
            value = self.check_node(node.path_x[i], node.path_y[i], robot_radius)
            if value:
                return False  # collision
        return ~self.check_node(node.x, node.y, robot_radius)  # return True if safe

    def check_node(self, x: float, y: float, rr: float) -> bool:
        """
        Check positon for a collision

        Args:
            x (float): x value of the position
            y (float): y value of the position
            rr (float): robot radius

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

    @staticmethod
    def calc_distance_and_angle(
        from_node: "Node", to_node: "Node"
    ) -> tuple[float, float]:
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
