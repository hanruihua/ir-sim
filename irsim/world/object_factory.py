from typing import Any

import numpy as np

from irsim.lib.handler.kinematics_handler import KinematicsFactory
from irsim.util.random import random_uniform
from irsim.util.util import (
    convert_list_length,
    convert_list_length_dict,
)
from irsim.world.map.obstacle_map import ObstacleMap
from irsim.world.object_base import ObjectBase

# Keep backward-compatible imports so existing code can still reach these
# via ``from irsim.world.object_factory import RobotDiff`` etc.
from irsim.world.obstacles.obstacle_acker import ObstacleAcker  # noqa: F401
from irsim.world.obstacles.obstacle_diff import ObstacleDiff  # noqa: F401
from irsim.world.obstacles.obstacle_omni import ObstacleOmni  # noqa: F401
from irsim.world.obstacles.obstacle_static import ObjectStatic
from irsim.world.robots.robot_acker import RobotAcker  # noqa: F401
from irsim.world.robots.robot_diff import RobotDiff  # noqa: F401
from irsim.world.robots.robot_omni import RobotOmni  # noqa: F401


class ObjectFactory:
    """
    Factory class for creating various objects in the simulation.
    """

    def __init__(self, world: Any = None) -> None:
        self.world = world

    def create_from_parse(
        self,
        parse: list[dict[str, Any]] | dict[str, Any],
        obj_type: str = "robot",
        group_start_index: int = 0,
    ) -> list[Any]:
        """
        Create objects from a parsed configuration.

        Args:
            parse (list or dict): Parsed configuration data.
            obj_type (str): Type of object to create, 'robot' or 'obstacle'.
            group_start_index (int): Starting index for the group.

        Returns:
            list: List of created objects.
        """
        object_list = []

        if isinstance(parse, list):
            object_list = [
                obj
                for group_index, sp in enumerate(parse)
                for obj in self.create_object(
                    obj_type, **{"group": group_start_index + group_index, **sp}
                )
            ]

        elif isinstance(parse, dict):
            object_list = list(self.create_object(obj_type, **parse))

        return object_list

    def create_from_map(
        self,
        points: np.ndarray,
        reso: np.ndarray | None = None,
        grid_map: np.ndarray | None = None,
        world_offset: list[float] | None = None,
    ) -> list[Any]:
        """
        Create map objects from points.

        Args:
            points (np.ndarray): (2, N) array of obstacle cell positions.
            reso (np.ndarray): (2, 1) array of [x_reso, y_reso] cell sizes.
            grid_map (np.ndarray, optional): Grid map array for fast collision
                detection. If None, no precomputed grid is used.
            world_offset (list[float], optional): World offset [x, y].
                If None, no additional world offset is applied.

        Returns:
            list: List of ObstacleMap objects.
        """
        if points is None or points.size == 0:
            return []
        return [
            ObstacleMap(
                shape={"name": "map", "points": points, "reso": reso},
                color="k",
                grid_map=grid_map,
                grid_reso=reso,
                world_offset=world_offset,
            )
        ]

    def create_object(
        self,
        obj_type: str = "robot",
        number: int = 1,
        distribution: dict[str, Any] | None = None,
        state: list[float] | None = None,
        goal: list[float] | None = None,
        **kwargs: Any,
    ) -> list[Any]:
        """
        Create multiple objects based on the parameters.

        Args:
            obj_type (str): Type of object, 'robot' or 'obstacle'.
            number (int): Number of objects to create.
            distribution (dict): Distribution type for generating states.
            state (list): Initial state for objects.
            goal (list): Goal state for objects.
            **kwargs: Additional parameters for object creation.

        Returns:
            list: List of created objects.
        """
        if distribution is None:
            distribution = {"name": "manual"}
        if state is None:
            state = [1, 1, 0]

        if not distribution.get("3d", False):
            state_list, goal_list = self.generate_state_list(
                number, distribution, state, goal
            )
        else:
            raise NotImplementedError(
                "3D state generation is not yet implemented. "
                "Please set '3d: false' in the distribution configuration."
            )

        object_list = []

        for i in range(number):
            obj_dict = {
                k: convert_list_length(v, number)[i]
                for k, v in kwargs.items()
                if k != "sensors"
            }
            obj_dict["state"] = state_list[i]
            obj_dict["goal"] = goal_list[i]
            sensors: list[Any] = kwargs.get("sensors") or []
            obj_dict["sensors"] = convert_list_length_dict(sensors, number)[i]

            if obj_type == "robot":
                object_list.append(self.create_robot(**obj_dict))
            elif obj_type == "obstacle":
                object_list.append(self.create_obstacle(**obj_dict))

        return object_list

    def create_robot(
        self, kinematics: dict[str, Any] | None = None, **kwargs: Any
    ) -> Any:
        """
        Create a robot based on kinematics.

        Uses the kinematics registry to look up handler-class metadata
        (default color, state_dim, description) and creates an ``ObjectBase``
        directly.  Static / ``None`` kinematics still produce an
        ``ObjectStatic``.

        Args:
            kinematics (dict): Kinematics configuration.
            **kwargs: Additional parameters for robot creation.

        Returns:
            ObjectBase: An instance of a robot.
        """
        if kinematics is None:
            kinematics = {}
        kinematics_name = kinematics.get("name")

        if kinematics_name == "static" or kinematics_name is None:
            return ObjectStatic(kinematics=kinematics, role="robot", **kwargs)

        handler_cls = KinematicsFactory.get_handler_class(kinematics_name)
        if handler_cls is None:
            raise NotImplementedError(
                f"Robot kinematics {kinematics_name} not implemented"
            )

        kwargs.setdefault("color", handler_cls.color)
        kwargs.setdefault("state_dim", handler_cls.state_dim)
        if handler_cls.description is not None:
            kwargs.setdefault("description", handler_cls.description)

        return ObjectBase(kinematics=kinematics, role="robot", **kwargs)

    def create_obstacle(
        self, kinematics: dict[str, Any] | None = None, **kwargs: Any
    ) -> Any:
        """
        Create an obstacle based on kinematics.

        Uses the kinematics registry to look up handler-class metadata
        (default color, state_dim) and creates an ``ObjectBase`` directly.
        Static / ``None`` kinematics still produce an ``ObjectStatic``.

        Args:
            kinematics (dict): Kinematics configuration.
            **kwargs: Additional parameters for obstacle creation.

        Returns:
            ObjectBase: An instance of an obstacle.
        """
        if kinematics is None:
            kinematics = {}
        kinematics_name = kinematics.get("name")

        if kinematics_name == "static" or kinematics_name is None:
            return ObjectStatic(kinematics=kinematics, role="obstacle", **kwargs)

        handler_cls = KinematicsFactory.get_handler_class(kinematics_name)
        if handler_cls is None:
            raise NotImplementedError(
                f"Obstacle kinematics {kinematics_name} not implemented"
            )

        kwargs.setdefault("color", handler_cls.obstacle_color)
        kwargs.setdefault("state_dim", handler_cls.state_dim)

        return ObjectBase(kinematics=kinematics, role="obstacle", **kwargs)

    def generate_state_list(
        self,
        number: int = 1,
        distribution: dict[str, Any] | None = None,
        state: list[float] | None = None,
        goal: list[float] | None = None,
    ) -> tuple[list[list[float]], list[list[float]]]:
        """
        Generate a list of state vectors for multiple objects based on the specified distribution method.

        This function creates initial states for multiple objects in the simulation environment.
        It supports various distribution methods such as 'manual', 'circle', and 'random' to
        position the objects according to specific patterns or randomness.

        Defaults for the ``circle`` and ``random`` distributions are derived
        from the world attached to the factory (``self.world``). If no world
        is attached (e.g. in unit tests that instantiate ``ObjectFactory()``
        directly), defaults fall back to a 10x10 world at offset ``[0, 0]``.

        Args:
            number (int):
                Number of state vectors to generate. Default is 1.
            distribution (Dict[str, Any]):
                Configuration dictionary specifying the distribution method and its parameters.
                Default is {"name": "manual"}.

                - 'name' (str): Name of the distribution method. Supported values are:

                  - 'manual': States are specified manually.
                  - 'circle': States are arranged in a circular pattern.
                  - 'random': States are placed at random positions.

                - Additional parameters depend on the distribution method:

                  - For 'manual': Manually specified states and goal.
                  - For 'circle':

                    - 'center' (List[float]): Center coordinates [x, y] of the
                      circle. Default is the world center
                      ``[offset_x + width / 2, offset_y + height / 2]``.
                    - 'radius' (float): Radius of the circle. Default is
                      ``min(width, height) / 2 - 0.5`` so the circle sits
                      inside the world with a small margin.

                  - For 'random':

                    - 'range_low' (List[float]): Lower bounds ``[x, y, theta]``
                      for random state values. Default is
                      ``[offset_x + 0.5, offset_y + 0.5, -pi]`` (the world
                      bounds inset by 0.5).
                    - 'range_high' (List[float]): Upper bounds ``[x, y, theta]``
                      for random state values. Default is
                      ``[offset_x + width - 0.5, offset_y + height - 0.5, pi]``.
                    - 'min_distance' (float): Minimum pairwise xy distance
                      between sampled points. Default is 1.0.

            state (List[float]):
                Base state vector [x, y, theta] used as a template when
                ``distribution['name'] == 'manual'``. Default is [1, 1, 0].
            goal (List[float]):
                Goal state vector [x, y, theta] used when
                ``distribution['name'] == 'manual'``. Default is [1, 9, 0].

        Returns:
            tuple[list[list[float]], list[list[float]]]:
                A pair ``(state_list, goal_list)`` where each element is a list of
                3-element state vectors ``[x, y, theta]`` for every generated object.

        Raises:
            ValueError:
                If the distribution method specified in 'name' is not supported or if required
                parameters for a distribution method are missing.
        """
        if distribution is None:
            distribution = {"name": "manual"}
        if state is None:
            state = [1, 1, 0]
        if goal is None:
            goal = [1, 9, 0]

        width = self.world.width if self.world is not None else 10.0
        height = self.world.height if self.world is not None else 10.0
        offset = self.world.offset if self.world is not None else [0.0, 0.0]
        margin = 0.5

        if distribution["name"] == "manual":
            state_list = convert_list_length(state, number)
            goal_list = convert_list_length(goal, number)

        elif distribution["name"] == "random":
            default_low = [offset[0] + margin, offset[1] + margin, -np.pi]
            default_high = [
                offset[0] + width - margin,
                offset[1] + height - margin,
                np.pi,
            ]
            range_low = distribution.get("range_low", default_low)
            range_high = distribution.get("range_high", default_high)
            min_distance = distribution.get("min_distance", 1.0)

            state_array = random_uniform(
                range_low, range_high, size=(3, number), min_distance=min_distance
            )
            state_list = state_array.T.tolist()

            goal_array = random_uniform(
                range_low, range_high, size=(3, number), min_distance=min_distance
            )
            goal_list = goal_array.T.tolist()

        elif distribution["name"] == "uniform":
            raise NotImplementedError(
                "The 'uniform' distribution is not yet implemented. "
                "Use 'random' or 'circle' distribution instead."
            )

        elif distribution["name"] == "circle":
            default_radius = min(width, height) / 2 - margin
            default_center = [offset[0] + width / 2, offset[1] + height / 2, 0]
            radius = distribution.get("radius", default_radius)
            center = distribution.get("center", default_center)

            state_list, goal_list = [], []
            for i in range(number):
                theta = 2 * np.pi * i / number
                x = center[0] + radius * np.cos(theta)
                y = center[1] + radius * np.sin(theta)
                state_list.append([x, y, theta - np.pi])

                goal_x = center[0] - radius * np.cos(theta)
                goal_y = center[1] - radius * np.sin(theta)
                goal_list.append([goal_x, goal_y, theta - np.pi])

        else:
            raise ValueError(
                f"Unknown distribution name: '{distribution['name']}'. "
                "Supported distributions are: 'manual', 'random', 'circle'."
            )

        return state_list, goal_list
