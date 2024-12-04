import numpy as np
from irsim.world.robots.robot_diff import RobotDiff
from irsim.world.robots.robot_acker import RobotAcker
from irsim.world.robots.robot_omni import RobotOmni
from irsim.world import ObjectBase
from irsim.world.obstacles.obstacle_diff import ObstacleDiff
from irsim.world.obstacles.obstacle_omni import ObstacleOmni
from irsim.world.obstacles.obstacle_static import ObstacleStatic
from irsim.world.obstacles.obstacle_acker import ObstacleAcker

from irsim.world.map.obstacle_map import ObstacleMap
from irsim.util.util import (
    convert_list_length,
    convert_list_length_dict,
    is_list_of_numbers,
)
from irsim.global_param import env_param
import random

class ObjectFactory:
    """
    Factory class for creating various objects in the simulation.
    """

    def create_from_parse(self, parse, obj_type="robot"):
        """
        Create objects from a parsed configuration.

        Args:
            parse (list or dict): Parsed configuration data.
            obj_type (str): Type of object to create, 'robot' or 'obstacle'.

        Returns:
            list: List of created objects.
        """
        object_list = list()

        if isinstance(parse, list):
            object_list = [
                obj for sp in parse for obj in self.create_object(obj_type, **sp)
            ]

        elif isinstance(parse, dict):
            object_list = [obj for obj in self.create_object(obj_type, **parse)]

        return object_list

    def create_from_map(self, points, reso=0.1):
        """
        Create map objects from points.

        Args:
            points (list): List of points.
            reso (float): Resolution of the map.

        Returns:
            list: List of ObstacleMap objects.
        """
        if points is None:
            return []
        else:
            return [
                ObstacleMap(shape={'name': "points", 'points': points, 'reso': reso}, color="k")
            ]

    def create_object(
        self,
        obj_type="robot",
        number=1,
        distribution={"name": "manual"},
        state=[1, 1, 0],
        goal=[1, 9, 0],
        **kwargs,
    ):
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
        state_list, goal_list = self.generate_state_list(
            number, distribution, state, goal
        )
        object_list = list()

        for i in range(number):
            obj_dict = {
                k: convert_list_length(v, number)[i]
                for k, v in kwargs.items()
                if k != "sensors"
            }
            obj_dict["state"] = state_list[i]
            obj_dict["goal"] = goal_list[i]
            obj_dict["sensors"] = convert_list_length_dict(
                kwargs.get("sensors", None), number
            )[i]

            if obj_type == "robot":
                object_list.append(self.create_robot(**obj_dict))
            elif obj_type == "obstacle":
                object_list.append(self.create_obstacle(**obj_dict))

        return object_list


    def create_robot(self, kinematics=dict(), **kwargs):
        """
        Create a robot based on kinematics.

        Args:
            kinematics (dict): Kinematics configuration.
            **kwargs: Additional parameters for robot creation.

        Returns:
            Robot: An instance of a robot.
        """
        kinematics_name = kinematics.get("name", "omni")

        if kinematics_name == "diff":
            return RobotDiff(
                kinematics=kinematics, **kwargs
            )
        elif kinematics_name == "acker":
            return RobotAcker(
                kinematics=kinematics, **kwargs
            )
        elif kinematics_name == "omni":
            return RobotOmni(
                kinematics=kinematics, **kwargs
            )
        else:
            raise NotImplementedError(
                f"Robot kinematics {kinematics_name} not implemented"
            )


    def create_obstacle(self, kinematics=dict(), **kwargs):
        """
        Create a obstacle based on kinematics.

        Args:
            kinematics (dict): Kinematics configuration.
            **kwargs: Additional parameters for robot creation.

        Returns:
            Obstacle: An instance of an obstacle.
        """
        kinematics_name = kinematics.get("name", "omni")

        if kinematics_name == "diff":
            return ObstacleDiff(
                kinematics=kinematics, **kwargs
            )
        elif kinematics_name == "acker":
            return ObstacleAcker(
                kinematics=kinematics, **kwargs
            )
        elif kinematics_name == "omni":
            return ObstacleOmni(
                kinematics=kinematics, **kwargs
            )
        else:
            raise NotImplementedError(
                f"Robot kinematics {kinematics_name} not implemented"
            )
   
    def generate_state_list(
        self,
        number=1,
        distribution={"name": "manual"},
        state=[1, 1, 0],
        goal=[1, 9, 0],
    ):
        """
        Generate lists of states and goals for objects.

        Args:
            number (int): Number of objects.
            distribution (dict): Distribution type for generating states.
            state (list): Initial state for objects.
            goal (list): Goal state for objects.

        Returns:
            tuple: Lists of states and goals.
        """
        if distribution["name"] == "manual":
            state_list = convert_list_length(state, number)
            goal_list = convert_list_length(goal, number)

        elif distribution["name"] == "random":
            range_low = distribution.get("range_low", [0, 0, -np.pi])
            range_high = distribution.get("range_high", [10, 10, np.pi])

            state_array = np.random.uniform(
                low=range_low, high=range_high, size=(number, 3)
            )
            state_list = state_array.tolist()

            goal_array = np.random.uniform(
                low=range_low, high=range_high, size=(number, 3)
            )
            goal_list = goal_array.tolist()

        elif distribution["name"] == "uniform":
            pass

        elif distribution["name"] == "circle":
            radius = distribution.get("radius", 4)
            center = distribution.get("center", [5, 5, 0])

            state_list, goal_list = [], []
            for i in range(number):
                theta = 2 * np.pi * i / number
                x = center[0] + radius * np.cos(theta)
                y = center[1] + radius * np.sin(theta)
                state_list.append([x, y, theta - np.pi])

                goal_x = center[0] - radius * np.cos(theta)
                goal_y = center[1] - radius * np.sin(theta)
                goal_list.append([goal_x, goal_y, 0])

        return state_list, goal_list