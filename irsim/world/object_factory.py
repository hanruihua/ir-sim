import numpy as np

from irsim.util.util import (
    convert_list_length,
    convert_list_length_dict,
)

from irsim.world import (
    RobotAcker,
    RobotDiff,
    RobotOmni,
    ObstacleAcker,
    ObstacleDiff,
    ObstacleOmni,
    ObjectStatic,
    ObstacleMap
)


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
                obj for group_index, sp in enumerate(parse) for obj in self.create_object(obj_type, group=group_index, **sp)
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
                ObstacleMap(
                    shape={"name": "map", "points": points, "reso": reso}, color="k"
                )
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

        if not distribution.get('3d', False):
            state_list, goal_list = self.generate_state_list(
                number, distribution, state, goal
            )
        else:
            state_list, goal_list = self.generate_state_list3D(
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
        kinematics_name = kinematics.get("name", None)

        if kinematics_name == "diff":
            return RobotDiff(kinematics=kinematics, **kwargs)
        elif kinematics_name == "acker":
            return RobotAcker(kinematics=kinematics, **kwargs)
        elif kinematics_name == "omni":
            return RobotOmni(kinematics=kinematics, **kwargs)
        elif kinematics_name == "static" or kinematics_name is None:
            return ObjectStatic(kinematics=kinematics, role="robot", **kwargs)
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
        kinematics_name = kinematics.get("name", None)

        if kinematics_name == "diff":
            return ObstacleDiff(kinematics=kinematics, **kwargs)
        elif kinematics_name == "acker":
            return ObstacleAcker(kinematics=kinematics, **kwargs)
        elif kinematics_name == "omni":
            return ObstacleOmni(kinematics=kinematics, **kwargs)
        elif kinematics_name == "static" or kinematics_name is None:
            return ObjectStatic(kinematics=kinematics, role="obstacle", **kwargs)
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
        Generate a list of state vectors for multiple objects based on the specified distribution method.

        This function creates initial states for multiple objects in the simulation environment.
        It supports various distribution methods such as 'manual', 'circle', and 'random' to
        position the objects according to specific patterns or randomness.

        Args:
            state (Optional[List[float]]): 
                Base state vector [x, y, theta] to use as a template for generating states.
                If None, default values will be used.
            number (int): 
                Number of state vectors to generate.
            distribution (Dict[str, Any]): 
                Configuration dictionary specifying the distribution method and its parameters.
                - 'name' (str): 
                    Name of the distribution method. Supported values are:
                    - 'manual': States are specified manually.
                    - 'circle': States are arranged in a circular pattern.
                    - 'random': States are placed at random positions.
                - Additional parameters depend on the distribution method:
                    - For 'manual':
                        Manually specified states and goal.
                    - For 'circle':
                        - 'center' (List[float]): Center coordinates [x, y] of the circle.
                        - 'radius' (float): Radius of the circle.
                    - For 'random':
                        - 'range_low' (List[float]): Lower bounds for random state values. 
                        - 'range_high' (List[float]): Upper bounds for random state values.

        Returns:
            List[List[float]]: 
                A list containing generated state vectors and goal vectors for objects.

        Raises:
            ValueError: 
                If the distribution method specified in 'name' is not supported or if required
                parameters for a distribution method are missing.
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


    def generate_state_list3D(self, number=1, distribution={"name": "manual"}, state=[1, 1, 0], goal=[1, 9, 0]):
        pass
        return [], []