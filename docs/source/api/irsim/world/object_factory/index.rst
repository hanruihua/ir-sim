irsim.world.object_factory
==========================

.. py:module:: irsim.world.object_factory


Classes
-------

.. autoapisummary::

   irsim.world.object_factory.ObjectFactory


Module Contents
---------------

.. py:class:: ObjectFactory

   Factory class for creating various objects in the simulation.


   .. py:method:: create_from_parse(parse: Union[list[dict[str, Any]], dict[str, Any]], obj_type: str = 'robot') -> list[Any]

      Create objects from a parsed configuration.

      :param parse: Parsed configuration data.
      :type parse: list or dict
      :param obj_type: Type of object to create, 'robot' or 'obstacle'.
      :type obj_type: str

      :returns: List of created objects.
      :rtype: list



   .. py:method:: create_from_map(points: numpy.ndarray, reso: float = 0.1) -> list[Any]

      Create map objects from points.

      :param points: List of points.
      :type points: list
      :param reso: Resolution of the map.
      :type reso: float

      :returns: List of ObstacleMap objects.
      :rtype: list



   .. py:method:: create_object(obj_type: str = 'robot', number: int = 1, distribution: Optional[dict[str, Any]] = None, state: Optional[list[float]] = None, goal: Optional[list[float]] = None, **kwargs: Any) -> list[Any]

      Create multiple objects based on the parameters.

      :param obj_type: Type of object, 'robot' or 'obstacle'.
      :type obj_type: str
      :param number: Number of objects to create.
      :type number: int
      :param distribution: Distribution type for generating states.
      :type distribution: dict
      :param state: Initial state for objects.
      :type state: list
      :param goal: Goal state for objects.
      :type goal: list
      :param \*\*kwargs: Additional parameters for object creation.

      :returns: List of created objects.
      :rtype: list



   .. py:method:: create_robot(kinematics: Optional[dict[str, Any]] = None, **kwargs: Any) -> Any

      Create a robot based on kinematics.

      :param kinematics: Kinematics configuration.
      :type kinematics: dict
      :param \*\*kwargs: Additional parameters for robot creation.

      :returns: An instance of a robot.
      :rtype: Robot



   .. py:method:: create_obstacle(kinematics: Optional[dict[str, Any]] = None, **kwargs: Any) -> Any

      Create a obstacle based on kinematics.

      :param kinematics: Kinematics configuration.
      :type kinematics: dict
      :param \*\*kwargs: Additional parameters for robot creation.

      :returns: An instance of an obstacle.
      :rtype: Obstacle



   .. py:method:: generate_state_list(number: int = 1, distribution: Optional[dict[str, Any]] = None, state: Optional[list[float]] = None, goal: Optional[list[float]] = None) -> tuple

      Generate a list of state vectors for multiple objects based on the specified distribution method.

      This function creates initial states for multiple objects in the simulation environment.
      It supports various distribution methods such as 'manual', 'circle', and 'random' to
      position the objects according to specific patterns or randomness.

      :param number: Number of state vectors to generate. Default is 1.
      :type number: int
      :param distribution: Configuration dictionary specifying the distribution method and its parameters.
                           Default is {"name": "manual"}.
      :type distribution: Dict[str, Any]
      :param state: Base state vector [x, y, theta] to use as a template for generating states.
                    Default is [1, 1, 0].
      :type state: List[float]
      :param goal: Goal state vector [x, y, theta] for the generated objects.
                   Default is [1, 9, 0].

                   - 'name' (str): Name of the distribution method. Supported values are:

                     - 'manual': States are specified manually.
                     - 'circle': States are arranged in a circular pattern.
                     - 'random': States are placed at random positions.

                   - Additional parameters depend on the distribution method:

                     - For 'manual': Manually specified states and goal.
                     - For 'circle':

                       - 'center' (List[float]): Center coordinates [x, y] of the circle.
                       - 'radius' (float): Radius of the circle.

                     - For 'random':

                       - 'range_low' (List[float]): Lower bounds for random state values.
                       - 'range_high' (List[float]): Upper bounds for random state values.
      :type goal: List[float]

      :returns:     A list containing generated state vectors and goal vectors for objects.
      :rtype: List[List[float]]

      :raises ValueError: If the distribution method specified in 'name' is not supported or if required
          parameters for a distribution method are missing.



