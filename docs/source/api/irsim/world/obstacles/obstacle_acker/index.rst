irsim.world.obstacles.obstacle_acker
====================================

.. py:module:: irsim.world.obstacles.obstacle_acker


Classes
-------

.. autoapisummary::

   irsim.world.obstacles.obstacle_acker.ObstacleAcker


Module Contents
---------------

.. py:class:: ObstacleAcker(color='k', state_dim=4, **kwargs)

   Bases: :py:obj:`irsim.world.object_base.ObjectBase`


   Base class representing a generic object in the robot simulator.

   This class encapsulates common attributes and behaviors for all objects,
   including robots and obstacles, managing their state, velocity, goals,
   and kinematics.

   :param shape: Parameters defining the shape of the object for geometry creation.
                 The dictionary should contain keys and values required by the GeometryFactory to create
                 the object's geometry, such as 'type' (e.g., 'circle', 'rectangle') and associated parameters.
                 Defaults to None.
   :type shape: dict
   :param kinematics: Parameters defining the kinematics of the object.
                      Includes kinematic model and any necessary parameters. If None, no kinematics model is applied.
                      Defaults to None.
   :type kinematics: dict
   :param state: Initial state vector [x, y, theta, ...].
                 The state can have more dimensions depending on `state_dim`. Excess dimensions are truncated,
                 and missing dimensions are filled with zeros. Defaults to [0, 0, 0].
   :type state: list of float
   :param velocity: Initial velocity vector [vx, vy] or according to the kinematics model.
                    Defaults to [0, 0].
   :type velocity: list of float
   :param goal: Goal state vector [x, y, theta, ...] or [[x, y, theta], [x, y, theta], ...] for multiple goals
                Used by behaviors to determine the desired movement. Defaults to [10, 10, 0].
   :type goal: list of float or list of list of float
   :param role: Role of the object in the simulation, e.g., "robot" or "obstacle".
                Defaults to "obstacle".
   :type role: str
   :param color: Color of the object when plotted.
                 Defaults to "k" (black).
   :type color: str
   :param static: Indicates if the object is static (does not move).
                  Defaults to False.
   :type static: bool
   :param vel_min: Minimum velocity limits for each control dimension.
                   Used to constrain the object's velocity. Defaults to [-1, -1].
   :type vel_min: list of float
   :param vel_max: Maximum velocity limits for each control dimension.
                   Used to constrain the object's velocity. Defaults to [1, 1].
   :type vel_max: list of float
   :param acce: Acceleration limits, specifying the maximum change in velocity per time step.
                Defaults to [inf, inf].
   :type acce: list of float
   :param angle_range: Allowed range of orientation angles [min, max] in radians.
                       The object's orientation will be wrapped within this range. Defaults to [-pi, pi].
   :type angle_range: list of float
   :param behavior: Behavioral mode or configuration of the object.
                    Can be a behavior name (str) or a dictionary with behavior parameters. If None, default behavior is applied.
                    Defaults to {'name': 'dash'}, moving to the goal directly.
   :type behavior: dict or str
   :param goal_threshold: Threshold distance to determine if the object has reached its goal.
                          When the object is within this distance to the goal, it's considered to have arrived. Defaults to 0.1.
   :type goal_threshold: float
   :param sensors: List of sensor configurations attached to the object.
                   Each sensor configuration is a dictionary specifying sensor type and parameters. Defaults to None.
   :type sensors: list of dict
   :param arrive_mode: Mode for arrival detection, either "position" or "state".
                       Determines how arrival at the goal is evaluated. Defaults to "position".
   :type arrive_mode: str
   :param description: Description or label for the object.
                       Can be used for identification or attaching images in plotting. Defaults to None.
   :type description: str
   :param group: Group identifier for organizational purposes, allowing objects to be grouped.
                 Defaults to 0.
   :type group: int
   :param state_dim: Dimension of the state vector.
                     If None, it is inferred from the class attribute `state_shape`. Defaults to None.
   :type state_dim: int
   :param vel_dim: Dimension of the velocity vector.
                   If None, it is inferred from the class attribute `vel_shape`. Defaults to None.
   :type vel_dim: int
   :param unobstructed: Indicates if the object should be considered to have an unobstructed path,
                        ignoring obstacles in certain scenarios. Defaults to False.
   :type unobstructed: bool
   :param fov: Field of view angles in radians for the object's sensors. Defaults to None. If set lidar, the default value is angle range of lidar.
   :type fov: float
   :param fov_radius: Field of view radius for the object's sensors. Defaults to None. If set lidar, the default value is range_max of lidar.
   :type fov_radius: float
   :param \*\*kwargs: Additional keyword arguments for extended functionality.

                      - plot (dict): Plotting options for the object.
                        May include 'show_goal', 'show_text', 'show_arrow', 'show_uncertainty', 'show_trajectory',
                        'trail_freq', etc.

   :raises ValueError: If dimension parameters do not match the provided shapes or if input parameters are invalid.

   .. attribute:: state_dim

      Dimension of the state vector.

      :type: int

   .. attribute:: state_shape

      Shape of the state array.

      :type: tuple

   .. attribute:: vel_dim

      Dimension of the velocity vector.

      :type: int

   .. attribute:: vel_shape

      Shape of the velocity array.

      :type: tuple

   .. attribute:: state

      Current state of the object.

      :type: np.ndarray

   .. attribute:: _init_state

      Initial state of the object.

      :type: np.ndarray

   .. attribute:: _velocity

      Current velocity of the object.

      :type: np.ndarray

   .. attribute:: _init_velocity

      Initial velocity of the object.

      :type: np.ndarray

   .. attribute:: _goal

      Goal state of the object.

      :type: np.ndarray

   .. attribute:: _init_goal

      Initial goal state of the object.

      :type: np.ndarray

   .. attribute:: _geometry

      Geometry representation of the object.

      :type: any

   .. attribute:: group

      Group identifier for the object.

      :type: int

   .. attribute:: stop_flag

      Flag indicating if the object should stop.

      :type: bool

   .. attribute:: arrive_flag

      Flag indicating if the object has arrived at the goal.

      :type: bool

   .. attribute:: collision_flag

      Flag indicating a collision has occurred.

      :type: bool

   .. attribute:: unobstructed

      Indicates if the object has an unobstructed path.

      :type: bool

   .. attribute:: static

      Indicates if the object is static.

      :type: bool

   .. attribute:: vel_min

      Minimum velocity limits.

      :type: np.ndarray

   .. attribute:: vel_max

      Maximum velocity limits.

      :type: np.ndarray

   .. attribute:: color

      Color of the object.

      :type: str

   .. attribute:: role

      Role of the object (e.g., "robot", "obstacle").

      :type: str

   .. attribute:: info

      Information container for the object.

      :type: ObjectInfo

   .. attribute:: wheelbase

      Distance between the front and rear wheels. Specified for ackermann robots.

      :type: float

   .. attribute:: fov

      Field of view angles in radians.

      :type: float

   .. attribute:: fov_radius

      Field of view radius.

      :type: float

   Initialize an ObjectBase instance.

   This method sets up a new ObjectBase object with the specified parameters, initializing its
   geometry, kinematics, behaviors, sensors, and other properties relevant to simulation.

   The initialization process includes:
   - Setting up geometry handlers and collision detection
   - Configuring kinematics models for movement
   - Initializing state vectors and goal management
   - Setting up behaviors and sensor systems
   - Configuring visualization and plotting options

   .. note::

      All parameters are documented in the class docstring above. Refer to the
      :py:class:`ObjectBase` class documentation for detailed parameter descriptions.

   :raises ValueError: If dimension parameters do not match the provided shapes or
       if input parameters are invalid.


