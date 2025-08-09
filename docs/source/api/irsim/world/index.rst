irsim.world
===========

.. py:module:: irsim.world


Submodules
----------

.. toctree::
   :maxdepth: 1

   /api/irsim/world/map/index
   /api/irsim/world/object_base/index
   /api/irsim/world/object_factory/index
   /api/irsim/world/obstacles/index
   /api/irsim/world/robots/index
   /api/irsim/world/sensors/index
   /api/irsim/world/world/index
   /api/irsim/world/world3d/index


Classes
-------

.. autoapisummary::

   irsim.world.ObstacleMap
   irsim.world.ObjectBase
   irsim.world.ObjectFactory
   irsim.world.ObstacleAcker
   irsim.world.ObstacleDiff
   irsim.world.ObstacleOmni
   irsim.world.ObjectStatic
   irsim.world.RobotAcker
   irsim.world.RobotDiff
   irsim.world.RobotOmni
   irsim.world.SensorFactory
   irsim.world.World


Package Contents
----------------

.. py:class:: ObstacleMap(shape=None, color='k', static=True, **kwargs)

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


   .. py:attribute:: linestrings


   .. py:attribute:: geometry_tree


.. py:class:: ObjectBase(shape: Optional[dict] = None, kinematics: Optional[dict] = None, state: Optional[list] = None, velocity: Optional[list] = None, goal: Optional[list] = None, role: str = 'obstacle', color: str = 'k', static: bool = False, vel_min: Optional[list] = None, vel_max: Optional[list] = None, acce: Optional[list] = None, angle_range: Optional[list] = None, behavior: Optional[dict] = None, goal_threshold: float = 0.1, sensors: Optional[dict] = None, arrive_mode: str = 'position', description: Optional[str] = None, group: int = 0, state_dim: Optional[int] = None, vel_dim: Optional[int] = None, unobstructed: bool = False, fov: Optional[float] = None, fov_radius: Optional[float] = None, **kwargs)

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


   .. py:attribute:: id_iter


   .. py:attribute:: vel_shape
      :value: (2, 1)



   .. py:attribute:: state_shape
      :value: (3, 1)



   .. py:attribute:: gf


   .. py:attribute:: kf


   .. py:attribute:: state_dim
      :value: None



   .. py:attribute:: vel_dim
      :value: None



   .. py:attribute:: role
      :value: 'obstacle'



   .. py:attribute:: group
      :value: 0



   .. py:attribute:: stop_flag
      :value: False



   .. py:attribute:: arrive_flag
      :value: False



   .. py:attribute:: collision_flag
      :value: False



   .. py:attribute:: unobstructed
      :value: False



   .. py:attribute:: static
      :value: False



   .. py:attribute:: vel_min


   .. py:attribute:: vel_max


   .. py:attribute:: color
      :value: 'k'



   .. py:attribute:: info


   .. py:attribute:: obstacle_info
      :value: None



   .. py:attribute:: trajectory
      :value: []



   .. py:attribute:: description
      :value: None



   .. py:attribute:: goal_threshold
      :value: 0.1



   .. py:attribute:: arrive_mode
      :value: 'position'



   .. py:attribute:: lidar
      :value: None



   .. py:attribute:: obj_behavior


   .. py:attribute:: rl


   .. py:attribute:: rh


   .. py:attribute:: wander


   .. py:attribute:: plot_kwargs


   .. py:attribute:: plot_patch_list
      :value: []



   .. py:attribute:: plot_line_list
      :value: []



   .. py:attribute:: plot_text_list
      :value: []



   .. py:attribute:: collision_obj
      :value: []



   .. py:attribute:: plot_trail_list
      :value: []



   .. py:method:: reset_id_iter(start: int = 0, step: int = 1)
      :classmethod:


      reset the id iterator



   .. py:method:: step(velocity: Optional[numpy.ndarray] = None, **kwargs: Any)

      Perform a single simulation step, updating the object's state and sensors.

      This method advances the object by one time step, integrating the given velocity
      or behavior-generated velocity to update the object's position, orientation, and
      other state variables. It also updates sensors and checks for collisions.

      :param velocity: Desired velocity for this step.
                       If None, the object will use its behavior system to generate velocity.
                       The shape and meaning depend on the kinematics model:

                       - Differential: [linear_velocity, angular_velocity]
                       - Omnidirectional: [velocity_x, velocity_y]
                       - Ackermann: [linear_velocity, steering_angle]
      :type velocity: np.ndarray, optional
      :param \*\*kwargs: Additional parameters passed to behavior generation and processing.

      :returns: The updated state vector of the object after the step.
                Returns the current state unchanged if the object is static or stopped.
      :rtype: np.ndarray

      .. note::

         - Static objects (static=True) will not move and return their current state
         - Objects with stop_flag=True will halt and return their current state
         - The method automatically handles sensor updates and trajectory recording



   .. py:method:: sensor_step()

      Update all sensors for the current state.



   .. py:method:: check_status()

      Check the current status of the object, including arrival and collision detection.

      This method evaluates collision detection and sets stop flags based on the collision mode.
      It also handles different collision modes like 'stop',  , 'unobstructed', etc.



   .. py:method:: check_arrive_status()

      Check if the object has arrived at its goal position.

      The arrival detection depends on the arrive_mode setting:
      - "state": Compares full state (x, y, theta)
      - "position": Compares only position (x, y)

      Updates the arrive_flag and handles multiple goals by removing completed ones.



   .. py:method:: check_collision_status()

      Check if the object is in collision with other objects in the environment.

      This method queries possible collision objects from the geometry tree and
      checks for intersections. It logs collision warnings for robots and updates
      the collision_flag and collision_obj list.



   .. py:method:: check_collision(obj)

      Check collision with another object.

      :param obj: Another object to check collision with.
      :type obj: ObjectBase

      :returns: True if collision occurs, False otherwise.
      :rtype: bool



   .. py:method:: gen_behavior_vel(velocity: Optional[numpy.ndarray] = None) -> numpy.ndarray

      Generate behavior-influenced velocity for the object.

      This method adjusts the desired velocity based on the object's behavior configurations.
      If no desired velocity is provided (`velocity` is None), the method may generate a default
      velocity or issue warnings based on the object's role and behavior settings.

      :param velocity: Desired velocity vector. If None, the method determines
                       the velocity based on behavior configurations. Defaults to None.
      :type velocity: Optional[np.ndarray]

      :returns: Velocity vector adjusted based on behavior configurations and constraints.
      :rtype: np.ndarray

      :raises Warning: If `velocity` is None and no behavior configuration is set for a robot.



   .. py:method:: pre_process()

      Perform pre-processing before stepping the object.

      This method is called before velocity generation and state updates.
      Can be overridden by subclasses to implement custom pre-processing logic.



   .. py:method:: post_process()

      Perform post-processing after stepping the object.

      This method is called after state updates and sensor updates.
      Can be overridden by subclasses to implement custom post-processing logic.



   .. py:method:: mid_process(state: numpy.ndarray)

      Process state in the middle of a step. Make sure the state is within the desired dimension.

      :param state: State vector.
      :type state: np.ndarray

      :returns: Processed state.
      :rtype: np.ndarray



   .. py:method:: get_lidar_scan()

      Get the lidar scan of the object.

      :returns: Lidar scan data containing range and angle information.
      :rtype: dict



   .. py:method:: get_lidar_points()

      Get the lidar scan points of the object.

      :returns: Array of lidar scan points.
      :rtype: np.ndarray



   .. py:method:: get_lidar_offset()

      Get the lidar offset relative to the object.

      :returns: Lidar offset [x, y, theta] relative to the object center.
      :rtype: list



   .. py:method:: get_fov_detected_objects()

      Detect the env objects that in the field of view.


      :returns: The objects that in the field of view of the object.
      :rtype: list



   .. py:method:: fov_detect_object(detected_object: ObjectBase)

      Detect whether the input object is in the field of view.

      :param object: The object that to be detected.

      :returns: Whether the object is in the field of view.
      :rtype: bool



   .. py:method:: set_state(state: Optional[Union[list, numpy.ndarray]] = None, init: bool = False)

      Set the current state of the object.

      This method updates the object's position, orientation, and other state variables.
      It also updates the object's geometry representation to match the new state.

      :param state: The new state vector for the object.
                    The format depends on the object's state dimension:

                    - 2D objects: [x, y, theta] where theta is orientation in radians
                    - 3D objects: [x, y, z, roll, pitch, yaw] or similar based on configuration

                    Must match the object's state_dim dimension.
      :type state: Union[list, np.ndarray]
      :param init: Whether to also set this as the initial state for reset purposes.
                   If True, the object will return to this state when reset() is called.
                   Default is False.
      :type init: bool

      :raises AssertionError: If the state dimension doesn't match the expected state_dim.

      .. rubric:: Example

      >>> # Set robot position and orientation
      >>> robot.set_state([5.0, 3.0, 1.57])  # x=5, y=3, facing pi/2 radians
      >>>
      >>> # Set as initial state for resets
      >>> robot.set_state([0, 0, 0], init=True)



   .. py:method:: set_velocity(velocity: Optional[Union[list, numpy.ndarray]] = None, init: bool = False) -> None

      Set the velocity of the object.

      :param velocity: The velocity of the object. Depending on the kinematics model.
      :param init: Whether to set the initial velocity (default False).
      :type init: bool



   .. py:method:: set_original_geometry(geometry: shapely.geometry.base.BaseGeometry)

      Set the original geometry of the object.



   .. py:method:: set_random_goal(obstacle_list, init: bool = False, free: bool = True, goal_check_radius: float = 0.2, range_limits: Optional[list] = None, max_attempts: int = 100)

      Set random goal(s) in the environment. If free set to True, the goal will be placed only in the free from
      obstacles part of the environment.

      :param obstacle_list: List of objects in the environment
      :param init: Whether to set the initial goal (default False).
      :type init: bool
      :param free: Whether to check that goal is placed in a position free of obstacles.
      :type free: bool
      :param goal_check_radius: Radius in which to check if the goal is free of obstacles.
      :type goal_check_radius: float
      :param range_limits: List of lower and upper bound range limits in which to set the random goal position.
      :type range_limits: list
      :param max_attempts: Max number of attempts to place the goal in a position free of obstacles.
      :type max_attempts: int



   .. py:method:: set_goal(goal: Optional[Union[list, numpy.ndarray]] = None, init: bool = False)

      Set the goal(s) for the object to navigate towards.

      This method configures the target location(s) that the object's behavior system
      will attempt to reach. Multiple goals can be provided for sequential navigation.

      :param goal: The goal specification. Can be:

                   - Single goal: [x, y, theta] for one target location
                   - Multiple goals: [[x1, y1, theta1], [x2, y2, theta2], ...] for sequential targets
                   - None: Clear all goals

                   The theta component specifies the desired final orientation in radians.
      :type goal: Union[list, np.ndarray]
      :param init: Whether to also set this as the initial goal for reset purposes.
                   If True, these goals will be restored when reset() is called.
                   Default is False.
      :type init: bool

      .. rubric:: Example

      >>> # Set single goal
      >>> robot.set_goal([10.0, 5.0, 0.0])  # Move to (10,5) facing East
      >>>
      >>> # Set multiple sequential goals
      >>> waypoints = [[5, 0, 0], [10, 5, 1.57], [0, 10, 3.14]]
      >>> robot.set_goal(waypoints)
      >>>
      >>> # Clear goals
      >>> robot.set_goal(None)



   .. py:method:: append_goal(goal: Union[list, numpy.ndarray])

      Append a goal to the goal list.



   .. py:method:: set_laser_color(laser_indices, laser_color: str = 'cyan', alpha: float = 0.3)

      Set the color of the lasers.

      :param laser_indices: The indices of the lasers to set the color.
      :type laser_indices: list
      :param laser_color: The color to set the lasers. Default is 'cyan'.
      :type laser_color: str
      :param alpha: The transparency of the lasers. Default is 0.3.
      :type alpha: float



   .. py:method:: input_state_check(state: list, dim: int = 3)

      Check and adjust the state to match the desired dimension.

      :param state: State of the object.
      :type state: list
      :param dim: Desired dimension. Defaults to 3.
      :type dim: int

      :returns: Adjusted state.
      :rtype: list



   .. py:method:: plot(ax, state: Optional[numpy.ndarray] = None, vertices: Optional[numpy.ndarray] = None, **kwargs)

      Plot the object on the given axis.

      :param ax: Matplotlib axis object for plotting.
      :param state: State vector [x, y, theta, ...] defining object position and orientation.
      :param vertices: Vertices array defining object shape for polygon/rectangle objects.
      :param \*\*kwargs: Plotting configuration options.



   .. py:method:: set_element_property(element, state, **kwargs)


   .. py:method:: plot_object(ax, state: Optional[numpy.ndarray] = None, vertices: Optional[numpy.ndarray] = None, **kwargs)

      Plot the object itself in the specified coordinate system.

      :param ax: Matplotlib axis object
      :param state: State of the object (x, y, r_phi) defining position and orientation.
                    If None, uses the object's current state. Defaults to None.
      :param vertices: Vertices of the object [[x1, y1], [x2, y2], ...] for polygon and rectangle shapes.
                       If None, uses the object's current vertices. Defaults to None.
      :param \*\*kwargs: Additional plotting options
                         - obj_linestyle (str): Line style for object outline, defaults to '-'
                         - obj_zorder (int): Drawing layer order, defaults to 3 if object is robot, 1 if object is the obstacle.
                         - obj_color (str): Color of the object, defaults to 'k' (black).
                         - obj_alpha (float): Transparency of the object, defaults to 1.0.

      :returns: None

      :raises ValueError: When object shape is not supported



   .. py:method:: plot_object_image(ax, state: Optional[numpy.ndarray] = None, vertices: Optional[numpy.ndarray] = None, description: Optional[str] = None, **kwargs)

      Plot the object using an image file based on the description.

      :param ax: Matplotlib axis object for plotting.
      :param state: State of the object (x, y, r_phi) defining position and orientation.
                    If None, uses the object's current state. Defaults to None.
      :type state: Optional[np.ndarray]
      :param vertices: Vertices of the object for positioning the image.
                       If None, uses the object's current vertices. Defaults to None.
      :type vertices: Optional[np.ndarray]
      :param description: Path or name of the image file to display. Defaults to None.
      :type description: str
      :param \*\*kwargs: Additional plotting options (currently unused).

      .. note::

         The image file is searched in the world/description/ directory relative to the project root.
         The image is rotated and positioned according to the object's state and vertices.



   .. py:method:: plot_trajectory(ax, trajectory: Optional[list] = None, keep_traj_length: int = 0, **kwargs)

      Plot the trajectory path of the object using the specified trajectory data.

      :param ax: Matplotlib axis.
      :param trajectory: List of trajectory points to plot, where each point is a numpy array [x, y, theta, ...].
                         If None, uses self.trajectory. Defaults to None.
      :param keep_traj_length: Number of steps to keep from the end of trajectory.
                               If 0, plots entire trajectory. Defaults to 0.
      :type keep_traj_length: int
      :param \*\*kwargs: Additional plotting options:
                         traj_color (str): Color of the trajectory line.
                         traj_style (str): Line style of the trajectory.
                         traj_width (float): Width of the trajectory line.
                         traj_alpha (float): Transparency of the trajectory line.
                         traj_zorder (int): Zorder of the trajectory.



   .. py:method:: plot_goal(ax, goal_state: Optional[numpy.ndarray] = None, vertices: Optional[numpy.ndarray] = None, goal_color: Optional[str] = None, goal_zorder: Optional[int] = 1, goal_alpha: Optional[float] = 0.5, **kwargs)

      Plot the goal position of the object in the specified coordinate system.

      :param ax: Matplotlib axis.
      :param goal_state: State of the goal (x, y, r_phi) defining goal position and orientation.
                         If None, uses [0, 0, 0]. Defaults to None.
      :param vertices: Vertices for polygon/rectangle goal shapes.
                       If None, uses original_vertices. Defaults to None.
      :param goal_color: Color of the goal marker. Defaults to be the color of the object.
      :type goal_color: str
      :param goal_zorder: Zorder of the goal marker. Defaults to 1.
      :type goal_zorder: int
      :param goal_alpha: Transparency of the goal marker. Defaults to 0.5.
      :type goal_alpha: float



   .. py:method:: plot_text(ax, state: Optional[numpy.ndarray] = None, **kwargs)

      Plot the text label of the object at the specified position.

      :param ax: Matplotlib axis.
      :param state: State of the object (x, y, r_phi) to determine text position.
                    If None, uses the object's current state. Defaults to None.
      :param \*\*kwargs: Additional plotting options.

                         - text_color (str): Color of the text, default is 'k'.
                         - text_size (int): Font size of the text, default is 10.
                         - text_position (list): Position offset from object center [dx, dy],
                           default is [-radius-0.1, radius+0.1].
                         - text_zorder (int): Zorder of the text. Defaults to 2.
                         - text_alpha (float): Transparency of the text. Defaults to 1.



   .. py:method:: plot_arrow(ax, state: Optional[numpy.ndarray] = None, velocity: Optional[numpy.ndarray] = None, arrow_length: float = 0.4, arrow_width: float = 0.6, arrow_color: Optional[str] = None, arrow_zorder: int = 3, **kwargs)

      Plot an arrow indicating the velocity orientation of the object at the specified position.

      :param ax: Matplotlib axis.
      :param state: State of the object (x, y, r_phi) to determine arrow position.
                    If None, uses the object's current state. Defaults to None.
      :param velocity: Velocity of the object to determine arrow direction.
                       If None, uses the object's current velocity_xy. Defaults to None.
      :param arrow_length: Length of the arrow. Defaults to 0.4.
      :type arrow_length: float
      :param arrow_width: Width of the arrow. Defaults to 0.6.
      :type arrow_width: float
      :param arrow_color: Color of the arrow. Defaults to "gold".
      :type arrow_color: str
      :param arrow_zorder: Z-order for drawing layer. Defaults to 4.
      :type arrow_zorder: int



   .. py:method:: plot_trail(ax, state: Optional[numpy.ndarray] = None, vertices: Optional[numpy.ndarray] = None, keep_trail_length: int = 0, **kwargs)

      Plot the trail/outline of the object at the specified position for visualization purposes.

      :param ax: Matplotlib axis.
      :param state: State of the object (x, y, r_phi) to determine trail position and orientation.
                    If None, uses the object's current state. Defaults to None.
      :param vertices: Vertices of the object for polygon and rectangle trail shapes.
                       If None, uses the object's current vertices. Defaults to None.
      :param keep_trail_length: Number of steps to keep from the recent trajectory of trail.
      :type keep_trail_length: int
      :param \*\*kwargs: Additional plotting options:
                         trail_type (str): Type of trail shape, defaults to object's shape.
                         trail_edgecolor (str): Edge color of the trail.
                         trail_linewidth (float): Line width of the trail edge.
                         trail_alpha (float): Transparency of the trail.
                         trail_fill (bool): Whether to fill the trail shape.
                         trail_color (str): Fill color of the trail.
                         trail_zorder (int): Z-order of the trail.



   .. py:method:: plot_fov(ax, **kwargs)

      Plot the field of view of the object.
      Creates FOV wedge at origin, will be positioned using transforms in step_plot.

      if fov is 2*pi, plot a circle, otherwise plot a wedge.

      :param ax: Matplotlib axis.
      :param \*\*kwargs: Additional plotting options.
                         fov_color (str): Color of the field of view. Default is 'lightblue'.
                         fov_edge_color (str): Edge color of the field of view. Default is 'blue'.
                         fov_zorder (int): Z-order of the field of view. Default is 1.
                         fov_alpha (float): Transparency of the field of view. Default is 0.5.



   .. py:method:: plot_uncertainty(ax, **kwargs)

      To be completed.



   .. py:method:: plot_clear(all: bool = False)

      Clear all plotted elements from the axis.

      :param all: If True, also clears trail elements. If False, keeps trail elements. Defaults to False.
      :type all: bool



   .. py:method:: done()

      Check if the object has completed its task.

      :returns: True if the task is done, False otherwise.
      :rtype: bool



   .. py:method:: reset()

      Reset the object to its initial state.



   .. py:method:: remove()

      Remove the object from the environment.



   .. py:method:: get_vel_range() -> tuple[numpy.ndarray, numpy.ndarray]

      Get the velocity range considering acceleration limits.

      :returns: Minimum and maximum velocities.
      :rtype: tuple



   .. py:method:: get_info() -> ObjectInfo

      Get object information.

      :returns: Information about the object.
      :rtype: ObjectInfo



   .. py:method:: get_obstacle_info() -> ObstacleInfo

      Get information about the object as an obstacle.

      :returns: Obstacle-related information, including state, vertices, velocity, and radius.
      :rtype: ObstacleInfo



   .. py:method:: get_init_Gh() -> tuple[numpy.ndarray, numpy.ndarray]

      Get the initial generalized inequality matrices G and h for the convex object.

      :returns: Tuple containing initial G matrix and h vector.
      :rtype: tuple[np.ndarray, np.ndarray]



   .. py:method:: get_Gh() -> tuple[numpy.ndarray, numpy.ndarray]

      Get the generalized inequality matrices G and h for the convex object.

      :returns: Tuple containing G matrix and h vector.
      :rtype: tuple[np.ndarray, np.ndarray]



   .. py:property:: name
      :type: str


      Get the name of the object.

      :returns: The name of the object.
      :rtype: str


   .. py:property:: abbr
      :type: str


      Get the abbreviation of the object.

      :returns: The abbreviation of the object.
      :rtype: str


   .. py:property:: shape
      :type: str


      Get the shape name of the object.

      :returns: The shape name of the object.
      :rtype: str


   .. py:property:: z
      :type: float


      Get the z coordinate of the object. For 3D object, the z coordinate is the height of the object, for 2D object, the z coordinate is 0.

      :returns: The z coordinate of the object.
      :rtype: float


   .. py:property:: kinematics
      :type: Optional[str]


      Get the kinematics name of the object.

      :returns: The kinematics name of the object.
      :rtype: str


   .. py:property:: geometry
      :type: shapely.geometry.base.BaseGeometry


      Get the geometry Instance of the object.

      :returns: The geometry of the object.
      :rtype: shapely.geometry.base.BaseGeometry


   .. py:property:: centroid
      :type: numpy.ndarray


      Get the centroid of the object.

      :returns: The centroid of the object.
      :rtype: np.ndarray


   .. py:property:: id
      :type: int


      Get the id of the object.

      :returns: The id of the object.
      :rtype: int


   .. py:property:: state
      :type: numpy.ndarray


      Get the state of the object.

      :returns: The state of the object.
      :rtype: np.ndarray


   .. py:property:: init_state
      :type: numpy.ndarray


      Get the initial state of the object.

      :returns: The initial state of the object.
      :rtype: np.ndarray


   .. py:property:: velocity
      :type: numpy.ndarray


      Get the velocity of the object.

      :returns: The velocity of the object.
      :rtype: np.ndarray


   .. py:property:: goal
      :type: Optional[numpy.ndarray]


      Get the goal of the object.

      :returns: The goal of the object.
      :rtype: np.ndarray


   .. py:property:: goal_vertices
      :type: numpy.ndarray


      Get the goal vertices of the object.

      :returns: The goal vertices of the object.
      :rtype: np.ndarray


   .. py:property:: position
      :type: numpy.ndarray


      Get the position of the object.

      :returns: The position of the object .
      :rtype: np.ndarray


   .. py:property:: radius
      :type: float


      Get the radius of the object.

      :returns: The radius of the object.
      :rtype: float


   .. py:property:: length
      :type: float


      Get the length of the object.

      :returns: The length of the object.
      :rtype: float


   .. py:property:: width
      :type: float


      Get the width of the object.

      :returns: The width of the object.
      :rtype: float


   .. py:property:: wheelbase
      :type: float


      Get the wheelbase of the object.

      :returns: The wheelbase of the object.
      :rtype: float


   .. py:property:: radius_extend
      :type: float


      Get the radius of the object with a buffer.

      :returns: The radius of the object with a buffer.
      :rtype: float


   .. py:property:: arrive
      :type: bool


      Get the arrive flag of the object.

      :returns: The arrive flag of the object.
      :rtype: bool


   .. py:property:: collision
      :type: bool


      Get the collision flag of the object.

      :returns: The collision flag of the object.
      :rtype: bool


   .. py:property:: vertices
      :type: numpy.ndarray


      Get the vertices of the object.

      :returns: The vertices of the object.
      :rtype: np.ndarray


   .. py:property:: original_vertices
      :type: numpy.ndarray


      Get the original vertices of the object.

      :returns: The original vertices of the object before any transformations.
      :rtype: np.ndarray


   .. py:property:: original_geometry
      :type: shapely.geometry.base.BaseGeometry


      Get the original geometry of the object.

      :returns: The original geometry of the object.
      :rtype: shapely.geometry.base.BaseGeometry


   .. py:property:: original_centroid
      :type: numpy.ndarray


      Get the center of the object.

      :returns: The center of the object.
      :rtype: np.ndarray


   .. py:property:: original_state
      :type: numpy.ndarray


      Get the original state of the object from the original centroid.

      :returns: The original state of the object.
      :rtype: np.ndarray (3,1)


   .. py:property:: external_objects

      Get the environment objects that are not the self object.

      :returns: The environment objects that are not the self object.
      :rtype: list


   .. py:property:: ego_object

      Get the ego object (this object itself).

      :returns: The ego object (this object).
      :rtype: ObjectBase


   .. py:property:: possible_collision_objects

      Get the possible collision objects of the object from the geometry tree.

      :returns: The possible collision objects that could collide with this object.
      :rtype: list


   .. py:property:: desired_omni_vel

      Calculate the desired omnidirectional velocity.

      :param goal_threshold: Threshold for goal proximity.
      :type goal_threshold: float

      :returns: Desired velocity [vx, vy].
      :rtype: np.ndarray


   .. py:property:: rvo_neighbors

      Get the list of RVO neighbors.
      :returns: List of RVO neighbor states [x, y, vx, vy, radius].
      :rtype: list


   .. py:property:: rvo_neighbor_state

      Get the RVO state for this object.

      :returns: State [x, y, vx, vy, radius].
      :rtype: list


   .. py:property:: rvo_state

      Get the full RVO state including desired velocity.

      :returns: State [x, y, vx, vy, radius, vx_des, vy_des, theta].
      :rtype: list


   .. py:property:: velocity_xy

      Get the velocity in x and y directions.

      :returns: Velocity [vx, vy].
      :rtype: (2*1) np.ndarray


   .. py:property:: beh_config

      Get the behavior configuration of the object.

      :returns: The behavior configuration of the object.
      :rtype: dict


   .. py:property:: logger

      Get the logger of the env_param.

      :returns: The logger associated in the env_param.
      :rtype: Logger


   .. py:property:: heading

      Get the heading of the object.

      :returns: The heading of the object.
      :rtype: float


   .. py:property:: orientation

      Get the orientation of the object.

      :returns: The orientation angle of the object in radians.
      :rtype: float


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


.. py:class:: ObstacleDiff(color='k', state_dim=3, **kwargs)

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


.. py:class:: ObstacleOmni(color='k', state_dim=3, **kwargs)

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


.. py:class:: ObjectStatic(color='k', role='obstacle', state_dim=3, **kwargs)

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


   .. py:attribute:: static
      :value: True



.. py:class:: RobotAcker(color: str = 'y', state_dim: int = 4, description: str = 'car_green.png', **kwargs: Any)

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


.. py:class:: RobotDiff(color: str = 'g', state_dim: int = 3, **kwargs: Any)

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


.. py:class:: RobotOmni(color: str = 'g', state_dim: int = 3, **kwargs: Any)

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


.. py:class:: SensorFactory

   .. py:method:: create_sensor(state: numpy.ndarray, obj_id: int, **kwargs: Any) -> Any


.. py:class:: World(name: Optional[str] = 'world', height: float = 10, width: float = 10, step_time: float = 0.1, sample_time: float = 0.1, offset: Optional[list[float]] = None, control_mode: str = 'auto', collision_mode: str = 'stop', obstacle_map: Optional[Any] = None, mdownsample: int = 1, plot: Optional[dict[str, Any]] = None, status: str = 'None', **kwargs: Any)

   Represents the main simulation environment, managing objects and maps.

   .. attribute:: name

      Name of the world.

      :type: str

   .. attribute:: height

      Height of the world.

      :type: float

   .. attribute:: width

      Width of the world.

      :type: float

   .. attribute:: step_time

      Time interval between steps.

      :type: float

   .. attribute:: sample_time

      Time interval between samples.

      :type: float

   .. attribute:: offset

      Offset for the world's position.

      :type: list

   .. attribute:: control_mode

      Control mode ('auto' or 'keyboard').

      :type: str

   .. attribute:: collision_mode

      Collision mode ('stop',  , 'unobstructed').

      :type: str

   .. attribute:: obstacle_map

      Image file for the obstacle map.

   .. attribute:: mdownsample

      Downsampling factor for the obstacle map.

      :type: int

   .. attribute:: status

      Status of the world and objects.

   .. attribute:: plot

      Plot configuration for the world.

   Initialize the world object.

   :param name: Name of the world.
   :type name: str
   :param height: Height of the world.
   :type height: float
   :param width: Width of the world.
   :type width: float
   :param step_time: Time interval between steps.
   :type step_time: float
   :param sample_time: Time interval between samples.
   :type sample_time: float
   :param offset: Offset for the world's position.
   :type offset: list
   :param control_mode: Control mode ('auto' or 'keyboard').
   :type control_mode: str
   :param collision_mode: Collision mode ('stop',  , 'unobstructed').
   :type collision_mode: str
   :param obstacle_map: Image file for the obstacle map.
   :param mdownsample: Downsampling factor for the obstacle map.
   :type mdownsample: int
   :param plot: Plot configuration.
   :type plot: dict
   :param status: Initial simulation status.
   :type status: str


   .. py:attribute:: name


   .. py:attribute:: height
      :value: 10



   .. py:attribute:: width
      :value: 10



   .. py:attribute:: step_time
      :value: 0.1



   .. py:attribute:: sample_time
      :value: 0.1



   .. py:attribute:: offset
      :value: None



   .. py:attribute:: count
      :value: 0



   .. py:attribute:: sampling
      :value: True



   .. py:attribute:: x_range


   .. py:attribute:: y_range


   .. py:attribute:: plot_parse
      :value: None



   .. py:attribute:: status
      :value: 'None'



   .. py:method:: step() -> None

      Advance the simulation by one step.



   .. py:method:: gen_grid_map(obstacle_map: Optional[str], mdownsample: int = 1) -> tuple

      Generate a grid map for obstacles.

      :param obstacle_map: Path to the obstacle map image.
      :param mdownsample: Downsampling factor.
      :type mdownsample: int

      :returns: Grid map, obstacle indices, and positions.
      :rtype: tuple



   .. py:method:: get_map(resolution: float = 0.1, obstacle_list: Optional[list[Any]] = None) -> irsim.world.map.Map

      Get the map of the world with the given resolution.



   .. py:method:: reset() -> None

      Reset the world simulation.



   .. py:property:: time
      :type: float


      Get the current simulation time.

      :returns: Current time based on steps and step_time.
      :rtype: float


   .. py:property:: buffer_reso
      :type: float


      Get the maximum resolution of the world.

      :returns: Maximum resolution.
      :rtype: float


   .. py:method:: rgb2gray(rgb: numpy.ndarray) -> numpy.ndarray


