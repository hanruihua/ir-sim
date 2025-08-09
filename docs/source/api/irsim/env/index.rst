irsim.env
=========

.. py:module:: irsim.env


Submodules
----------

.. toctree::
   :maxdepth: 1

   /api/irsim/env/env_base/index
   /api/irsim/env/env_base3d/index
   /api/irsim/env/env_config/index
   /api/irsim/env/env_logger/index
   /api/irsim/env/env_plot/index
   /api/irsim/env/env_plot3d/index


Classes
-------

.. autoapisummary::

   irsim.env.EnvBase
   irsim.env.EnvBase3D


Package Contents
----------------

.. py:class:: EnvBase(world_name: Optional[str] = None, display: bool = True, disable_all_plot: bool = False, save_ani: bool = False, full: bool = False, log_file: Optional[str] = None, log_level: str = 'INFO')

   The base class for simulation environments in IR-SIM.

   This class serves as the foundation for creating and managing robotic simulation
   environments. It reads YAML configuration files to create worlds, robots, obstacles,
   and map objects, and provides the core simulation loop functionality.

   :param world_name: Path to the world YAML configuration file.
                      If None, the environment will attempt to find a default configuration
                      or use a minimal setup.
   :type world_name: str, optional
   :param display: Whether to display the environment visualization.
                   Set to False for headless operation. Default is True.
   :type display: bool
   :param disable_all_plot: Whether to disable all plots and figures completely.
                            When True, no visualization will be created even if display is True.
                            Default is False.
   :type disable_all_plot: bool
   :param save_ani: Whether to save the simulation as an animation file.
                    Useful for creating videos of simulation runs. Default is False.
   :type save_ani: bool
   :param full: Whether to display the visualization in full screen mode.
                Only effective on supported platforms. Default is False.
   :type full: bool
   :param log_file: Path to the log file for saving simulation logs.
                    If None, logs will only be output to console.
   :type log_file: str, optional
   :param log_level: Logging level for the environment. Options include
                     'DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'. Default is 'INFO'.
   :type log_level: str

   .. attribute:: display

      Display flag for the environment.

      :type: bool

   .. attribute:: disable_all_plot

      Plot disable flag.

      :type: bool

   .. attribute:: save_ani

      Animation saving flag.

      :type: bool

   .. attribute:: objects

      List of all objects in the environment.

      :type: list

   .. attribute:: world

      The world object containing environment configuration.

      :type: World

   .. attribute:: robot_collection

      List of robot objects.

      :type: list

   .. attribute:: obstacle_collection

      List of obstacle objects.

      :type: list

   .. attribute:: map_collection

      List of map objects.

      :type: list

   .. rubric:: Example

   >>> # Create a basic environment
   >>> env = EnvBase("my_world.yaml")
   >>>
   >>> # Create headless environment for training
   >>> env = EnvBase("world.yaml", display=False, log_level="WARNING")
   >>>
   >>> # Create environment with animation saving
   >>> env = EnvBase("world.yaml", save_ani=True, full=True)


   .. py:attribute:: display
      :value: True



   .. py:attribute:: disable_all_plot
      :value: False



   .. py:attribute:: save_ani
      :value: False



   .. py:attribute:: env_config


   .. py:attribute:: object_factory


   .. py:attribute:: mouse


   .. py:attribute:: pause_flag
      :value: False



   .. py:method:: step(action: Optional[Union[numpy.ndarray, list[Any]]] = None, action_id: Optional[Union[int, list[int]]] = 0) -> None

      Perform a single simulation step in the environment.

      This method advances the simulation by one time step, applying the given actions
      to the specified robots and updating all objects in the environment.

      :param action: Action(s) to be performed in the environment.
                     Can be a single action or a list of actions. Action format depends on robot type:

                     - **Differential robot**: [linear_velocity, angular_velocity]
                     - **Omnidirectional robot**: [velocity_x, velocity_y]
                     - **Ackermann robot**: [linear_velocity, steering_angle]

                     If None, robots will use their default behavior or keyboard control if enabled.
      :type action: Union[np.ndarray, list], optional
      :param action_id: ID(s) of the robot(s) to apply the action(s) to.
                        Can be a single robot ID or a list of IDs. Default is 0 (first robot).
                        If action is a list and action_id is a single int, all actions will be
                        applied to robots sequentially starting from action_id.
      :type action_id: Union[int, list], optional

      .. note::

         - If the environment is paused, this method returns without performing any updates.
         - The method automatically handles collision detection, status updates, and plotting.
         - In keyboard control mode, the action parameter is ignored and keyboard input is used.

      .. rubric:: Example

      >>> # Move first robot with differential drive
      >>> env.step([1.0, 0.5])  # 1.0 m/s forward, 0.5 rad/s turn
      >>>
      >>> # Move specific robot by ID
      >>> env.step([0.8, 0.0], action_id=2)  # Move robot with ID 2
      >>>
      >>> # Move multiple robots
      >>> actions = [[1.0, 0.0], [0.5, 0.3]]
      >>> env.step(actions, action_id=[0, 1])  # Move robots 0 and 1



   .. py:method:: render(interval: float = 0.02, figure_kwargs: Optional[dict[str, Any]] = None, mode: str = 'dynamic', **kwargs: Any) -> None

      Render the environment.

      :param interval: Time interval between frames in seconds.
      :type interval: float
      :param figure_kwargs: Additional keyword arguments for saving figures, see `savefig <https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html>`_ for detail.
      :type figure_kwargs: dict
      :param mode: "dynamic", "static", "all" to specify which type of objects to draw and clear.
      :type mode: str
      :param kwargs: Additional keyword arguments for drawing components. see :py:meth:`.ObjectBase.plot` function for detail.



   .. py:method:: show() -> None

      Show the environment figure.



   .. py:method:: draw_trajectory(traj: list[Any], traj_type: str = 'g-', **kwargs: Any) -> None

      Draw the trajectory on the environment figure.

      :param traj: List of trajectory points (2 * 1 vector).
      :type traj: list
      :param traj_type: Type of the trajectory line, see matplotlib plot function for detail.
      :param \*\*kwargs: Additional keyword arguments for drawing the trajectory, see :py:meth:`.EnvPlot.draw_trajectory` for detail.



   .. py:method:: draw_points(points: list[Any], s: int = 30, c: str = 'b', refresh: bool = True, **kwargs: Any) -> None

      Draw points on the environment figure.

      :param points: List of points (2*1) to be drawn.
                     or (np.array): (2, Num) to be drawn.
      :type points: list
      :param s: Size of the points.
      :type s: int
      :param c: Color of the points.
      :type c: str
      :param refresh: Flag to refresh the points in the figure.
      :type refresh: bool
      :param \*\*kwargs: Additional keyword arguments for drawing the points, see `ax.scatter <https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.scatter.html>`_ function for detail.



   .. py:method:: draw_box(vertex: numpy.ndarray, refresh: bool = False, color: str = '-b') -> None

      Draw a box by the vertices.

      :param vertex: matrix of vertices, point_dim*vertex_num
      :type vertex: np.ndarray
      :param refresh: whether to refresh the plot, default True
      :type refresh: bool
      :param color: color of the box, default '-b'
      :type color: str



   .. py:method:: draw_quiver(point: Any, refresh: bool = False, **kwargs: Any) -> None

      Draw a single quiver (arrow) on the environment figure.

      :param point: Point data for the quiver
      :param refresh: Flag to refresh the quiver in the figure, default False
      :type refresh: bool
      :param \*\*kwargs: Additional keyword arguments for drawing the quiver



   .. py:method:: draw_quivers(points: Any, refresh: bool = False, **kwargs: Any) -> None

      Draw multiple quivers (arrows) on the environment figure.

      :param points: Points data for the quivers
      :param refresh: Flag to refresh the quivers in the figure, default False
      :type refresh: bool
      :param \*\*kwargs: Additional keyword arguments for drawing the quivers



   .. py:method:: end(ending_time: float = 3.0, **kwargs: Any) -> None

      End the simulation, save the animation, and close the environment.

      :param ending_time: Time in seconds to wait before closing the figure, default is 3 seconds.
      :type ending_time: float
      :param \*\*kwargs: Additional keyword arguments for saving the animation, see :py:meth:`.EnvPlot.save_animate` for detail.



   .. py:method:: done(mode: str = 'all') -> Optional[bool]

      Check if the simulation should terminate based on robot completion status.

      This method evaluates whether robots in the environment have reached their
      goals or completed their tasks, using different criteria based on the mode.

      :param mode: Termination condition mode. Options are:

                   - "all": Simulation is done when ALL robots have completed their tasks
                   - "any": Simulation is done when ANY robot has completed its task

                   Default is "all".
      :type mode: str

      :returns: True if the termination condition is met based on the specified mode,
                False otherwise. Returns False if no robots are present in the environment.
      :rtype: bool

      .. rubric:: Example

      >>> # Check if all robots have reached their goals
      >>> if env.done(mode="all"):
      ...     print("All robots completed!")
      >>>
      >>> # Check if any robot has completed
      >>> if env.done(mode="any"):
      ...     print("At least one robot completed!")



   .. py:method:: step_status() -> None

      Update and log the current status of all robots in the environment.

      This method checks the arrival status of all robots and logs information
      about which robots have reached their goals. It's automatically called
      during each simulation step.

      .. note::

         This is an internal method primarily used for status tracking and logging.
         The status information is automatically updated during simulation steps.



   .. py:method:: pause() -> None

      Pause the simulation execution.

      When paused, calls to :py:meth:`step` will return immediately without
      performing any simulation updates. The environment status is set to "Pause".

      .. rubric:: Example

      >>> env.pause()
      >>> env.step([1.0, 0.0])  # This will have no effect while paused



   .. py:method:: resume() -> None

      Resume the simulation execution after being paused.

      Re-enables simulation updates and sets the environment status back to "Running".
      Subsequent calls to :py:meth:`step` will function normally.

      .. rubric:: Example

      >>> env.pause()
      >>> # ... some time later ...
      >>> env.resume()
      >>> env.step([1.0, 0.0])  # This will now work again



   .. py:method:: reset() -> None

      Reset the environment to its initial state.

      This method resets all objects, robots, obstacles, and the world to their
      initial configurations. It also resets the visualization and sets the
      environment status to "Reset".

      The reset process includes:
      - Resetting all objects to their initial positions and states
      - Clearing accumulated trajectories and sensor data
      - Resetting the world timer and status
      - Refreshing the visualization plot

      .. rubric:: Example

      >>> # Reset environment after simulation
      >>> env.reset()
      >>> # Environment is now ready for a new simulation run



   .. py:method:: reset_plot() -> None

      Reset the environment figure.



   .. py:method:: random_obstacle_position(range_low: Optional[list[float]] = None, range_high: Optional[list[float]] = None, ids: Optional[list[int]] = None, non_overlapping: bool = False) -> None

      Random obstacle positions in the environment.

      :param range_low: Lower bound of the random range for the obstacle states. Default is [0, 0, -3.14].
      :type range_low: list [x, y, theta]
      :param range_high: Upper bound of the random range for the obstacle states. Default is [10, 10, 3.14].
      :type range_high: list [x, y, theta]
      :param ids: A list of IDs of objects for which to set random positions. Default is None.
      :type ids: list
      :param non_overlapping: If set, the obstacles that will be reset to random obstacles will not overlap with other obstacles. Default is False.
      :type non_overlapping: bool



   .. py:method:: random_polygon_shape(center_range: Optional[list[float]] = None, avg_radius_range: Optional[list[float]] = None, irregularity_range: Optional[list[float]] = None, spikeyness_range: Optional[list[float]] = None, num_vertices_range: Optional[list[int]] = None) -> None

      Random polygon shapes for the obstacles in the environment.

      :param center_range: Range of the center of the polygon. Default is [0, 0, 10, 10].
      :type center_range: list
      :param avg_radius_range: Range of the average radius of the polygon. Default is [0.1, 1].
      :type avg_radius_range: list
      :param irregularity_range: Range of the irregularity of the polygon. Default is [0, 1].
      :type irregularity_range: list
      :param spikeyness_range: Range of the spikeyness of the polygon. Default is [0, 1].
      :type spikeyness_range: list
      :param num_vertices_range: Range of the number of vertices of the polygon. Default is [4, 10].
      :type num_vertices_range: list
      :param center: a pair representing the center of the circumference used
                     to generate the polygon.
      :type center: Tuple[float, float]
      :param avg_radius: the average radius (distance of each generated vertex to
                         the center of the circumference) used to generate points
                         with a normal distribution.
      :type avg_radius: float
      :param irregularity: 0 - 1
                           variance of the spacing of the angles between consecutive
                           vertices.
      :type irregularity: float
      :param spikeyness: 0 - 1
                         variance of the distance of each vertex to the center of
                         the circumference.
      :type spikeyness: float
      :param num_vertices: the number of vertices of the polygon.
      :type num_vertices: int



   .. py:method:: create_obstacle(**kwargs: Any)

      Create an obstacle in the environment.

      :param \*\*kwargs: Additional parameters for obstacle creation.
                         see ObjectFactory.create_obstacle for detail

      :returns: An instance of an obstacle.
      :rtype: Obstacle



   .. py:method:: add_object(obj: irsim.world.ObjectBase) -> None

      Add the object to the environment.

      :param obj: The object to be added to the environment.
      :type obj: ObjectBase



   .. py:method:: add_objects(objs: list[irsim.world.ObjectBase]) -> None

      Add the objects to the environment.

      :param objs: List of objects to be added to the environment.
      :type objs: list



   .. py:method:: delete_object(target_id: int) -> None

      Delete the object with the given id.

      :param target_id: ID of the object to be deleted.
      :type target_id: int



   .. py:method:: delete_objects(target_ids: list[int]) -> None

      Delete the objects with the given ids.

      :param target_ids: List of IDs of objects to be deleted.
      :type target_ids: list



   .. py:method:: build_tree() -> None

      Build the geometry tree for the objects in the environment to detect the possible collision objects.



   .. py:method:: get_robot_state() -> numpy.ndarray

      Get the current state of the robot.

      :returns: 3*1 vector [x, y, theta]
      :rtype: state



   .. py:method:: get_lidar_scan(id: int = 0) -> dict[str, Any]

      Get the lidar scan of the robot with the given id.

      :param id: Id of the robot.
      :type id: int

      :returns: Dict of lidar scan points, see :py:meth:`.world.sensors.lidar2d.Lidar2D.get_scan` for detail.
      :rtype: Dict



   .. py:method:: get_lidar_offset(id: int = 0) -> list[float]

      Get the lidar offset of the robot with the given id.


      :param id: Id of the robot.
      :type id: int

      :returns: Lidar offset of the robot, [x, y, theta]
      :rtype: list of float



   .. py:method:: get_obstacle_info_list() -> list[dict[str, Any]]

      Get the information of the obstacles in the environment.

      :returns: List of obstacle information, see :py:meth:`.ObjectBase.get_obstacle_info` for detail.
      :rtype: list of dict



   .. py:method:: get_robot_info(id: int = 0) -> Any

      Get the information of the robot with the given id.

      :param id: Id of the robot.
      :type id: int

      :returns: see :py:meth:`.ObjectBase.get_info` for detail



   .. py:method:: get_robot_info_list() -> list[dict[str, Any]]

      Get the information of the robots in the environment.

      :returns: List of robot information, see :py:meth:`.ObjectBase.get_info` for detail.
      :rtype: list of dict



   .. py:method:: get_map(resolution: float = 0.1) -> Any

      Get the map of the environment with the given resolution.

      :param resolution: Resolution of the map. Default is 0.1.
      :type resolution: float

      :returns: The map of the environment with the specified resolution.



   .. py:method:: set_title(title: str) -> None

      Set the title of the plot.



   .. py:method:: save_figure(save_name: Optional[str] = None, include_index: bool = False, save_gif: bool = False, **kwargs: Any) -> None

      Save the current figure.

      :param save_name: Name of the file with format to save the figure. Default is None.
      :type save_name: str
      :param include_index: Flag to include index in the saved file name. Default is False.
      :type include_index: bool
      :param save_gif: Flag to save as GIF format. Default is False.
      :type save_gif: bool
      :param \*\*kwargs: Additional keyword arguments for saving the figure, see `savefig <https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html>`_ function for detail.



   .. py:method:: load_behavior(behaviors: str = 'behavior_methods') -> None

      Load behavior parameters from the script. Please refer to the behavior_methods.py file for more details.
      Please make sure the python file is placed in the same folder with the implemented script.

      :param behaviors: name of the bevavior script.
      :type behaviors: str



   .. py:property:: robot_list
      :type: list[irsim.world.ObjectBase]


      Get the list of robots in the environment.

      :returns: List of robot objects [].
      :rtype: list


   .. py:property:: obstacle_list
      :type: list[irsim.world.ObjectBase]


      Get the list of obstacles in the environment.

      :returns: List of obstacle objects.
      :rtype: list


   .. py:property:: objects
      :type: list[irsim.world.ObjectBase]


      Get all objects in the environment.

      :returns: List of all objects in the environment.
      :rtype: list


   .. py:property:: static_objects
      :type: list[irsim.world.ObjectBase]


      Get all static objects in the environment.

      :returns: List of static objects in the environment.
      :rtype: list


   .. py:property:: dynamic_objects
      :type: list[irsim.world.ObjectBase]


      Get all dynamic objects in the environment.

      :returns: List of dynamic objects in the environment.
      :rtype: list


   .. py:property:: step_time
      :type: float


      Get the step time of the simulation.

      :returns: Step time of the simulation from the world.
      :rtype: float


   .. py:property:: time
      :type: float


      Get the time of the simulation.


   .. py:property:: status
      :type: str


      Get the status of the environment.


   .. py:property:: robot
      :type: irsim.world.ObjectBase


      Get the first robot in the environment.

      :returns: The first robot object in the robot list.
      :rtype: Robot


   .. py:property:: obstacle_number
      :type: int


      Get the number of obstacles in the environment.

      :returns: Number of obstacles in the environment.
      :rtype: int


   .. py:property:: robot_number
      :type: int


      Get the number of robots in the environment.

      :returns: Number of robots in the environment.
      :rtype: int


   .. py:property:: logger
      :type: irsim.env.env_logger.EnvLogger


      Get the environment logger.

      :returns: The logger instance for the environment.
      :rtype: EnvLogger


   .. py:property:: key_vel
      :type: Any



   .. py:property:: key_id
      :type: int



   .. py:property:: mouse_pos
      :type: Any



   .. py:property:: mouse_left_pos
      :type: Any



   .. py:property:: mouse_right_pos
      :type: Any



.. py:class:: EnvBase3D(world_name: Optional[str], **kwargs: Any)

   Bases: :py:obj:`irsim.env.EnvBase`


   This class is the 3D version of the environment class. It inherits from the :py:class:`.EnvBase` class to provide the 3D plot environment.


