irsim.lib.algorithm
===================

.. py:module:: irsim.lib.algorithm

.. autoapi-nested-parse::

   Core algorithms for IR-SIM simulation.

   This package contains:
   - kinematics: Robot kinematics functions
   - rvo: Reciprocal Velocity Obstacle algorithm
   - generation: Polygon generation utilities



Submodules
----------

.. toctree::
   :maxdepth: 1

   /api/irsim/lib/algorithm/generation/index
   /api/irsim/lib/algorithm/kinematics/index
   /api/irsim/lib/algorithm/rvo/index


Classes
-------

.. autoapisummary::

   irsim.lib.algorithm.reciprocal_vel_obs


Functions
---------

.. autoapisummary::

   irsim.lib.algorithm.generate_polygon
   irsim.lib.algorithm.random_generate_polygon
   irsim.lib.algorithm.ackermann_kinematics
   irsim.lib.algorithm.differential_kinematics
   irsim.lib.algorithm.omni_kinematics


Package Contents
----------------

.. py:function:: generate_polygon(center: list[float], avg_radius: float, irregularity: float, spikeyness: float, num_vertices: int) -> numpy.ndarray

   Generate a random polygon around a center point.

   :param center: Center of the polygon.
   :type center: Tuple[float, float]
   :param avg_radius: Average radius from the center to vertices.
   :type avg_radius: float
   :param irregularity: Variance of angle spacing between vertices. Range [0, 1]
   :type irregularity: float
   :param spikeyness: Variance of radius from the center. Range [0, 1]
   :type spikeyness: float
   :param num_vertices: Number of vertices for the polygon.
   :type num_vertices: int

   :returns: Vertices of the polygon in CCW order.
   :rtype: numpy.ndarray


.. py:function:: random_generate_polygon(number: int = 1, center_range: Optional[list[float]] = None, avg_radius_range: Optional[list[float]] = None, irregularity_range: Optional[list[float]] = None, spikeyness_range: Optional[list[float]] = None, num_vertices_range: Optional[list[int]] = None, **kwargs: Any) -> Union[numpy.ndarray, list[numpy.ndarray]]

   reference: https://stackoverflow.com/questions/8997099/algorithm-to-generate-random-2d-polygon

   Generate random polygons with specified properties.

   :param number: Number of polygons to generate (default 1).
   :type number: int
   :param center_range: Range for the polygon center [min_x, min_y, max_x, max_y].
   :type center_range: List[float]
   :param avg_radius_range: Range for the average radius of the polygons.
   :type avg_radius_range: List[float]
   :param irregularity_range: Range for the irregularity of the polygons.
   :type irregularity_range: List[float]
   :param spikeyness_range: Range for the spikeyness of the polygons.
   :type spikeyness_range: List[float]
   :param num_vertices_range: Range for the number of vertices of the polygons.
   :type num_vertices_range: List[int]

   :returns: List of vertices for each polygon or a single polygon's vertices if number=1.


.. py:function:: ackermann_kinematics(state: numpy.ndarray, velocity: numpy.ndarray, step_time: float, noise: bool = False, alpha: Optional[list[float]] = None, mode: str = 'steer', wheelbase: float = 1) -> numpy.ndarray

   Calculate the next state for an Ackermann steering vehicle.

   :param state: A 4x1 vector [x, y, theta, steer_angle] representing the current state.
   :param velocity: A 2x1 vector representing the current velocities, format depends on mode.
                    For "steer" mode, [linear, steer_angle] is expected.
                    For "angular" mode, [linear, angular] is expected.
   :param step_time: The time step for the simulation.
   :param noise: Boolean indicating whether to add noise to the velocity (default False).
   :param alpha: List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]). alpha[0] and alpha[1] are for linear velocity, alpha[2] and alpha[3] are for angular velocity.
   :param mode: The kinematic mode, either "steer" or "angular" (default "steer").
   :param wheelbase: The distance between the front and rear axles (default 1).

   :returns: A 4x1 vector representing the next state.
   :rtype: new_state


.. py:function:: differential_kinematics(state: numpy.ndarray, velocity: numpy.ndarray, step_time: float, noise: bool = False, alpha: Optional[list[float]] = None) -> numpy.ndarray

   Calculate the next state for a differential wheel robot.

   :param state: A 3x1 vector [x, y, theta] representing the current position and orientation.
   :param velocity: A 2x1 vector [linear, angular] representing the current velocities.
   :param step_time: The time step for the simulation.
   :param noise: Boolean indicating whether to add noise to the velocity (default False).
   :param alpha: List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]). alpha[0] and alpha[1] are for linear velocity, alpha[2] and alpha[3] are for angular velocity.

   :returns: A 3x1 vector [x, y, theta] representing the next state.
   :rtype: next_state


.. py:function:: omni_kinematics(state: numpy.ndarray, velocity: numpy.ndarray, step_time: float, noise: bool = False, alpha: Optional[list[float]] = None) -> numpy.ndarray

   Calculate the next position for an omnidirectional robot.

   :param state: A 2x1 vector [x, y] representing the current position.
   :param velocity: A 2x1 vector [vx, vy] representing the current velocities.
   :param step_time: The time step for the simulation.
   :param noise: Boolean indicating whether to add noise to the velocity (default False).
   :param alpha: List of noise parameters for the velocity model (default [0.03, 0.03]). alpha[0] is for x velocity, alpha[1] is for y velocity.

   :returns: A 2x1 vector [x, y] representing the next position.
   :rtype: new_position


.. py:class:: reciprocal_vel_obs(state: list, obs_state_list=None, vxmax=1.5, vymax=1.5, acce=0.5, factor=1.0)

   A class to implement the Reciprocal Velocity Obstacle (RVO) algorithm for multi-robot collision avoidance.

   :param state: The rvo state of the agent [x, y, vx, vy, radius, vx_des, vy_des].
   :type state: list
   :param obs_state_list: List of states of static obstacles [[x, y, vx, vy, radius]].
   :type obs_state_list: list
   :param vxmax: Maximum velocity in the x direction.
   :type vxmax: float
   :param vymax: Maximum velocity in the y direction.
   :type vymax: float
   :param acce: Acceleration limit.
   :type acce: float


   .. py:attribute:: state


   .. py:attribute:: obs_state_list
      :value: None



   .. py:attribute:: vxmax
      :value: 1.5



   .. py:attribute:: vymax
      :value: 1.5



   .. py:attribute:: acce
      :value: 0.5



   .. py:attribute:: factor
      :value: 1.0



   .. py:method:: update(state, obs_state_list)


   .. py:method:: cal_vel(mode='rvo')

      Calculate the velocity of the agent based on the Reciprocal Velocity Obstacle (RVO) algorithm.

      :param mode: The vo configure to calculate the velocity. It can be "rvo", "hrvo", or "vo".
                   - rvo: Reciprocal Velocity Obstacle (RVO) algorithm, for multi-robot collision avoidance.
                   - hrvo: Hybrid Reciprocal Velocity Obstacle (HRVO) algorithm, for multi-robot collision avoidance.
                   - vo: Velocity Obstacle (VO) algorithm, for obstacle-robot collision avoidance.
      :type mode: str



   .. py:method:: config_rvo()


   .. py:method:: config_rvo_mode(obstacle)


   .. py:method:: config_hrvo()


   .. py:method:: config_hrvo_mode(obstacle)


   .. py:method:: config_vo()


   .. py:method:: config_vo_mode(obstacle)


   .. py:method:: vel_candidate(rvo_list)


   .. py:method:: vo_out(vx, vy, rvo_list)


   .. py:method:: vel_select(vo_outside, vo_inside)


   .. py:method:: penalty(vel, vel_des, factor)


   .. py:method:: between_vector(line_left_vector, line_right_vector, line_vector)
      :staticmethod:



   .. py:method:: cross_product(vector1, vector2)
      :staticmethod:



