irsim.lib
=========

.. py:module:: irsim.lib


Submodules
----------

.. toctree::
   :maxdepth: 1

   /api/irsim/lib/algorithm/index
   /api/irsim/lib/behavior/index
   /api/irsim/lib/handler/index
   /api/irsim/lib/path_planners/index


Attributes
----------

.. autoapisummary::

   irsim.lib.kinematics_factory


Classes
-------

.. autoapisummary::

   irsim.lib.Behavior
   irsim.lib.reciprocal_vel_obs
   irsim.lib.KinematicsFactory
   irsim.lib.GeometryFactory


Functions
---------

.. autoapisummary::

   irsim.lib.differential_kinematics
   irsim.lib.ackermann_kinematics
   irsim.lib.omni_kinematics
   irsim.lib.register_behavior
   irsim.lib.random_generate_polygon
   irsim.lib.generate_polygon


Package Contents
----------------

.. py:function:: differential_kinematics(state, velocity, step_time, noise=False, alpha=[0.03, 0, 0, 0.03])

   Calculate the next state for a differential wheel robot.

   :param state: A 3x1 vector [x, y, theta] representing the current position and orientation.
   :param velocity: A 2x1 vector [linear, angular] representing the current velocities.
   :param step_time: The time step for the simulation.
   :param noise: Boolean indicating whether to add noise to the velocity (default False).
   :param alpha: List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]). alpha[0] and alpha[1] are for linear velocity, alpha[2] and alpha[3] are for angular velocity.

   :returns: A 3x1 vector [x, y, theta] representing the next state.
   :rtype: next_state


.. py:function:: ackermann_kinematics(state, velocity, step_time, noise=False, alpha=[0.03, 0, 0, 0.03], mode='steer', wheelbase=1)

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


.. py:function:: omni_kinematics(state, velocity, step_time, noise=False, alpha=[0.03, 0, 0, 0.03])

   Calculate the next position for an omnidirectional robot.

   :param state: A 2x1 vector [x, y] representing the current position.
   :param velocity: A 2x1 vector [vx, vy] representing the current velocities.
   :param step_time: The time step for the simulation.
   :param noise: Boolean indicating whether to add noise to the velocity (default False).
   :param alpha: List of noise parameters for the velocity model (default [0.03, 0.03]). alpha[0] is for x velocity, alpha[1] is for y velocity.

   :returns: A 2x1 vector [x, y] representing the next position.
   :rtype: new_position


.. py:data:: kinematics_factory

.. py:function:: register_behavior(kinematics: str, action: str)

   decorator to register a method in the behavior registry

   :param kinematics: only support diff, omni, or acker
   :param action: defined action for the kinematics


.. py:class:: Behavior(object_info=None, behavior_dict=None)

   Represents the behavior of an agent in the simulation.

   :param object_info: Object information from the object_base class ObjectInfo.
   :type object_info: object
   :param behavior_dict: Dictionary containing behavior parameters for different behaviors.
                         Name Options include: 'dash', 'rvo'.
                         target_roles:

                         - 'all': all objects in the environment will be considered within this behavior.
                         - 'obstacle': only obstacles will be considered within this behavior.
                         - 'robot': only robots will be considered within this behavior.
   :type behavior_dict: dict

   Initializes the Behavior class with object information and behavior parameters.

   :param object_info: Information about the agent.
   :type object_info: object
   :param behavior_dict: Behavior parameters.
   :type behavior_dict: dict


   .. py:attribute:: object_info
      :value: None



   .. py:attribute:: behavior_dict


   .. py:method:: gen_vel(ego_object, external_objects=[])

      Generate velocity for the agent based on the behavior dictionary.

      :param ego_object: the object itself
      :param external_objects: all the other objects in the environment

      :returns: Generated velocity for the agent.
      :rtype: np.array (2, 1)



   .. py:method:: load_behavior(behaviors: str = '.behavior_methods')

      Load behavior parameters from the script.

      :param behaviors: name of the bevavior script.
      :type behaviors: str



   .. py:method:: invoke_behavior(kinematics: str, action: str, **kwargs: Any) -> Any

      Invoke a specific behavior method based on kinematics model and action type.

      This method looks up and executes the appropriate behavior function from the
      behavior registry based on the combination of kinematics model and action name.

      :param kinematics: Kinematics model identifier. Supported values:

                         - 'diff': Differential drive kinematics
                         - 'omni': Omnidirectional kinematics
                         - 'acker': Ackermann steering kinematics
      :type kinematics: str
      :param action: Behavior action name. Examples:

                     - 'dash': Direct movement toward goal
                     - 'rvo': Reciprocal Velocity Obstacles for collision avoidance
      :type action: str
      :param \*\*kwargs: Additional keyword arguments passed to the behavior function.
                         Common parameters include ego_object, external_objects, goal, etc.

      :returns: Generated velocity vector (2x1) in the format appropriate
                for the specified kinematics model.
      :rtype: np.ndarray

      :raises ValueError: If no behavior method is found for the given kinematics
          and action combination.

      .. rubric:: Example

      >>> # Invoke differential drive dash behavior
      >>> vel = behavior.invoke_behavior('diff', 'dash',
      ...                               ego_object=robot,
      ...                               external_objects=obstacles)



   .. py:property:: logger


.. py:function:: random_generate_polygon(number=1, center_range=[0, 0, 0, 0], avg_radius_range=[0.1, 1], irregularity_range=[0, 1], spikeyness_range=[0, 1], num_vertices_range=[4, 10], **kwargs)

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


.. py:function:: generate_polygon(center, avg_radius, irregularity, spikeyness, num_vertices)

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


.. py:class:: reciprocal_vel_obs(state: list, obs_state_list=[], vxmax=1.5, vymax=1.5, acce=0.5, factor=1.0)

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
      :value: []



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



.. py:class:: KinematicsFactory

   Factory class to create kinematics handlers.


   .. py:method:: create_kinematics(name: str = None, noise: bool = False, alpha: list = None, mode: str = 'steer', wheelbase: float = None, role: str = 'robot') -> KinematicsHandler
      :staticmethod:



.. py:class:: GeometryFactory

   Factory class to create geometry handlers.


   .. py:method:: create_geometry(name: str = 'circle', **kwargs) -> geometry_handler
      :staticmethod:



