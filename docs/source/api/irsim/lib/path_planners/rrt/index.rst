irsim.lib.path_planners.rrt
===========================

.. py:module:: irsim.lib.path_planners.rrt

.. autoapi-nested-parse::

   Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

   author: AtsushiSakai(@Atsushi_twi)

   adapted by: Reinis Cimurs



Classes
-------

.. autoapisummary::

   irsim.lib.path_planners.rrt.RRT


Module Contents
---------------

.. py:class:: RRT(env_map: irsim.world.map.Map, robot_radius: float, expand_dis: float = 1.0, path_resolution: float = 0.25, goal_sample_rate: int = 5, max_iter: int = 500)

   Class for RRT planning

   Initialize RRT planner

   :param env_map: environment map where the planning will take place
   :type env_map: Env
   :param robot_radius: robot body modeled as circle with given radius
   :type robot_radius: float
   :param expand_dis: expansion distance
   :type expand_dis: float
   :param path_resolution: resolution of the path
   :type path_resolution: float
   :param goal_sample_rate: goal sample rate
   :type goal_sample_rate: int
   :param max_iter: max iteration count
   :type max_iter: int


   .. py:class:: Node(x: float, y: float)

      RRT Node

      Initialize Node

      :param x: x position of the node
      :type x: float
      :param y: y position of the node
      :type y: float


      .. py:attribute:: x


      .. py:attribute:: y


      .. py:attribute:: path_x
         :value: []



      .. py:attribute:: path_y
         :value: []



      .. py:attribute:: parent
         :value: None




   .. py:class:: AreaBounds(env_map: irsim.world.map.Map)

      Area Bounds

      Initialize AreaBounds

      :param env_map: environment where the planning will take place
      :type env_map: EnvBase



   .. py:attribute:: obstacle_list


   .. py:attribute:: play_area


   .. py:attribute:: min_rand
      :value: 0.0



   .. py:attribute:: max_rand


   .. py:attribute:: expand_dis
      :value: 1.0



   .. py:attribute:: path_resolution
      :value: 0.25



   .. py:attribute:: goal_sample_rate
      :value: 5



   .. py:attribute:: max_iter
      :value: 500



   .. py:attribute:: node_list
      :value: []



   .. py:attribute:: robot_radius


   .. py:method:: planning(start_pose: list[float], goal_pose: list[float], show_animation: bool = True) -> Optional[tuple[list[float], list[float]]]

      rrt path planning

      :param start_pose: start pose [x,y]
      :type start_pose: np.array
      :param goal_pose: goal pose [x,y]
      :type goal_pose: np.array
      :param show_animation: If true, shows the animation of planning process
      :type show_animation: bool

      :returns: xy position array of the final path
      :rtype: (np.array)



   .. py:method:: steer(from_node: Node, to_node: Node, extend_length: float = float('inf')) -> Node

      Generate a new node by steering from `from_node` towards `to_node`.

      This method incrementally moves from `from_node` in the direction of `to_node`,
      using a fixed step size (`self.path_resolution`) and not exceeding the
      specified `extend_length`. The result is a new node that approximates a path
      from the start node toward the goal, constrained by resolution and maximum
      step distance.

      If the final position is within one resolution step of `to_node`, it snaps the
      new node exactly to `to_node`.

      :param from_node: The node from which to begin extending.
      :type from_node: Node
      :param to_node: The target node to steer toward.
      :type to_node: Node
      :param extend_length: The maximum length to extend. Defaults to infinity.
      :type extend_length: float, optional

      :returns: A new node with updated position, path history (path_x, path_y),
      :rtype: (Node)



   .. py:method:: generate_final_course(goal_ind: int) -> tuple[list[float], list[float]]

      Generate the final path

      :param goal_ind: index of the final goal
      :type goal_ind: int

      :returns: xy position array of the final path
      :rtype: (np.array)



   .. py:method:: calc_dist_to_goal(x: float, y: float) -> float

      Calculate distance to goal

      :param x: x coordinate of the position
      :type x: float
      :param y: y coordinate of the position
      :type y: float

      :returns: distance to the goal
      :rtype: (float)



   .. py:method:: get_random_node() -> Node

      Create random node

      :returns: new random node
      :rtype: (Node)



   .. py:method:: draw_graph(rnd: Optional[Node] = None) -> None


   .. py:method:: plot_circle(x: float, y: float, size: float, color: str = '-b') -> None
      :staticmethod:



   .. py:method:: get_nearest_node_index(node_list: list[Node], rnd_node: Node) -> int
      :staticmethod:



   .. py:method:: check_if_outside_play_area(node: Node, play_area: AreaBounds) -> bool
      :staticmethod:



   .. py:method:: check_collision(node: Node, robot_radius: float) -> bool

      Check if node is acceptable - free of collisions

      :param node: node to check
      :type node: Node
      :param robot_radius: robot radius
      :type robot_radius: float

      :returns: True if there is no collision. False otherwise
      :rtype: (bool)



   .. py:method:: check_node(x: float, y: float, rr: float) -> bool

      Check positon for a collision

      :param x: x value of the position
      :type x: float
      :param y: y value of the position
      :type y: float
      :param rr: robot radius
      :type rr: float

      :returns: True if there is a collision. False otherwise
      :rtype: (bool)



   .. py:method:: calc_distance_and_angle(from_node: Node, to_node: Node) -> tuple[float, float]
      :staticmethod:



