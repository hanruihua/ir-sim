irsim.lib.path_planners.a_star
==============================

.. py:module:: irsim.lib.path_planners.a_star

.. autoapi-nested-parse::

   A* grid planning

   author: Atsushi Sakai(@Atsushi_twi)
           Nikos Kanargias (nkana@tee.gr)

   adapted by: Reinis Cimurs

   See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)



Classes
-------

.. autoapisummary::

   irsim.lib.path_planners.a_star.AStarPlanner


Module Contents
---------------

.. py:class:: AStarPlanner(env_map: irsim.world.map.Map, resolution: float)

   
   Initialize A* planner

   :param env_map: environment map where the planning will take place
   :type env_map: Env
   :param resolution: grid resolution [m]
   :type resolution: float


   .. py:attribute:: resolution


   .. py:attribute:: obstacle_list


   .. py:attribute:: x_width


   .. py:attribute:: y_width


   .. py:attribute:: motion


   .. py:class:: Node(x: int, y: int, cost: float, parent_index: int)

      Node class

      Initialize Node

      :param x: x position of the node
      :type x: float
      :param y: y position of the node
      :type y: float
      :param cost: heuristic cost of the node
      :type cost: float
      :param parent_index: Nodes parent index
      :type parent_index: int


      .. py:attribute:: x


      .. py:attribute:: y


      .. py:attribute:: cost


      .. py:attribute:: parent_index



   .. py:method:: planning(start_pose: list[float], goal_pose: list[float], show_animation: bool = True) -> tuple[list[float], list[float]]

      A star path search

      :param start_pose: start pose [x,y]
      :type start_pose: np.array
      :param goal_pose: goal pose [x,y]
      :type goal_pose: np.array
      :param show_animation: If true, shows the animation of planning process
      :type show_animation: bool

      :returns: xy position array of the final path
      :rtype: (np.array)



   .. py:method:: calc_final_path(goal_node: Node, closed_set: dict) -> tuple[list[float], list[float]]

      Generate the final path

      :param goal_node: final goal node
      :type goal_node: Node
      :param closed_set: dict of closed nodes
      :type closed_set: dict

      :returns: list of x positions of final path
                ry (list): list of y positions of final path
      :rtype: rx (list)



   .. py:method:: calc_heuristic(n1: Node, n2: Node) -> float
      :staticmethod:



   .. py:method:: calc_grid_position(index: int, min_position: float) -> float

      calc grid position

      :param index: index of a node
      :type index: int
      :param min_position: min value of search space
      :type min_position: float

      :returns: position of coordinates along the given axis
      :rtype: (float)



   .. py:method:: calc_xy_index(position: float, min_pos: float) -> int

      calc xy index of node

      :param position: position of a node
      :type position: float
      :param min_pos: min value of search space
      :type min_pos: float

      :returns: index of position along the given axis
      :rtype: (int)



   .. py:method:: calc_grid_index(node: Node) -> int

      calc grid index of node

      :param node: node to calculate the index for
      :type node: Node

      :returns: grid index of the node
      :rtype: (float)



   .. py:method:: verify_node(node: Node) -> bool

      Check if node is acceptable - within limits of search space and free of collisions

      :param node: node to check
      :type node: Node

      :returns: True if node is acceptable. False otherwise
      :rtype: (bool)



   .. py:method:: check_node(x: int, y: int) -> bool

      Check positon for a collision

      :param x: x value of the position
      :type x: float
      :param y: y value of the position
      :type y: float

      :returns: True if there is a collision. False otherwise
      :rtype: result (bool)



   .. py:method:: get_motion_model() -> list[list[float]]
      :staticmethod:



