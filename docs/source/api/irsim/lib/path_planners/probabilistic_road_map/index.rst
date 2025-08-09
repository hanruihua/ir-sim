irsim.lib.path_planners.probabilistic_road_map
==============================================

.. py:module:: irsim.lib.path_planners.probabilistic_road_map

.. autoapi-nested-parse::

   Probabilistic Road Map (PRM) Planner

   author: Atsushi Sakai (@Atsushi_twi)

   adapted by: Reinis Cimurs



Classes
-------

.. autoapisummary::

   irsim.lib.path_planners.probabilistic_road_map.Node
   irsim.lib.path_planners.probabilistic_road_map.PRMPlanner


Module Contents
---------------

.. py:class:: Node(x: float, y: float, cost: float, parent_index: int)

   Node class for dijkstra search

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


.. py:class:: PRMPlanner(env_map: irsim.world.map.Map, robot_radius: float, n_sample: int = 500, n_knn: int = 10, max_edge_len: float = 30.0)

   
   Initialize PRM planner

   :param env_map: environment map where the planning will take place
   :type env_map: Env
   :param robot_radius: robot body modeled as circle with given radius
   :type robot_radius: float
   :param n_sample: number of samples
   :type n_sample: int
   :param n_knn: number of edges
   :type n_knn: int
   :param max_edge_len: max edge length
   :type max_edge_len: float


   .. py:attribute:: rr


   .. py:attribute:: obstacle_list


   .. py:attribute:: n_sample
      :value: 500



   .. py:attribute:: n_knn
      :value: 10



   .. py:attribute:: max_edge_len
      :value: 30.0



   .. py:method:: planning(start_pose: list[float], goal_pose: list[float], rng: Optional[Any] = None, show_animation: bool = True) -> Optional[tuple[list[float], list[float]]]

      A star path search

      :param start_pose: start pose [x,y]
      :type start_pose: np.array
      :param goal_pose: goal pose [x,y]
      :type goal_pose: np.array
      :param rng: Random generator
      :type rng: Optional
      :param show_animation: If true, shows the animation of planning process
      :type show_animation: bool

      :returns: xy position array of the final path
      :rtype: (np.array)



   .. py:method:: check_node(x: float, y: float, rr: float) -> bool

      Check positon for a collision

      :param x: x value of the position
      :type x: float
      :param y: y value of the position
      :type y: float

      :returns: True if there is a collision. False otherwise
      :rtype: (bool)



   .. py:method:: is_collision(sx: float, sy: float, gx: float, gy: float) -> bool

      Check if line between points is acceptable - within edge limits and free of collisions

      :param sx: start x position
      :type sx: float
      :param sy: start y position
      :type sy: float
      :param gx: goal x position
      :type gx: float
      :param gy: goal y position
      :type gy: float

      :returns: True if node is not acceptable. False otherwise
      :rtype: result (bool)



   .. py:method:: generate_road_map(sample_x: list[float], sample_y: list[float]) -> list[list[int]]

      Road map generation

      :param sample_x: [m] x positions of sampled points
      :type sample_x: List
      :param sample_y: [m] y positions of sampled points
      :type sample_y: List

      :returns: list of edge ids
      :rtype: road_map (List)



   .. py:method:: dijkstra_planning(sx: float, sy: float, gx: float, gy: float, road_map: list[list[int]], sample_x: list[float], sample_y: list[float], show_animation: bool) -> Optional[tuple[list[float], list[float]]]
      :staticmethod:


      :param sx: start x position [m]
      :type sx: float
      :param sy: start y position [m]
      :type sy: float
      :param gx: goal x position [m]
      :type gx: float
      :param gy: goal y position [m]
      :type gy: float
      :param road_map: list of edge ids
      :type road_map: list
      :param sample_x: ??? [m]
      :type sample_x: float
      :param sample_y: ??? [m]
      :type sample_y: float

      :returns: Two lists of path coordinates ([x1, x2, ...], [y1, y2, ...]), empty list when no path was found
      :rtype: (tuple(list, list))



   .. py:method:: plot_road_map(road_map: list[list[int]], sample_x: list[float], sample_y: list[float]) -> None
      :staticmethod:



   .. py:method:: sample_points(sx: float, sy: float, gx: float, gy: float, rng: Optional[Any]) -> tuple[list[float], list[float]]

      Generate sample points

      :param sx: start x position [m]
      :type sx: float
      :param sy: start y position [m]
      :type sy: float
      :param gx: goal x position [m]
      :type gx: float
      :param gy: goal y position [m]
      :type gy: float
      :param rng: Random generator

      :returns: sample positions
      :rtype: sample (tuple (list, list))



