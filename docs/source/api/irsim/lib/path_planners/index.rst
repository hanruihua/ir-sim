irsim.lib.path_planners
=======================

.. py:module:: irsim.lib.path_planners

.. autoapi-nested-parse::

   Path planning algorithms for IR-SIM simulation.

   This package contains:
   - a_star: A* path planning algorithm
   - rrt: Rapidly-exploring Random Tree algorithm
   - rrt_star: RRT* optimized path planning
   - probabilistic_road_map: PRM path planning algorithm



Submodules
----------

.. toctree::
   :maxdepth: 1

   /api/irsim/lib/path_planners/a_star/index
   /api/irsim/lib/path_planners/probabilistic_road_map/index
   /api/irsim/lib/path_planners/rrt/index
   /api/irsim/lib/path_planners/rrt_star/index


Classes
-------

.. autoapisummary::

   irsim.lib.path_planners.AStarPlanner
   irsim.lib.path_planners.PRMPlanner
   irsim.lib.path_planners.RRT
   irsim.lib.path_planners.RRTStar


Package Contents
----------------

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



.. py:class:: RRTStar(env_map: irsim.world.map.Map, robot_radius: float, expand_dis: float = 1.5, path_resolution: float = 0.25, goal_sample_rate: int = 5, max_iter: int = 500, connect_circle_dist: float = 0.5, search_until_max_iter: bool = False)

   Bases: :py:obj:`irsim.lib.path_planners.rrt.RRT`


   Class for RRT Star planning

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
   :param connect_circle_dist: maximum distance for nearby node connection ir rewiring
   :type connect_circle_dist: float
   :param search_until_max_iter: if to search for path until the max_iter count
   :type search_until_max_iter: bool


   .. py:class:: Node(x: float, y: float)

      Bases: :py:obj:`irsim.lib.path_planners.rrt.RRT.Node`


      RRT Node

      Initialize Node

      :param x: x position of the node
      :type x: float
      :param y: y position of the node
      :type y: float


      .. py:attribute:: cost
         :value: 0.0




   .. py:attribute:: connect_circle_dist
      :value: 0.5



   .. py:attribute:: search_until_max_iter
      :value: False



   .. py:attribute:: node_list
      :value: []



   .. py:method:: planning(start_pose: list[float], goal_pose: list[float], show_animation: bool = True) -> Optional[tuple[list[float], list[float]]]

      rrt star path planning

      :param start_pose: start pose [x,y]
      :type start_pose: np.array
      :param goal_pose: goal pose [x,y]
      :type goal_pose: np.array
      :param show_animation: If true, shows the animation of planning process
      :type show_animation: bool

      :returns: xy position array of the final path
      :rtype: (np.array)



   .. py:method:: choose_parent(new_node: Node, near_inds: list[int]) -> Optional[Node]

      Computes the cheapest point to new_node contained in the list
      near_inds and set such a node as the parent of new_node.

      :param new_node: randomly generated node with a path from its neared point
      :type new_node: Node
      :param near_inds: indices of the nodes what are near to new_node
      :type near_inds: List

      :returns: a copy of new_node
      :rtype: (Node)



   .. py:method:: search_best_goal_node() -> Optional[int]

      Search for the best goal node in the current RRT* tree.

      :returns: Index of the best goal node in the node list if found, otherwise None.
      :rtype: (int or None)



   .. py:method:: find_near_nodes(new_node: Node) -> list[int]

      1) defines a ball centered on new_node
      2) Returns all nodes of the three that are inside this ball
          Args:
              new_node (Node): new randomly generated node, without collisions between it and its nearest node

          Returns:
              (List): List with the indices of the nodes inside the ball radius



   .. py:method:: rewire(new_node: Node, near_inds: list[int]) -> None

      Rewire the tree to improve path cost by checking nearby nodes.

      For each node in near_inds, this method checks whether it is more
      cost-effective to reach it via new_node. If so, the parent of that
      node is updated to new_node, and the cost is propagated to its children.

      :param new_node: The newly added node that may provide a better path.
      :type new_node: Node
      :param near_inds: Indices of nodes within the connection radius
                        that are candidates for rewiring.
      :type near_inds: list of int

      :returns: None



   .. py:method:: calc_new_cost(from_node: Node, to_node: Node) -> float

      Calculate the cost of reaching to_node from from_node.

      :param from_node: The starting node.
      :type from_node: Node
      :param to_node: The target node.
      :type to_node: Node

      :returns: The total cost to reach to_node via from_node.
      :rtype: (float)



   .. py:method:: propagate_cost_to_leaves(parent_node: Node) -> None

      Recursively update the cost of all descendant nodes.

      This function updates the cost of all child nodes of the given
      parent_node by recalculating their cost, and then propagates
      the update down to their children.

      :param parent_node: The node from which to start cost propagation.
      :type parent_node: Node

      :returns: None



