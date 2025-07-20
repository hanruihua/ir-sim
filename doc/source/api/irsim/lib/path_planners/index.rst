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

   irsim.lib.path_planners.RRT
   irsim.lib.path_planners.RRTStar


Package Contents
----------------

.. py:class:: RRT(env_map, robot_radius, expand_dis=1.0, path_resolution=0.25, goal_sample_rate=5, max_iter=500)

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


   .. py:class:: Node(x, y)

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




   .. py:class:: AreaBounds(env_map)

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


   .. py:method:: planning(start_pose, goal_pose, show_animation=True)

      rrt path planning

      :param start_pose: start pose [x,y]
      :type start_pose: np.array
      :param goal_pose: goal pose [x,y]
      :type goal_pose: np.array
      :param show_animation: If true, shows the animation of planning process
      :type show_animation: bool

      :returns: xy position array of the final path
      :rtype: (np.array)



   .. py:method:: steer(from_node, to_node, extend_length=float('inf'))

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



   .. py:method:: generate_final_course(goal_ind)

      Generate the final path

      :param goal_ind: index of the final goal
      :type goal_ind: int

      :returns: xy position array of the final path
      :rtype: (np.array)



   .. py:method:: calc_dist_to_goal(x, y)

      Calculate distance to goal

      :param x: x coordinate of the position
      :type x: float
      :param y: y coordinate of the position
      :type y: float

      :returns: distance to the goal
      :rtype: (float)



   .. py:method:: get_random_node()

      Create random node

      :returns: new random node
      :rtype: (Node)



   .. py:method:: draw_graph(rnd=None)


   .. py:method:: plot_circle(x, y, size, color='-b')
      :staticmethod:



   .. py:method:: get_nearest_node_index(node_list, rnd_node)
      :staticmethod:



   .. py:method:: check_if_outside_play_area(node, play_area)
      :staticmethod:



   .. py:method:: check_collision(node, robot_radius)

      Check if node is acceptable - free of collisions

      :param node: node to check
      :type node: Node
      :param robot_radius: robot radius
      :type robot_radius: float

      :returns: True if there is no collision. False otherwise
      :rtype: (bool)



   .. py:method:: check_node(x, y, rr)

      Check positon for a collision

      :param x: x value of the position
      :type x: float
      :param y: y value of the position
      :type y: float
      :param rr: robot radius
      :type rr: float

      :returns: True if there is a collision. False otherwise
      :rtype: (bool)



   .. py:method:: calc_distance_and_angle(from_node, to_node)
      :staticmethod:



.. py:class:: RRTStar(env_map, robot_radius, expand_dis=1.5, path_resolution=0.25, goal_sample_rate=5, max_iter=500, connect_circle_dist=0.5, search_until_max_iter=False)

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


   .. py:class:: Node(x, y)

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



   .. py:method:: planning(start_pose, goal_pose, show_animation=True)

      rrt star path planning

      :param start_pose: start pose [x,y]
      :type start_pose: np.array
      :param goal_pose: goal pose [x,y]
      :type goal_pose: np.array
      :param show_animation: If true, shows the animation of planning process
      :type show_animation: bool

      :returns: xy position array of the final path
      :rtype: (np.array)



   .. py:method:: choose_parent(new_node, near_inds)

      Computes the cheapest point to new_node contained in the list
      near_inds and set such a node as the parent of new_node.

      :param new_node: randomly generated node with a path from its neared point
      :type new_node: Node
      :param near_inds: indices of the nodes what are near to new_node
      :type near_inds: List

      :returns: a copy of new_node
      :rtype: (Node)



   .. py:method:: search_best_goal_node()

      Search for the best goal node in the current RRT* tree.

      :returns: Index of the best goal node in the node list if found, otherwise None.
      :rtype: (int or None)



   .. py:method:: find_near_nodes(new_node)

      1) defines a ball centered on new_node
      2) Returns all nodes of the three that are inside this ball
          Args:
              new_node (Node): new randomly generated node, without collisions between it and its nearest node

          Returns:
              (List): List with the indices of the nodes inside the ball radius



   .. py:method:: rewire(new_node, near_inds)

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



   .. py:method:: calc_new_cost(from_node, to_node)

      Calculate the cost of reaching to_node from from_node.

      :param from_node: The starting node.
      :type from_node: Node
      :param to_node: The target node.
      :type to_node: Node

      :returns: The total cost to reach to_node via from_node.
      :rtype: (float)



   .. py:method:: propagate_cost_to_leaves(parent_node)

      Recursively update the cost of all descendant nodes.

      This function updates the cost of all child nodes of the given
      parent_node by recalculating their cost, and then propagates
      the update down to their children.

      :param parent_node: The node from which to start cost propagation.
      :type parent_node: Node

      :returns: None



