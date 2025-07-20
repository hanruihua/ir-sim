irsim.lib.path_planners.rrt_star
================================

.. py:module:: irsim.lib.path_planners.rrt_star

.. autoapi-nested-parse::

   Path planning Sample Code with RRT*

   author: Atsushi Sakai(@Atsushi_twi)

   adapted by: Reinis Cimurs



Classes
-------

.. autoapisummary::

   irsim.lib.path_planners.rrt_star.RRTStar


Module Contents
---------------

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



