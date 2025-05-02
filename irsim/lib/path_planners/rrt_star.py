"""

Path planning Sample Code with RRT*

author: Atsushi Sakai(@Atsushi_twi)

adapted by: Reinis Cimurs

"""

import math

from irsim.lib.path_planners.rrt import RRT


class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        """
        RRT Node
        """

        def __init__(self, x, y):
            """
            Initialize Node

            Args:
                x (float): x position of the node
                y (float): y position of the node
            """
            super().__init__(x, y)
            self.cost = 0.0

    def __init__(
        self,
        env_map,
        robot_radius,
        expand_dis=1.5,
        path_resolution=0.25,
        goal_sample_rate=5,
        max_iter=500,
        connect_circle_dist=0.5,
        search_until_max_iter=False,
    ):
        """
        Initialize RRT planner

        Args:
            env_map (Env): environment map where the planning will take place
            robot_radius (float): robot body modeled as circle with given radius
            expand_dis (float): expansion distance
            path_resolution (float): resolution of the path
            goal_sample_rate (int): goal sample rate
            max_iter (int): max iteration count
            connect_circle_dist (float): maximum distance for nearby node connection ir rewiring
            search_until_max_iter (bool): if to search for path until the max_iter count
        """
        super().__init__(
            env_map,
            robot_radius,
            expand_dis,
            path_resolution,
            goal_sample_rate,
            max_iter,
        )
        self.connect_circle_dist = connect_circle_dist
        self.search_until_max_iter = search_until_max_iter
        self.node_list = []

    def planning(self, start_pose, goal_pose, show_animation=True):
        """
        rrt star path planning

        Args:
            start_pose (np.array): start pose [x,y]
            goal_pose (np.array): goal pose [x,y]
            show_animation (bool): If true, shows the animation of planning process

        Returns:
            (np.array): xy position array of the final path
        """
        self.start = self.Node(start_pose[0].item(), start_pose[1].item())
        self.end = self.Node(goal_pose[0].item(), goal_pose[1].item())

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd, self.expand_dis)
            near_node = self.node_list[nearest_ind]
            new_node.cost = near_node.cost + math.hypot(
                new_node.x - near_node.x, new_node.y - near_node.y
            )

            if self.check_collision(new_node, self.robot_radius):
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(new_node, near_inds)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                    if show_animation:
                        self.draw_graph(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)
                    if show_animation:
                        self.draw_graph(new_node)

            if (not self.search_until_max_iter) and new_node:  # if reaches goal
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        return None

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.

        Args:
            new_node (Node): randomly generated node with a path from its neared point
            near_inds (List): indices of the nodes what are near to new_node

        Returns:
            (Node): a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.robot_radius):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        """
        Search for the best goal node in the current RRT* tree.

        Returns:
            (int or None): Index of the best goal node in the node list if found, otherwise None.
        """
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        goal_inds = [
            dist_to_goal_list.index(i)
            for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.end)
            if self.check_collision(t_node, self.robot_radius):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        safe_goal_costs = [
            self.node_list[i].cost
            + self.calc_dist_to_goal(self.node_list[i].x, self.node_list[i].y)
            for i in safe_goal_inds
        ]

        min_cost = min(safe_goal_costs)
        for i, cost in zip(safe_goal_inds, safe_goal_costs):
            if cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Args:
                new_node (Node): new randomly generated node, without collisions between it and its nearest node

            Returns:
                (List): List with the indices of the nodes inside the ball radius
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, "expand_dis"):
            r = min(r, self.expand_dis)
        dist_list = [
            (node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2
            for node in self.node_list
        ]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
        Rewire the tree to improve path cost by checking nearby nodes.

        For each node in near_inds, this method checks whether it is more
        cost-effective to reach it via new_node. If so, the parent of that
        node is updated to new_node, and the cost is propagated to its children.

        Args:
            new_node (Node): The newly added node that may provide a better path.
            near_inds (list of int): Indices of nodes within the connection radius
                                     that are candidates for rewiring.

        Returns:
            None
        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node, self.robot_radius)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                for node in self.node_list:
                    if node.parent == self.node_list[i]:
                        node.parent = edge_node
                self.node_list[i] = edge_node
                self.propagate_cost_to_leaves(self.node_list[i])

    def calc_new_cost(self, from_node, to_node):
        """
        Calculate the cost of reaching to_node from from_node.

        Args:
            from_node (Node): The starting node.
            to_node (Node): The target node.

        Returns:
            (float): The total cost to reach to_node via from_node.
        """
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):
        """
        Recursively update the cost of all descendant nodes.

        This function updates the cost of all child nodes of the given
        parent_node by recalculating their cost, and then propagates
        the update down to their children.

        Args:
            parent_node (Node): The node from which to start cost propagation.

        Returns:
            None
        """

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)
