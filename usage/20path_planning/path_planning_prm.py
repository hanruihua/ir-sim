"""
Example: Probabilistic Road Map (PRM) path planning.

PRM samples random nodes in free space, connects each node to its k nearest
neighbours through collision-free edges, and runs Dijkstra on the resulting
graph. Useful for grid maps where you want a graph that can be reused for
multiple queries.
"""

import numpy as np

import irsim
from irsim.lib.path_planners import PRMPlanner

env = irsim.make("path_planning.yaml", save_ani=False, full=False)

env_map = env.get_map(resolution=0.1)

planner = PRMPlanner(
    env_map,
    robot_radius=env.robot.radius,
    n_sample=500,
    n_knn=10,
    max_edge_len=5.0,
)

robot_state = env.get_robot_state()
robot_info = env.get_robot_info()
goal_xy = robot_info.goal[:2, 0].tolist()
trajectory = planner.planning(robot_state, goal_xy, show_animation=True)

if trajectory is not None and trajectory.size > 0:
    env.draw_trajectory(np.asarray(trajectory), traj_type="r-")
else:
    print("[PRM] no feasible path found")

env.end(5)
