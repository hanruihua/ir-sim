"""
Example: JPS (Jump Point Search) path planning on the world occupancy map.

Loads a world from YAML, builds the map, plans a path from robot state to goal
using JPS, then draws the trajectory and shows the result.
"""

import irsim
from irsim.lib.path_planners import JPSPlanner

env = irsim.make("path_planning.yaml", save_ani=False, full=False)

env_map = env.get_map(resolution=0.2)
planner = JPSPlanner(env_map)

robot_state = env.get_robot_state()
robot_info = env.get_robot_info()
goal_xy = robot_info.goal[:2, 0].tolist()
trajectory = planner.planning(robot_state, goal_xy, show_animation=True)

if trajectory is not None:
    env.draw_trajectory(trajectory, traj_type="r-")

env.end(5)
