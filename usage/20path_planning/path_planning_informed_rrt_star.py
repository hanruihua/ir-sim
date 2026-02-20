"""
Example: Informed RRT* path planning on the world occupancy map.

Informed RRT* first finds a feasible path using standard RRT* exploration,
then constrains all subsequent sampling to an ellipsoidal region whose foci
are the start and goal.  As the path cost improves the ellipse shrinks,
dramatically accelerating convergence towards the optimal path.

During planning you will see:
  - Grey lines:   the exploration tree
  - Blue ellipse: the informed sampling region (appears after first path)
  - Red line:     the current best path (updating as cost drops)
"""

import numpy as np

import irsim
from irsim.lib.path_planners import InformedRRTStar

env = irsim.make("path_planning.yaml", save_ani=False, full=False)

env_map = env.get_map(resolution=0.1)
planner = InformedRRTStar(
    env_map,
    robot=env.robot,
    expand_dis=1.5,
    max_iter=5000,
    connect_circle_dist=50.0,
    search_until_max_iter=True,
)

robot_state = env.get_robot_state()
robot_info = env.get_robot_info()
goal_xy = robot_info.goal[:2, 0].tolist()
trajectory = planner.planning(robot_state, goal_xy, show_animation=True)

if trajectory is not None:
    env.draw_trajectory(np.array(trajectory), traj_type="r-")

env.end(5)
