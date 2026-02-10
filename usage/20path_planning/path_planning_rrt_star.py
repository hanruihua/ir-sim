"""
Example: RRT* path planning.

RRT* **optimises** path cost: it chooses the best parent in a neighbourhood
(choose-parent) and rewires nearby nodes through the new node to shorten paths.
Result is typically a shorter, smoother path than basic RRT.

Compare with path_planning_rrt.py to see the difference in path length/cost.
"""

import numpy as np

import irsim
from irsim.lib.path_planners import RRTStar

env = irsim.make("path_planning.yaml", save_ani=False, full=False)

env_map = env.get_map(resolution=0.1)
planner = RRTStar(
    env_map,
    robot=env.robot,
    expand_dis=1.5,
    max_iter=5000,
    search_until_max_iter=False,
)

robot_state = env.get_robot_state()
robot_info = env.get_robot_info()
goal_xy = robot_info.goal[:2, 0].tolist()
trajectory = planner.planning(robot_state, goal_xy, show_animation=True)

if trajectory is not None:
    env.draw_trajectory(np.array(trajectory), traj_type="r-")
    # RRT* already logs path cost in planner.planning(); print for quick comparison
    print(
        f"[RRT*] path cost = {planner.end.cost:.3f}, tree nodes = {len(planner.node_list)}"
    )
else:
    print("[RRT*] no feasible path found")

env.end(5)
