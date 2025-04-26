import numpy as np
import pytest
from matplotlib import pyplot as plt

import irsim
from irsim.lib.path_planners.a_star import AStarPlanner
from irsim.lib.path_planners.probabilistic_road_map import PRMPlanner
from irsim.lib.path_planners.rrt import RRT


@pytest.mark.parametrize(
    "planner, resolution",
    [
        (AStarPlanner, 0.3),
        (RRT, 0.3),
        (PRMPlanner, 0.3),
    ],
)
def test_path_planners(planner, resolution):
    env = irsim.make("test_collision_world.yaml", save_ani=False, full=False, display=False)
    env_map = env.get_map()
    planner = planner(env_map, resolution)
    robot_info = env.get_robot_info()
    robot_state = env.get_robot_state()
    trajectory = planner.planning(robot_state, robot_info.goal)
    env.draw_trajectory(trajectory, traj_type="r-")
