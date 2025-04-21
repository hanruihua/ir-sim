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
def test_rrt_planner(planner, resolution):
    env = irsim.make("test_collision_world.yaml", save_ani=False, full=False, display=False)
    planner = planner(env, resolution)
    robot_info = env.get_robot_info()
    robot_state = env.get_robot_state()
    rx, ry = planner.planning(
        robot_state[0].item(),
        robot_state[1].item(),
        robot_info.goal[0].item(),
        robot_info.goal[1].item(),
    )

    plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    plt.show()