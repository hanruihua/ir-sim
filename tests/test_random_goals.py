import time

import numpy as np
import shapely
from irsim.lib.handler.geometry_handler import GeometryFactory
from irsim.util.util import time_it
import irsim
import matplotlib.pyplot as plt

def check_goal(goal, obstacle_list, goal_check_radius):
    shape = {"name": "circle", "radius": goal_check_radius}
    gf = GeometryFactory.create_geometry(**shape)
    geometry = gf.step(np.c_[goal])
    covered_goal = any(
        [
            shapely.intersects(geometry, obj._geometry)
            for obj in obstacle_list
        ]
    )

    return covered_goal


@time_it("test_all_objects")
def test_random_goals():
    goal_check_radius = 0.4
    env = irsim.make('test_collision_world.yaml', save_ani=False, display=False)
    env.robot.set_goal([5, 10, 0], init=True)

    env_objects = env.obstacle_list

    goal = env.robot._goal[0]
    covered_goal = check_goal(goal, env_objects, goal_check_radius)
    assert covered_goal == False

    env.robot.set_goal([5, 5, 0], init=True)
    goal = env.robot._goal[0]
    covered_goal = check_goal(goal, env_objects, goal_check_radius)
    assert covered_goal == True

    for _ in range(100):
        env.robot.set_random_goal(env_objects, goal_check_radius=goal_check_radius)
        goal = env.robot._goal[0]
        covered_goal = check_goal(goal, env_objects, goal_check_radius)
        assert covered_goal == False

    env.robot.set_goal([[5, 10, 0],[5, 9, 0],[5, 8, 0]], init=True)

    for _ in range(100):
        env.robot.set_random_goal(env_objects, goal_check_radius=goal_check_radius, range_limits=[[3, 3, -3.141592653589793], [7, 7, 3.141592653589793]])
        goals = env.robot._goal
        for goal in goals:
            covered_goal = check_goal(goal, env_objects, goal_check_radius)
            assert covered_goal == False
        assert all([3 < point[0] < 7 for point in goals])
        assert all([3 < point[1] < 7 for point in goals])
    goal = env.robot._goal[0]
    assert len(goal) == 3


if __name__ == "__main__":
    test_random_goals()
