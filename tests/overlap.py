import time

import irsim
import numpy as np
import random


class SIM_ENV:
    def __init__(self, world_file="robot_world.yaml"):
        self.env = irsim.make(world_file)
        robot_info = self.env.get_robot_info(0)
        self.robot_goal = robot_info.goal

    def step(self, lin_velocity=0.0, ang_velocity=0.0):
        self.env.step(action_id=0, action=np.array([[lin_velocity], [ang_velocity]]))
        self.env.render()


    def reset(self):
        self.env.robot.set_state(
            state=np.array([[random.uniform(1, 9)], [random.uniform(1, 9)], [0], [0]]),
            init=True,
        )
        self.env.reset()
        self.env.robot.set_goal(
            np.array([[random.uniform(1, 9)], [random.uniform(1, 9)], [0]])
        )
        self.env.random_obstacle_position(
            range_low=[0, 0, -3.14],
            range_high=[10, 10, 3.14],
            ids=[i + 1 for i in range(5)],
            non_overlapping=True
        )


        self.robot_goal = self.env.robot.goal


sim = SIM_ENV()
for i in range(100):
    sim.reset()
    for j in range(3):
        sim.step()
        time.sleep(1)


