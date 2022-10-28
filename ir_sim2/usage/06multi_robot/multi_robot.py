from ir_sim2.env import EnvBase
import time 
import numpy as np
import matplotlib.pyplot as plt

# env = EnvBase(world_name = 'multi_robot_car.yaml', plot=True, log_level='info')
env = EnvBase(world_name = 'multi_robot.yaml', plot=True)

for i in range(3000):

    vel = env.cal_des_vel()
    env.step(vel)
    env.render(0.001)

    if env.done('all'):
        env.render_once(show_text=True)
        break

env.show(show_text=True)
