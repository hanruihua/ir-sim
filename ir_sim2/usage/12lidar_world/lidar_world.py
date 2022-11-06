from ir_sim2.env import EnvBase, env_global
import numpy as np
import matplotlib.pyplot as plt
import time

env = EnvBase(world_name = 'lidar_world.yaml', control_mode='keyboard')
# env = EnvBase(world_name = 'lidar_world.yaml')
# env = EnvBase(world_name = 'lidar_world_car.yaml', control_mode='keyboard')

for i in range(3000):

    env.step(env.key_vel, vel_id=env.key_id)

    env.render(pause_time=0.001, show_traj=False, show_text=True)

    if env.done():
        break

env.end(show_text=True)
