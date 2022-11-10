from ir_sim2.env import EnvBase, env_global
import numpy as np
import matplotlib.pyplot as plt
import time

env = EnvBase(world_name = 'lidar_world_car.yaml', control_mode='keyboard')
# env = EnvBase(world_name = 'all_world.yaml', control_mode='keyboard')

for i in range(3000):

    env.step()
    env.render(0.001, show_traj=False, show_text=True, show_goal=False)

    # if env.done():
    #     break
    
    if env.done('any'):
        env.reset('single')

env.end(show_text=True)
