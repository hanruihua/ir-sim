from ir_sim2.env import EnvBase
import time 
import numpy as np
import matplotlib.pyplot as plt

env = EnvBase(world_name = 'lidar_world.yaml')

start_time = time.time()
for i in range(3000):

    vel = env.cal_des_vel()
    env.step(vel)
    env.render()

    if env.done('any'): 
        # env.reset()
        break
    
    # env.reset(env.done_list(), 'any')  # 'all'; 'any'

env.end(show_text=True)
