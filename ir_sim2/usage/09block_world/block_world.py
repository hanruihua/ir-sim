from ir_sim2.env import EnvBase
import time 
import numpy as np
import matplotlib.pyplot as plt

env = EnvBase(world_name = 'block_world.yaml', control_mode='keyboard')

start_time = time.time()
for i in range(3000):

    vel = env.cal_des_vel()
    env.step(vel)
    env.render(show_text=True, bbox_inches='tight', pad_inches=0)

    if env.done('any'): 
        # env.reset()
        break
    
    # env.reset(env.done_list(), 'any')  # 'all'; 'any'

env.end(show_text=True)
