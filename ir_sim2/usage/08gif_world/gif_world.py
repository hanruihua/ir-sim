from ir_sim2.env import EnvBase
import time 
import numpy as np
import matplotlib.pyplot as plt


env = EnvBase(world_name = 'gif_world.yaml', plot=True, save_ani=True)

start_time = time.time()
for i in range(300):

    vel = env.cal_des_vel()
    env.step(vel)
    env.render(show_text=True, bbox_inches='tight', pad_inches=0.2)

    # if env.done('any'): 
    #     # env.reset()
    #     # break
    
    env.reset(env.done_list(), 'single')  # 'all'; 'any'; 'single'

env.end(ani_name='gif_world', show_text=True)
