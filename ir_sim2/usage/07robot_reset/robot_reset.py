from ir_sim2.env import EnvBase
import time 
import numpy as np
import matplotlib.pyplot as plt

env = EnvBase(world_name = 'robot_reset.yaml', plot=True, save_ani=False)
# env = EnvBase(world_name = 'robot_reset_car.yaml', plot=True, save_ani=False)

start_time = time.time()
for i in range(300):

    vel = env.cal_des_vel()
    env.step(vel)
    env.render(0.00001, show_text=True, show_traj=False)

    env.reset(env.done_list(), 'single')  # 'all'; 'any'; 'single'

print('time cost', time.time() - start_time)   # car 26s diff 3s
env.end(ani_name='robot_reset')
