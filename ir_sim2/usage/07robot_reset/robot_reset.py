from ir_sim2.env import EnvBase
import time 
import numpy as np
import matplotlib.pyplot as plt

env = EnvBase(world_name = 'robot_reset.yaml', plot=True, save_ani=False)

start_time = time.time()
for i in range(3000):

    if i % 10==0: vel = env.cal_des_vel()
    env.step(vel)
    env.render(0.00001, show_text=True, show_traj=False)

    if env.done('all'): 
        env.reset()
        print(i)
        if i >= 200:
            break
        # break
        # env.reset(env.done_list(), 'any')  # 'all'; 'any'

print('time cost', time.time() - start_time)   # car 26s diff 3s
# env.save_animate('robot reset')
env.end(ani_name='robot_reset')
# env.show(show_text=True)
