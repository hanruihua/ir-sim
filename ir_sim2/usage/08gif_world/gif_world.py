from ir_sim2.env import EnvBase
import time 
import numpy as np
import matplotlib.pyplot as plt

# env = EnvBase(world_name = 'gif_world_car.yaml', plot=False, log_level='warning')
env = EnvBase(world_name = 'gif_world.yaml', plot=True, log_level='warning', save_ani=True)

start_time = time.time()
for i in range(3000):

    if i % 10==0: vel = env.cal_des_vel()
    env.step(vel)
    env.render(show_text=True, bbox_inches='tight', pad_inches=0)

    if env.done('any'): 
        # env.reset()
        break
    
    # env.reset(env.done_list(), 'any')  # 'all'; 'any'

env.save_animate('ani_test')
print('time cost', time.time() - start_time)
env.show(show_text=True)
