from ir_sim2.env import EnvBase
import time 
import numpy as np
import matplotlib.pyplot as plt

env = EnvBase(world_name = 'teleop_world.yaml', control_mode='keyboard')

start_time = time.time()
for i in range(3000):

    env.step(env.key_vel, vel_id=env.key_id)
    print(env.key_id)
    env.render()

    if env.done('any'): 
        # env.reset()
        break
    
    # env.reset(env.done_list(), 'any')  # 'all'; 'any'
env.end(show_text=True)
