from ir_sim2.env import EnvBase, env_global
import numpy as np
import matplotlib.pyplot as plt
import time

env = EnvBase(world_name = 'lidar_world.yaml', control_mode='keyboard')
# env = EnvBase(world_name = 'lidar_world.yaml')
# env = EnvBase(world_name = 'teleop_world_car.yaml', control_mode='keyboard')

for i in range(3000):

    start_time = time.time()
    env.step(env.key_vel, vel_id=env.key_id)
    # vel = env.cal_des_vel()
    # env.step(vel)
    end_time = time.time() - start_time
    print('step time', end_time)
    env.render(pause_time=0.000001, show_traj=False, show_text=False)

    if env.done(): 
        print('done')
        break
    
    env.reset(env.done_list(), 'any')  # 'all'; 'any'

env.end(show_text=True)
