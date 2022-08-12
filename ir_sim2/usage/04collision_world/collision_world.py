from ir_sim2.env import EnvBase
import time 
import numpy as np

env = EnvBase(world_name = 'collision_world.yaml')
# env = EnvBase(world_name = 'collision_world_car.yaml')

for i in range(3000):

    vel = env.cal_des_vel()
    env.step(vel)
    env.render(0.001, show_traj=True)
    
    if env.done():
        env.render_once()
        break

env.show(show_traj=True)
