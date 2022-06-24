from ir_sim2.env import EnvBase
import time 
import numpy as np

env = EnvBase(world_name = 'collision_world.yaml', plot=True)

for i in range(3000):

    if i % 10==0: vel = env.cal_des_vel()
    env.step(vel)
    env.render(0.001)
    
    if env.done():
        env.render_once()
        break

env.show()
