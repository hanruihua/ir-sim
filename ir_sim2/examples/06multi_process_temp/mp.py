from ir_sim2.env import EnvBase
import time 
import numpy as np
import matplotlib.pyplot as plt

env = EnvBase(world_name = 'mp.yaml', plot=True, logging=True)

start_time= time.time()
for i in range(3000):

    if i % 10==0: vel = env.cal_des_vel()
    env.step(vel)
    env.render(0.001)

    if env.done():
        env.render_once()
        break
        # pass
       
print( time.time()-start_time)
env.show()
