from ir_sim2.env import EnvBase
import time 
import numpy as np
import matplotlib.pyplot as plt

env = EnvBase(world_name = 'robot_reset.yaml', plot=True, log_level='info')

for i in range(3000):

    if i % 10==0: vel = env.cal_des_vel()
    env.step(vel)
    env.render(0.00001, show_text=True)

    if env.done('any'):
        env.reset()

    # env.reset(env.done_list(), 'any') 

env.show(show_text=True)
