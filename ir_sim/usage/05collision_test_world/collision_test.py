from ir_sim.env import EnvBase
import numpy as np
import matplotlib.pyplot as plt

env = EnvBase('collision_test.yaml')

def on_move(event):
    if event.inaxes:
        x = event.xdata
        y = event.ydata
        point = np.array( [ [x], [y] ] )
        for num, robot in enumerate(env.robot_list):
            if robot.collision_check_point(point):
                print('collision with robot id', num)

        for num, env_obs in enumerate(env.env_obstacle_list):
            if env_obs.collision_check_point(point):
                print('collision with obstacles')

binding_id = plt.connect('motion_notify_event', on_move)
env.end()

