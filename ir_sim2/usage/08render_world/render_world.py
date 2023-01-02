from ir_sim2.env import EnvBase

env = EnvBase('render_world.yaml', save_fig=False, display=True)

for i in range(300):

    vel = env.cal_des_vel()
    env.step(vel)

    env.render(0.05, robot_color='g', show_traj=True, show_text=True, goal_color='r', show_goal=True, traj_type='-g', show_sensor=True)
    # env.render(0.05, show_trail=True, edgecolor='y', trail_type='rectangle')  for ackermann
    # figure_args: arguments when saving the figures for animation, see https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for detail

    env.reset('single')  # 'all'; 'any'; 'single'

env.end(robot_color='g', show_traj=True, show_text=True, goal_color='r', show_goal=True, traj_type='-g')
