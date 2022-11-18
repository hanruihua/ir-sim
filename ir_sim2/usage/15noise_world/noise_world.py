from ir_sim2.env import EnvBase

env = EnvBase('noise_world.yaml', control_mode='keyboard')

for i in range(3000):

    env.step()
    # re = env.get_robot_estimation()
    # env.draw_uncertainty(re['mean'], re['std'])
    env.render(show_traj=True, show_uncertainty=True)
    env.reset('any')  # 'all'; 'any'
    
env.end(show_traj=True)
