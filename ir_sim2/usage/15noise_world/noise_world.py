from ir_sim2.env import EnvBase

env = EnvBase('noise_world.yaml', control_mode='keyboard')

for i in range(3000):

    env.step()
    env.render(show_traj=True)
    env.reset('any')  # 'all'; 'any'

env.end(show_traj=True)
