from ir_sim2.env import EnvBase

env = EnvBase('custom_robot.yaml', control_mode='keyboard')

for i in range(3000):

    env.step()
    env.render(show_text=True)
    env.reset('any')  # 'all'; 'any'

env.end(show_text=True)
