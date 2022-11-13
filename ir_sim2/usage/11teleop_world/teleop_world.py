from ir_sim2.env import EnvBase

# env = EnvBase('teleop_world.yaml', control_mode='keyboard')
env = EnvBase('teleop_omni_world.yaml', control_mode='keyboard')
# env = EnvBase('teleop_world_car.yaml', control_mode='keyboard')

for i in range(3000):

    env.step()
    env.render(show_text=True)
    env.reset('any')  # 'all'; 'any'

env.end(show_text=True)
