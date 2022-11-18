from ir_sim2.env import EnvBase

env = EnvBase(world_name = 'lidar_world_car.yaml', control_mode='keyboard')
# env = EnvBase('lidar_world.yaml', control_mode='keyboard')
# env = EnvBase('lidar_world_omni.yaml', control_mode='keyboard')

for i in range(3000):

    env.step()
    env.render(show_text=True, show_goal=False)
    env.reset('single')

env.end(show_text=True)
