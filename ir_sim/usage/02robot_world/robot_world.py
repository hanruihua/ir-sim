from ir_sim.env import EnvBase

env = EnvBase('robot_world.yaml')
# env = EnvBase('robot_omni_world.yaml')
# env = EnvBase('car_world.yaml')

for i in range(1000):

    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
    
