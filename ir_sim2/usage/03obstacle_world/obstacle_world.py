from ir_sim2.env import EnvBase

env = EnvBase('obstacle_world.yaml')

for i in range(300):

    env.step()
    env.render(0.05)
    
    if env.done():
        break

env.end()
