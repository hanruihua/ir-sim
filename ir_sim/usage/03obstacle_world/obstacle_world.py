from ir_sim.env import EnvBase

env = EnvBase('obstacle_world.yaml')

for i in range(3000):

    env.step()
    env.render(0.05)
    
    if env.done():
        break


env.end()
