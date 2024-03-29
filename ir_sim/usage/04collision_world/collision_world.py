from ir_sim.env import EnvBase

env = EnvBase('collision_world.yaml')

for i in range(300):

    env.step()
    env.render(0.05)
    
    if env.done():
        break

env.end()
