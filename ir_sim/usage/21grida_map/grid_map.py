from ir_sim.env import EnvBase

env = EnvBase('grid_map.yaml', control_mode='keyboard')
 
for i in range(100):
    env.step()
    env.render()

env.end()


