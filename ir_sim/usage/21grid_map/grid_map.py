from ir_sim.env import EnvBase

env = EnvBase('grid_map.yaml', control_mode='keyboard', init_args={'no_axis': False})

print(env.get_grid_map())

for i in range(300):
    env.step()
    env.render()
    
env.end()


