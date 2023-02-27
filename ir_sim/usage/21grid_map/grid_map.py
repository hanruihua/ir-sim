from ir_sim.env import EnvBase

env = EnvBase('grid_map.yaml', control_mode='keyboard', init_args={'no_axis': False}, collision_mode='stop', save_ani=False)
# env = EnvBase('grid_map_car.yaml', control_mode='keyboard', init_args={'no_axis': False}, collision_mode='stop', save_ani=True)

print(env.get_grid_map())

for i in range(300):
    env.step()
    env.render(show_traj=True)

    if env.done(): break
        
env.end(ani_name='grid_map', ani_kwargs={'subrectangles': True})
