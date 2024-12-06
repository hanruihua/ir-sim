import irsim

# env = irsim.make('lidar_world.yaml')
env = irsim.make('lidar_world_noise.yaml', projection='3d')

for i in range(300):

    env.step()
    env.render(0.05)
    
    if env.done():
        break

env.end(3)
