import irsim

# env = irsim.make('car_world.yaml', projection='3d')
env = irsim.make('robot_world_3d.yaml', projection='3d')

for i in range(1000):

    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
