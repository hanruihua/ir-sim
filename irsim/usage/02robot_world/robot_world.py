import irsim

env = irsim.make('robot_world.yaml')
# env = irsim.make("robot_omni_world.yaml")
# env = irsim.make('car_world.yaml')
# env = irsim.make("robot_polygon_world.yaml")

for i in range(1000):

    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
