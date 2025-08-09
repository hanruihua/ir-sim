import irsim

env = irsim.make("robot_world_3d.yaml", projection="3d")

for _i in range(100):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
