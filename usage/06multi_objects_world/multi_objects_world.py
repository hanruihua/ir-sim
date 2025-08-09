import irsim

env = irsim.make("multi_objects_world.yaml")

for _i in range(300):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end(3)
