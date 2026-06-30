import irsim

env = irsim.make("orca_diff_world.yaml", save_ani=False, display=True)

for _i in range(1000):
    env.step()
    env.render()

    if env.done():
        break

env.end()
