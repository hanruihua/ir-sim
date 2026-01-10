import irsim

env = irsim.make(save_ani=True, display=True)

for _i in range(1000):
    env.step()
    env.render()

    if env.done():
        break

env.end()
