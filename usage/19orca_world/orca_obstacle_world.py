import irsim

env = irsim.make(save_ani=False, display=True)

for _i in range(300):
    env.step()
    env.render()

    if env.done():
        break

env.end()
