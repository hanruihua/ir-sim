import irsim

env = irsim.make(save_ani=True, display=False)

for _i in range(300):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
