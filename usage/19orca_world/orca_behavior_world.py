import irsim

env = irsim.make(save_ani=False, display=True)

while True:
    env.step()
    env.render()

    if env.done():
        break

env.end()
