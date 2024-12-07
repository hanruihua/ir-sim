import irsim

env = irsim.make(save_ani=True, display=False, projection='3d')

for i in range(300):

    env.step()
    env.render(0.05)

    if env.done():
        break

env.end(3)
