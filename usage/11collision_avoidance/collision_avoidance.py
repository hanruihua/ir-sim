import irsim

env = irsim.make(save_ani=False, full=False)

for _i in range(1000):
    env.step()
    env.render(0.01)

    # if env.done():
    #     break

env.end(3)
