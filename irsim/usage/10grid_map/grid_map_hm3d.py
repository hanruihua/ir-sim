import irsim
import time

env = irsim.make(save_ani=False, full=False)

for i in range(1000):

    env.step()
    env.render()

    if env.done():
        break

env.end(5)
