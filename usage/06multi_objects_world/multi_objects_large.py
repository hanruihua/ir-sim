import time

import irsim

env = irsim.make(save_ani=False, full=False)

for _i in range(100):
    start_time = time.time()
    env.step()
    env.render(0.01)
    end_time = time.time()
    print(f"Time taken: {end_time - start_time} seconds")

    if env.done():
        break

env.end(5)
