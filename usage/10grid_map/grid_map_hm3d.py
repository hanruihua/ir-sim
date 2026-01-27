import irsim
import time
env = irsim.make(save_ani=False, full=False)

for _i in range(1000):
    start = time.perf_counter()
    env.step()
    print(f"Step {_i}: {(time.perf_counter() - start)*1000:.2f} ms")
    env.render()

    if env.done():
        break

env.end(5)