import time

import numpy as np

import irsim

env = irsim.make("thousand_robot_world.yaml", seed=0, log_level="ERROR")
n = len(env.robot_list)
rng = np.random.default_rng(0)
step_time = []

for _ in range(200):
    action = np.column_stack([rng.uniform(0, 1, n), rng.uniform(-1, 1, n)]).tolist()
    start = time.perf_counter()
    env.step(action)
    step_time.append(time.perf_counter() - start)
    print(f"step {_}: {1e3 * step_time[-1]:.1f} ms")
    env.render(0.01)

# the first steps warm up the caches (and compile the numba kernel, if installed)
print(f"{n} robots with lidar: {1e3 * np.mean(step_time[20:]):.1f} ms per env.step()")
env.end()
