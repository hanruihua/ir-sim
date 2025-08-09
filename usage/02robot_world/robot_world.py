import time

import irsim

env = irsim.make("robot_world.yaml")
# env = irsim.make("robot_omni_world.yaml")
# env = irsim.make('car_world.yaml')
# env = irsim.make("robot_polygon_world.yaml")

for _i in range(1000):
    env.step()
    start_time = time.time()
    env.render()
    end_time = time.time()
    print(f"Time taken: {end_time - start_time} seconds")

    if env.done():
        break

env.end()
