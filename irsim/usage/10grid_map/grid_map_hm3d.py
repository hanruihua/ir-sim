import irsim
import time
env = irsim.make(save_ani=False, full=False)

for i in range(1000):

    env.step()
    start_time = time.time()
    
    env.render()
    end_time = time.time()
    print(f"Time taken: {end_time - start_time} seconds")
    
    if env.done():
        break

env.end(5)