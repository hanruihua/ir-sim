import irsim
import time 
env = irsim.make(save_ani=False, full=False)

for i in range(1000):

    start_time = time.time()
  
    env.step()
    end_time = time.time()
    print(f"Time taken: {end_time - start_time} seconds")
    env.render(0.01)
    
    if env.done():
        break

env.end(5)
