import irsim
import time 
env = irsim.make(save_ani=False, full=False)

for i in range(100):

    env.step()
    env.render(0.01)
    
    if env.done():
        break

env.end(5)
