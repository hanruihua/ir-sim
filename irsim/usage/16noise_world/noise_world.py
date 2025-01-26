import irsim

env = irsim.make(save_ani=False)

for i in range(200):

    env.step()
    env.render(0.05)
    
env.end(3)
