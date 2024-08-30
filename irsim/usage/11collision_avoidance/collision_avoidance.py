import irsim

env = irsim.make('collision_avoidance.yaml', save_ani=False, full=False)

for i in range(1000):

    env.step()
    env.render(0.01)
    
    if env.done():
        break

env.end(5)
