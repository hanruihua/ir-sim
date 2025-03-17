import irsim

env = irsim.make('grid_map_hm3d.yaml', save_ani=True, full=False)

for i in range(1000):

    env.step()
    env.render()
    
    if env.done():
        break

env.end(5)