import irsim

env = irsim.make('render.yaml', save_ani=True, display=True)

for i in range(300):

    env.step()
    env.render(0.05)
    
    # env.save_figure()
    if env.done():
        break

env.end(3)
