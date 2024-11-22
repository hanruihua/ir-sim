import irsim

env = irsim.make('render.yaml', save_ani=False, display=False)

for i in range(300):

    env.step()
    env.render(0.05)

    if env.done():
        break

env.save_figure('render2.pdf')
env.end(3)
