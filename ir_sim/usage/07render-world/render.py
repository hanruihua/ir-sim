from ir_sim.env import EnvBase

env = EnvBase('render.yaml', save_ani=True, full=False)

for i in range(300):

    env.step()
    env.render(0.05)
    
    if env.done():
        break

env.end(3)
