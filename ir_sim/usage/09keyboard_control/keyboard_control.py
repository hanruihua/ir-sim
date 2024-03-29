from ir_sim.env import EnvBase

env = EnvBase('keyboard_control.yaml', save_ani=False, rm_fig_path=False, full=False)

for i in range(1000):

    env.step()
    env.render(0.05, show_goal=False, show_trajectory=True)
    
    if env.done():
        break

env.end(10)
