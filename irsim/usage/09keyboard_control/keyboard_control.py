import irsim

env = irsim.make('keyboard_control.yaml', save_ani=False, full=False)

for i in range(300):

    env.step()
    env.render(0.05, show_goal=False, show_trajectory=True)
    
    if env.done():
        break

env.end(3)
