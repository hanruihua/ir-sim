import irsim

env = irsim.make("keyboard_control.yaml", save_ani=False, full=False)

for _i in range(300):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end(3)
