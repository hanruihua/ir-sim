import irsim

env = irsim.make("keyboard_control_mpl.yaml", save_ani=False, full=False)
# env = irsim.make("keyboard_control_pynput.yaml", save_ani=False, full=False)

for _i in range(1000):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end(3)
