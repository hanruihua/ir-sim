import irsim

env = irsim.make("random_obstacle.yaml", save_ani=False, full=False, seed=2)

"""
or
env = irsim.make("random_obstacle.yaml", save_ani=False, full=False)
env.set_random_seed(2)
env.reload()
"""

for _i in range(300):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end(3, rm_fig_path=False)
