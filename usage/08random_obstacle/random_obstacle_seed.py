import irsim

# Option 1: Set seed directly in make()
env = irsim.make("random_obstacle.yaml", save_ani=False, full=False, seed=2)

# Option 2: Set seed after creation with reload=True to regenerate obstacles
# env = irsim.make("random_obstacle.yaml", save_ani=False, full=False)
# env.set_random_seed(2, reload=True)  # Regenerates random obstacles with new seed

# Option 3: Only set seed without regenerating (for future random operations)
# env.set_random_seed(2)  # Default: reload=False, no regeneration

for _i in range(300):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end(3, rm_fig_path=False)
