"""
Example: Perlin noise grid map via YAML obstacle_map dict spec.

The YAML uses ``obstacle_map: { name: perlin, ... }`` so the grid is
generated at load time — no external PNG or Python pre-generation needed.
"""

import irsim

env = irsim.make("grid_map_perlin.yaml", save_ani=False, full=False)

for _ in range(500):
    env.step()
    env.render()

    if env.done():
        break

env.end(5)
