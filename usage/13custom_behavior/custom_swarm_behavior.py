"""
Custom swarm group behavior example.

This demonstrates using a function-based custom group behavior
(@register_group_behavior) for simple stateless swarm coordination.

The swarm behavior makes robots:
1. Move toward the group centroid (cohesion)
2. Move toward their individual goals

The behavior is defined in custom_group_methods.py.
"""

import irsim

env = irsim.make()
env.load_behavior("custom_group_methods")

for _i in range(500):
    env.step()
    env.render()

    if env.done():
        break

env.end()
