"""
Custom group behavior example.

This demonstrates using a custom group behavior that coordinates
multiple robots to maintain formation while moving toward goals.

The behavior is defined in custom_group_methods.py and registered
using @register_group_behavior_class decorator.
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
