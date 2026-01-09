"""
Custom class-based individual behavior example.

This demonstrates using @register_behavior_class for stateful behaviors.
The smooth_dash behavior maintains previous velocity for smooth acceleration.

Two robots are shown:
- Blue: Uses smooth_dash (smoother motion)
- Red: Uses regular dash (comparison)

The behavior is defined in custom_behavior_methods.py.
"""

import irsim

env = irsim.make()
env.load_behavior("custom_behavior_methods")

for _i in range(500):
    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
