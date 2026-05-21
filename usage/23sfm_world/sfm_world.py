"""Social Force Model (SFM) usage example — wandering pedestrians.

Ten differential-drive "pedestrians" wander inside a 12x12 m world. Each
agent uses the anisotropic Moussaid-Helbing 2009 social force (ported
from ``pedsim_ros`` / libpedsim) for inter-agent avoidance, and ``wander:
true`` in the YAML so a new random goal is sampled every time an agent
arrives. The scene therefore runs indefinitely.

See ``irsim/lib/algorithm/social_force_model.py`` for the algorithm and
``irsim/lib/behavior/behavior_methods.py`` for the registered behaviors
``("diff", "sfm")`` and ``("omni", "sfm")``.
"""

import irsim

env = irsim.make(save_ani=False, full=False)

# Wander mode -> never terminates; just run until the window is closed.
while True:
    env.step()
    env.render(0.01)

env.end()
