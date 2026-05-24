"""Social Force Model (SFM) usage example — cross-corridor intersection.

Twenty-four differential-drive "pedestrians" cross a "+" intersection of
two 8 m corridors. Each direction has three lanes on its right-hand side
of the corridor, and each lane carries two agents staggered in depth so
the central square stays busy across the run. Opposite-direction streams
don't head-on within a single arm; all conflicts happen in the
intersection, resolved by the anisotropic Moussaid-Helbing 2009 social
force. The L-shaped wall corners enter SFM as line obstacles.

See ``irsim/lib/algorithm/social_force_model.py`` for the algorithm and
``irsim/lib/behavior/behavior_methods.py`` for the registered behaviors
``("diff", "sfm")`` and ``("omni", "sfm")``.
"""

import irsim

env = irsim.make(save_ani=False, full=False)

while not env.done():
    env.step()
    env.render(0.01)

env.end()
