"""Social Force Model (SFM) usage example — cross-corridor intersection.

Twenty-four differential-drive "pedestrians" cross a "+" intersection of
two 6.6 m corridors. Each direction has three lanes, and each lane
carries one head-on pair (W↔E blue/red along y-lanes; S↔N green/orange
along x-lanes), so the central square stays busy across the run. Each
pair shares a lane line with a sub-pixel offset to break exact mirror
symmetry. All conflicts happen in the intersection, resolved by the
anisotropic Moussaid-Helbing 2009 social force. The L-shaped wall
corners enter SFM as line obstacles.

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
