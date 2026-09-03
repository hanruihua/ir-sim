"""Group Social Force Model usage example — pedestrians walking together.

Twelve differential-drive "pedestrians" share a corridor under the
``sfm`` *group* behavior. All members are integrated from one state
snapshot in a single vectorised pass, and ``social_groups`` splits them
into two pairs and two triples that walk together thanks to the Moussaid
et al. (2010) coherence, repulsion and gaze forces. Opposing groups
overlap by one lane, so each group has to slide sideways as a unit while
two singles thread through the middle.

See ``irsim/lib/algorithm/social_force_model.py`` for the batch solver
and ``irsim/lib/behavior/group_behavior_methods.py`` for the registered
group behaviors ``("diff", "sfm")`` and ``("omni", "sfm")``.
"""

import irsim

env = irsim.make("sfm_group_world.yaml", save_ani=False, full=False)

while not env.done():
    env.step()
    env.render(0.01)

env.end(3)
