"""Two-robot head-on pass under the Social Force Model.

A minimal stress test showing the Moussaid-Helbing anisotropic swerve:
the tiny lateral offset (0.1 m) on robot B breaks symmetry and both
agents bend their paths to pass each other.
"""

import irsim

env = irsim.make("sfm_head_on.yaml", save_ani=False, full=False)

for _i in range(300):
    env.step()
    env.render(0.01)

    if env.done():
        break

env.end(3)
