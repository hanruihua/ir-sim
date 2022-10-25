
import numpy as np
import time
import sys
from ir_sim2 import env
from ir_sim2.env import env_global

a = np.array([ [-1], [10] ])
a_clip = np.array([ [-0.5], [0.5] ])
a_clip2 = np.array([ [0.5], [1] ])

a = np.clip(a, a_clip, a_clip2)

# vel[1, 0] = np.clip(vel[1, 0], -self.psi_limit, self.psi_limit)
# print(np.clip(a[1, 0], -0.5, 0.5))
# np.clip(a[1, 0], -0.5, 0.5)
            # vel[1, 0] = np.clip(vel[1, 0], -self.psi_limit, self.psi_limit)
# a[1, 0] = c
print(a)
# print(a[1, 0])
# print(c)

# print(b)
