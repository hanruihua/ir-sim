
import numpy as np
import time
import sys
from ir_sim2 import env
from ir_sim2.env import env_global

# a = np.array([ [-1], [10] ])
# a_clip = np.array([ [-0.5], [0.5] ])
# a_clip2 = np.array([ [0.5], [1] ])

# a = np.clip(a, a_clip, a_clip2)
# b = 2.5
# a = np.array([[2], [5.0]])

# if a[1, 0] > b or a[1, 0] < -b:          
#     a[1, 0] = np.clip(a[1, 0], -b, b)
a = np.array([[2], [5]])
b = a.astype(float)

# vel[1, 0] = np.clip(vel[1, 0], -self.psi_limit, self.psi_limit)
# print(np.clip(a[1, 0], -0.5, 0.5))
# np.clip(a[1, 0], -0.5, 0.5)
            # vel[1, 0] = np.clip(vel[1, 0], -self.psi_limit, self.psi_limit)
# a[1, 0] = c
print(a, b)
# print(a[1, 0])
# print(c)

# print(b)
