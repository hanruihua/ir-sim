
import numpy as np
import time
import sys
from ir_sim2 import env
from ir_sim2.env import env_global
import matplotlib.pyplot as plt 
from ir_sim2.util.util import WrapToPi, random_points, random_value
from math import pi
import random

x = np.array([[1], [2]])

# x[0, 0] = x[1, 0]
# x[1, 0] = x[0, 0]

x[(0, 1), 0] = x[(1, 0), 0]

print(x)
