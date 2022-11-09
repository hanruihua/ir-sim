
import numpy as np
import time
import sys
from ir_sim2 import env
from ir_sim2.env import env_global
import matplotlib.pyplot as plt 
from ir_sim2.util.util import WrapToPi, random_points, random_value
from math import pi
import random

mu, sigma = 0, 0.1 # mean and standard deviation
s = np.random.normal(mu, sigma, 1000)

print(s.size)
