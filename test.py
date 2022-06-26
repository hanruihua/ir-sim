import numpy as np
import time
from multiprocessing import Pool
from math import sqrt

a = np.array([[2], [1]])
b = np.array([[5], [6]])

start_time = time.time()
for i in range(10000):
    dis = np.linalg.norm(a - b)
print('1', time.time()-start_time)

start_time = time.time()
for i in range(10000):
    dis = sqrt( (a[0, 0] - b[0, 0])**2 + (a[1, 0] - b[1, 0])**2 )
print('2', time.time()-start_time)


