import numpy as np
import time
from multiprocessing import Pool

s1 = time.time()
for i in range(3000):
    a = np.array([[3], [1]]) + np.array([[5], [2]])
print(time.time()-s1)

s2 = time.time()
b = np.array([[3], [1]])  
c = np.array([[5], [2]])

for i in range(3000):
    b[0,0] = 0
    c[1, 0] = 10
    a = b + c

print(time.time()-s2)

s3 = time.time()
b = np.zeros((3, 1))
c = np.zeros((3, 1))
for i in range(3000):
    
    b[0, 0] = 3
    b[1, 0] = 1

    
    c[0, 0] = 3
    c[1, 0] = 1

    a = b + c

print(time.time()-s3)

