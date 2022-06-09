import numpy as np

a = np.c_[[100, 100]]
amin = np.c_[[4, 10]]
amax = np.c_[[5, 20]]
# amin =

# b = np.c_[a]
print(a.shape)

if (a < amin).any(): print('min')
if (a > amax).any(): print('max')

# min = np.array([[3], [3]])

# max = np.array([[5], [7]])

c = np.clip(a, amin, amax)

print(c)