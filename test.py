import numpy as np

goal = np.random.uniform(low=[0, 0], high=[10, 10])
goal = np.expand_dims(goal, axis=1)
print(goal)