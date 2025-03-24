import irsim
import numpy as np

env = irsim.make()

for i in range(300):

    env.step()
    env.render(0.05)

    indices = np.arange(0, i)
    env.robot.set_laser_color(indices, 'cyan')

    if env.done():
        break

env.end(3)
