import irsim
import numpy as np
'''
Keyboard control:

+=========+=================================+
| w       | forward                         |
+---------+---------------------------------+
| s       | back forward                    |
+---------+---------------------------------+
| a       | turn left                       |
+---------+---------------------------------+
| d       | turn right                      |
+---------+---------------------------------+
| q       | decrease linear velocity        |
+---------+---------------------------------+
| e       | increase linear velocity        |
+---------+---------------------------------+
| z       | decrease angular velocity       |
+---------+---------------------------------+
| c       | increase angular velocity       |
+---------+---------------------------------+
| alt+num | change current control robot id |
+---------+---------------------------------+
| r       | reset the environment           |
+---------+---------------------------------+

Mouse control:

'''


env = irsim.make(save_ani=False, full=False)

for i in range(10000):

    env.step()
    env.render(0.05, show_goal=False, show_trajectory=True)
    
    if env.mouse_left_pos is not None:
        mouse_pos = np.pad(env.mouse_left_pos, (0, 1), 'constant', constant_values=0)
        env.robot.append_goal(mouse_pos)
        # env.robot.set_goal(mouse_pos)

    if env.done():
        break

env.end(3)
