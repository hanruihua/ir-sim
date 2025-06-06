import irsim

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

Button:

'''


env = irsim.make(save_ani=False, full=False)

for i in range(300):

    env.step()
    env.render(0.05, show_goal=False, show_trajectory=True)
    
    if env.mouse_pos is not None:
        print(env.mouse_pos)

    if env.done():
        break

env.end(3)
