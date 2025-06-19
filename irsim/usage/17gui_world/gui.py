import irsim
import numpy as np

"""
Interactive Robot Simulation GUI Controls

Keyboard Controls (control_mode: 'keyboard'):
+=========+=================================+
| w       | Move forward                    |
+---------+---------------------------------+
| s       | Move backward                   |
+---------+---------------------------------+
| a       | Turn left                       |
+---------+---------------------------------+
| d       | Turn right                      |
+---------+---------------------------------+
| q       | Decrease linear velocity        |
+---------+---------------------------------+
| e       | Increase linear velocity        |
+---------+---------------------------------+
| z       | Decrease angular velocity       |
+---------+---------------------------------+
| c       | Increase angular velocity       |
+---------+---------------------------------+
| alt+num | Switch to robot ID 'num'        |
+---------+---------------------------------+
| r       | Reset environment               |
+---------+---------------------------------+
| space   | Pause/Resume the environment    |
+---------+---------------------------------+

Mouse Controls:
- Mouse Movement: Track mouse position and update display coordinates
- Middle Click: Reset zoom to default view
- Scroll Up: Zoom in (centered on mouse position)
- Scroll Down: Zoom out (centered on mouse position)

Mouse Position Attributes:
- env.mouse_left_pos: Position of last left click
- env.mouse_right_pos: Position of last right click  
- env.mouse_pos: Current mouse position
"""

env = irsim.make(save_ani=False, full=False)

for i in range(10000):
    env.step()
    env.render(0.05, show_goal=False)
    
    if env.mouse_left_pos is not None:
        env.robot.set_goal(env.mouse_left_pos)
    
    if env.done():
        break

env.end(3)
