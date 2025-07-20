import os
import irsim


'''
world parameters:
    time: time elapse of the simulation
    control_mode:
        - auto: robot will be controlled automatically
        - keyboard: robot will be controlled by keyboard
    
    collision_mode: 
        - stop (default): All Objects stop when collision,
        - unobstructed: No collision check
        - reactive: robot will have reaction when collision with others    
        - unobstructed_obstacles: Only allows obstacles to pass through each other without consideration of any collision. The robots will stop when they are in collision with the obstacles.

    step_time: time of the simulation step, default is 0.1
    count: count of the simulation, time = count * step_time

'''

time = 0
control_mode = "auto"
collision_mode = "stop"  

step_time = 0.1
count = 0
