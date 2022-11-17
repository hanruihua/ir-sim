from ir_sim2.world import RobotBase
import numpy as np

class RobotCustom(RobotBase):

    robot_type = 'custom'  
    appearance = 'circle'  # ['circle', 'rectangle']
    state_dim = (2, 1) # the state dimension, x, y, theta(heading direction)
    vel_dim = (2, 1)  # the velocity dimension, linear and angular velocity
    goal_dim = (2, 1) # the goal dimension, x, y, theta
    position_dim=(2,1) # the position dimension, x, y 
    
    def __init__(self, id, state, vel = np.zeros((2, 1)), goal=np.zeros((2, 1)), step_time=0.1, **kwargs):
        
        self.radius = kwargs.get('radius', 0.2)
        self.shape = kwargs.get('shape', [4.6, 1.6, 3, 1.6])

        super(RobotCustom, self).__init__(id, state, vel, goal, step_time, **kwargs)
        
    def dynamics(self, state, vel, **kwargs):
        # The custom robot dynamics
        # vel: (2*1) matrix
        # time:  self.step_time
    
        next_state = state + vel * self.step_time

        return next_state


    # def plot_robot(self, ax, **kwargs):
    #     default: plot the robot as a circle, you can custom plot function by yourself     
