import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from .robot_base import RobotBase
from math import sin, cos, pi, atan2, inf
from ir_sim2.env import env_global


class RobotOmni(RobotBase):

    robot_type = 'omni'  # omni, acker, diff
    appearance = 'circle'  # shape list: ['circle', 'rectangle', 'polygon']
    state_dim = (2, 1) # the state dimension, x, y, theta(heading direction)
    vel_dim = (2, 1)  # the velocity dimension, linear and angular velocity
    goal_dim = (2, 1) # the goal dimension, x, y, theta
    position_dim=(2, 1) # the position dimension, x, y 
    cone_type = 'norm2' # 'Rpositive'; 'norm2' 

    coefficient_vel = np.zeros((3, 2))

    def __init__(self, id, state=np.zeros((2, 1)), vel=np.zeros((2, 1)), goal=np.zeros((2, 1)), radius=0.2, radius_exp=0.1, vel_min=[-2, -2], vel_max=[2, 2], step_time=0.1, acce=[inf, inf], **kwargs):

        # shape args
        self.radius = radius
        self.radius_collision = radius + radius_exp
        super(RobotOmni, self).__init__(id, state, vel, goal, step_time, vel_min=vel_min, vel_max=vel_max, acce=acce, **kwargs)
        
        self.vel_omni = np.zeros((2, 1))
    
    def dynamics(self, state, vel, noise=False, alpha = [0.01, 0, 0, 0.01, 0, 0], **kwargs):
        # The omni directional robot dynamics
        # reference: Probability robotics, motion model
        # vel: vel_x, vel_y
        
        if env_global.control_mode == 'keyboard':
            vel[(0, 1), 0] = vel[(1, 0), 0]
            vel[0, 0] = -vel[0, 0]

        self.vel_omni = vel
        self.vel = vel
        new_state = RobotOmni.motion_omni(state, self.vel, self.step_time, noise, alpha)
    
        return new_state

    def cal_des_vel(self):
        # calculate desire velocity
        dis, radian = RobotOmni.relative_position(self.state, self.goal)    

        if dis > self.goal_threshold:
            vx = self.vel_max[0, 0] * cos(radian)
            vy = self.vel_max[1, 0] * sin(radian)
        else:
            vx = 0
            vy = 0

        return np.array([[vx], [vy]])

    def gen_inequal(self):
        # generalized inequality, inside: Gx <=_k g, norm2 cone
        G = np.array([ [1, 0], [0, 1], [0, 0] ])
        h = np.array( [ [0], [0], [-self.radius] ] )
        self.h_collision = np.array( [ [0], [0], [-self.radius_collision] ])
        return G, h
    
    def gen_inequal_global(self):
        # generalized inequality, inside: Gx <=_k g, norm2 cone  at current position
        G = np.array([ [1, 0], [0, 1], [0, 0] ])
        h = np.row_stack((self.center, -self.radius * np.ones((1,1))))

        return G, h

    def plot_robot(self, ax, robot_color = 'g', goal_color='r', show_goal=True, show_text=False, show_traj=False, traj_type='-g', fontsize=10, **kwargs):
        x = self.state[0, 0]
        y = self.state[1, 0]
        
        goal_x = self.goal[0, 0]
        goal_y = self.goal[1, 0]

        robot_circle = mpl.patches.Circle(xy=(x, y), radius = self.radius, color = robot_color)
        robot_circle.set_zorder(3)

        ax.add_patch(robot_circle)
        if show_text: ax.text(x - 0.5, y, 'r'+ str(self.id), fontsize = fontsize, color = 'r')
        self.plot_patch_list.append(robot_circle)

        if show_goal:
            goal_circle = mpl.patches.Circle(xy=(goal_x, goal_y), radius = self.radius, color=goal_color, alpha=0.5)
            goal_circle.set_zorder(1)
        
            ax.add_patch(goal_circle)
            if show_text: ax.text(goal_x + 0.3, goal_y, 'g'+ str(self.id), fontsize = fontsize, color = 'k')
            self.plot_patch_list.append(goal_circle)

        if show_traj:
            x_list = [t[0, 0] for t in self.trajectory]
            y_list = [t[1, 0] for t in self.trajectory]
            self.plot_line_list.append(ax.plot(x_list, y_list, traj_type))
    
    @classmethod
    def motion_omni(cls, current_state, vel, step_time, noise = False, std = [0.05, 0.05]):
        assert current_state.shape == cls.state_dim and vel.shape == cls.vel_dim and cls.robot_type == 'omni'

        if noise == True:
            vel_noise = vel + np.random.normal([[0], [0]], scale = [[std[0]], [std[1]]])
        else:
            vel_noise = vel
        
        next_state = current_state + vel_noise * step_time

        return next_state
    
   






        
    
