import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from .robot_base import RobotBase
from math import sin, cos, pi

class RobotDiff(RobotBase):

    robot_type = 'diff'
    robot_shape = 'circle'

    def __init__(self, id, step_time=0.1, radius=0.2, radius_exp=0.1, vel_min=[-2, -2], vel_max=[2, 2], **kwargs):

        self.radius = radius
        self.radius_collision = radius + radius_exp
        super(RobotDiff, self).__init__(id=id, step_time=step_time, **kwargs)
        
        self.vel_omni = np.zeros(self.vel_dim)

        self.plot_patch_list = []
        self.plot_line_list = []

    def dynamics(self, vel, vel_type='diff', noise=False, alpha = [0.01, 0, 0, 0.01, 0, 0], **kwargs):
        # The differential-wheel robot dynamics
        # reference: Probability robotics, motion model
        # vel_tpe: 'diff' or 'omni'
        if vel_type == 'omni':
            self.vel_omni = vel
            self.vel=RobotDiff.omni_to_diff(self.state, vel, **kwargs)
            self.state = RobotDiff.motion_diff(self.state, self.vel, step_time, noise, alpha)

        elif vel_type == 'diff':
            self.state = RobotDiff.motion_diff(self.state, vel, step_time, noise, alpha)
            self.vel = vel
            self.vel_omni = RobotDiff.diff_to_omni(self.state, self.vel) 

    def gen_inequal(self):
        # generalized inequality, inside: Gx <=_k g, norm2 cone
        G = np.array([ [1, 0], [0, 1], [0, 0] ])
        g = np.array( [ [0], [0], [-self.radius] ] )
        self.g_collision = np.array( [ [0], [0], [-self.radius_collision] ])
        return G, g

    def plot(self, ax, robot_color = 'g', goal_color='r', show_lidar=True, show_goal=False, show_text=True, show_traj=False, traj_type='-g', fontsize=10, **kwargs):
        x = self.state[0, 0]
        y = self.state[1, 0]
        
        goal_x = self.goal[0, 0]
        goal_y = self.goal[1, 0]

        lidar_line_list = []

        robot_circle = mpl.patches.Circle(xy=(x, y), radius = self.radius, color = robot_color)
        robot_circle.set_zorder(2)

        ax.add_patch(robot_circle)
        if show_text: ax.text(x - 0.5, y, 'r'+ str(self.id), fontsize = fontsize, color = 'k')
        self.plot_patch_list.append(robot_circle)

        # arrow
        theta = self.state[2][0]
        arrow = mpl.patches.Arrow(x, y, 0.5*cos(theta), 0.5*sin(theta), width = 0.6)
        ax.add_patch(arrow)
        self.plot_patch_list.append(arrow)

        if show_goal:
            goal_circle = mpl.patches.Circle(xy=(goal_x, goal_y), radius = self.radius, color=goal_color, alpha=0.5)
            goal_circle.set_zorder(1)
        
            ax.add_patch(goal_circle)
            if show_text: ax.text(goal_x + 0.3, goal_y, 'g'+ str(self.id), fontsize = fontsize, color = 'k')
            self.plot_patch_list.append(goal_circle)

        if show_traj:
            x_list = [self.previous_state[0, 0], self.state[0, 0]]  
            y_list = [self.previous_state[1, 0], self.state[1, 0]]   
            ax.plot(x_list, y_list, traj_type)

        if self.lidar is not None and show_lidar:
            for point in robot.lidar.inter_points[:, :]:
                x_value = [x, point[0]]
                y_value = [y, point[1]]
                lidar_line_list.append(ax.plot(x_value, y_value, color = 'b', alpha=0.5))
        
    def plot_clear(self):
        for patch in self.plot_patch_list:
            patch.remove()
        for line in self.plot_line_list:
            line.pop(0).remove()
    
    # reference: probabilistic robotics[book], motion model P127
    @classmethod
    def motion_diff(cls, current_state, vel, sampletime, noise = False, alpha = [0.01, 0, 0, 0.01, 0, 0]):
        
        assert current_state.shape == self.state_dim and vel.shape == self.vel_dim and cls.robot_type == 'diff'

        if noise:
            std_linear = np.sqrt(alpha[0] * (vel[0, 0] ** 2) + alpha[1] * (vel[1, 0] ** 2))
            std_angular = np.sqrt(alpha[2] * (vel[0, 0] ** 2) + alpha[3] * (vel[1, 0] ** 2))
            # gamma = alpha[4] * (vel[0, 0] ** 2) + alpha[5] * (vel[1, 0] ** 2)
            real_vel = vel + np.random.normal([[0], [0]], scale = [[std_linear], [std_angular]])
        else:
            real_vel = vel

        vt = float(real_vel[0, 0])
        omegat = float(real_vel[1, 0])
        theta = float(cls.wraptopi(current_state[2, 0]))

        if omegat >= 0.01 or omegat <= -0.01:
            ratio = vt/omegat
            next_state = current_state + np.array([ [-ratio * sin(theta) + ratio * sin(theta + omegat * sampletime)], 
                                        [ratio * cos(theta) - ratio * cos(theta + omegat * sampletime)], 
                                        [omegat * sampletime]])
        else:
            next_state = current_state + np.array([[vt * sampletime * cos(theta)], [vt * sampletime * sin(theta)], [0]])

        next_state[2, 0] =  float(cls.wraptopi(next_state[2, 0])) 
        
        return next_state 

    @staticmethod
    def diff_to_omni(state, vel_diff):
        vel_linear = vel_diff[0, 0]
        theta = state[2, 0]
        vx = vel_linear * cos(theta)
        vy = vel_linear * sin(theta)
        return np.array([[vx], [vy]])

    @staticmethod
    def omni_to_diff(state, vel_omni, guarantee_time = 0.2, tolerance = 0.1, mini_speed=0.02):
        speed = sqrt(vel_omni[0, 0] ** 2 + vel_omni[1, 0] ** 2)
        
        if speed <= 0.01:
            return np.zeros(self.vel_dim)

        vel_radians = atan2(vel_omni[1, 0], vel_omni[0, 0])
        robot_radians = self.state[2, 0]
        diff_radians = robot_radians - vel_radians
        w_max = self.vel_max[1, 0]

        diff_radians = RobotDiff.wraptopi(diff_radians)

        if abs(diff_radians) < tolerance: w = 0
        else:
            w = -diff_radians / guarantee_time
            if w > w_max: w = w_max
    
        v = speed * cos(diff_radians)
        if v<0: v = 0
    
        return np.array([[v], [w]])
    
    @staticmethod
    def wraptopi(radian):

        while radian > pi:
            radian = radian - 2 * pi

        while radian < -pi:
            radian = radian + 2 * pi

        return radian





        
    
