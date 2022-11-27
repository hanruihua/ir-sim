import numpy as np
import matplotlib as mpl
from .robot_base import RobotBase
from math import sin, cos, atan2, inf, pi
from ir_sim2.util.util import WrapToPi

class RobotDiff(RobotBase):

    robot_type = 'diff'  # omni, acker
    appearance = 'circle'  # shape list: ['circle', 'rectangle', 'polygon']
    state_dim = (3, 1) # the state dimension, x, y, theta(heading direction)
    vel_dim = (2, 1)  # the velocity dimension, linear and angular velocity
    goal_dim = (3, 1) # the goal dimension, x, y, theta
    position_dim=(2,1) # the position dimension, x, y 
    
    def __init__(self, id, state=np.zeros((3, 1)), vel=np.zeros((2, 1)), goal=np.zeros((3, 1)), radius=0.2, radius_exp=0.1, vel_min=[-2, -2], vel_max=[2, 2], step_time=0.1, acce=[inf, inf], alpha=[0.03, 0, 0, 0.03, 0, 0], **kwargs):

        # shape args
        self.radius = radius
        self.radius_collision = radius + radius_exp
        self.shape = radius
        super(RobotDiff, self).__init__(id, state, vel, goal, step_time, vel_min=vel_min, vel_max=vel_max, acce=acce, **kwargs)
        
        self.vel_omni = np.zeros((2, 1))
        
        
        if self.noise:
            self.e_state = {'mean': self.state, 'std': np.array([[0.04], [0.01]])}   # estimated state
        self.alpha = alpha
            
    def dynamics(self, state, vel, vel_type='diff', **kwargs):
        # The differential-wheel robot dynamics
        # reference: Probability robotics, motion model
        # vel_tpe: 'diff' or 'omni'

        if vel_type == 'omni':
            self.vel_omni = vel
            self.vel=RobotDiff.omni_to_diff(state, vel, self.vel_max[1, 0], **kwargs)
            new_state = RobotDiff.motion_diff(state, self.vel, self.step_time, self.noise, self.alpha)

        elif vel_type == 'diff':
            new_state = RobotDiff.motion_diff(state, vel, self.step_time, self.noise, self.alpha)
            self.vel = vel
            self.vel_omni = RobotDiff.diff_to_omni(state, self.vel) 
        
        if self.noise:
            self.e_state['mean'] = RobotDiff.motion_diff(self.e_state['mean'], vel, self.step_time, False, self.alpha, **kwargs) 

        return new_state

    def cal_des_vel(self, tolerance=0.12):
        # calculate desire velocity
        des_vel = np.zeros((2, 1))

        if self.arrive_mode == 'position':

            dis, radian = RobotDiff.relative_position(self.state, self.goal)      

            if dis < self.goal_threshold:
                return des_vel
            else:
                diff_radian = RobotDiff.wraptopi( radian - self.state[2, 0] )
                des_vel[0, 0] = np.clip(self.vel_acce_max[0, 0] * cos(diff_radian), 0, inf) 

                if abs(diff_radian) < tolerance:
                    des_vel[1, 0] = 0
                else:
                    des_vel[1, 0] = self.vel_acce_max[1, 0] * (diff_radian / abs(diff_radian))

        elif self.arrive_mode == 'state':
            pass
        
        return des_vel
    
    def gen_inequal_global(self):
        # generalized inequality, inside: Gx <=_k g, norm2 cone  at current position
        G = np.array([ [1, 0], [0, 1], [0, 0] ])
        h = np.row_stack((self.center, -self.radius * np.ones((1,1))))

        return G, h

    def plot_robot(self, ax, robot_color = 'g', goal_color='r', 
                    show_goal=True, show_text=False, show_traj=False, 
                    show_uncertainty=False, traj_type='-g', fontsize=10, 
                    arrow_width=0.6, arrow_length=0.4, **kwargs):
        x = self.state[0, 0]
        y = self.state[1, 0]
        
        goal_x = self.goal[0, 0]
        goal_y = self.goal[1, 0]

        robot_circle = mpl.patches.Circle(xy=(x, y), radius = self.radius, color = robot_color)
        robot_circle.set_zorder(3)

        ax.add_patch(robot_circle)
        if show_text: 
            r_text = ax.text(x - 0.5, y, 'r'+ str(self.id), fontsize = fontsize, color = 'r')
            self.plot_text_list.append(r_text)

        self.plot_patch_list.append(robot_circle)
        

        # arrow
        theta = self.state[2][0]
        arrow = mpl.patches.Arrow(x, y, arrow_length*cos(theta), arrow_length*sin(theta), width = arrow_width)
        arrow.set_zorder(3)
        ax.add_patch(arrow)
        self.plot_patch_list.append(arrow)

        if show_goal:
            goal_circle = mpl.patches.Circle(xy=(goal_x, goal_y), radius = self.radius, color=goal_color, alpha=0.5)
            goal_circle.set_zorder(1)
        
            ax.add_patch(goal_circle)
            if show_text: 
                g_text = ax.text(goal_x + 0.3, goal_y, 'g'+ str(self.id), fontsize = fontsize, color = 'k')
                self.plot_text_list.append(g_text)

            self.plot_patch_list.append(goal_circle)

        if show_traj:
            x_list = [t[0, 0] for t in self.trajectory]
            y_list = [t[1, 0] for t in self.trajectory]
            self.plot_line_list.append(ax.plot(x_list, y_list, traj_type))

        if show_uncertainty and self.noise:
            scale = 20
           
            ex = self.e_state['mean'][0, 0]
            ey = self.e_state['mean'][1, 0]
            etheta = self.e_state['mean'][2, 0]
            std_x = self.e_state['std'][0, 0]
            std_y = self.e_state['std'][1, 0]

            angle = etheta * (180 / pi)

            ellipse = mpl.patches.Ellipse(xy=(ex, ey), width=scale*std_x, height=scale*std_y, angle=angle, facecolor='gray', alpha=0.8)
            ellipse.set_zorder(1)
            ax.add_patch(ellipse)
            self.plot_patch_list.append(ellipse)


    # reference: Modern Robotics[book], P523 
    @classmethod
    def motion_diff(cls, current_state, vel, step_time, noise = False, alpha = [0.03, 0, 0, 0.03, 0, 0]):
        
        assert current_state.shape == cls.state_dim and vel.shape == cls.vel_dim and cls.robot_type == 'diff'

        if noise:
            std_linear = np.sqrt(alpha[0] * (vel[0, 0] ** 2) + alpha[1] * (vel[1, 0] ** 2))
            std_angular = np.sqrt(alpha[2] * (vel[0, 0] ** 2) + alpha[3] * (vel[1, 0] ** 2))
            # gamma = alpha[4] * (vel[0, 0] ** 2) + alpha[5] * (vel[1, 0] ** 2)
            real_vel = vel + np.random.normal([[0], [0]], scale = [[std_linear], [std_angular]])
        else:
            real_vel = vel

        coefficient_vel = np.zeros((3, 2))
        coefficient_vel[0, 0] = cos(current_state[2, 0])
        coefficient_vel[1, 0] = sin(current_state[2, 0])
        coefficient_vel[2, 1] = 1

        next_state = current_state + coefficient_vel @ real_vel * step_time

        next_state[2, 0] = WrapToPi(next_state[2, 0])

        return next_state

    @staticmethod
    def diff_to_omni(state, vel_diff):
        vel_linear = vel_diff[0, 0]
        theta = state[2, 0]
        vx = vel_linear * cos(theta)
        vy = vel_linear * sin(theta)
        return np.array([[vx], [vy]])

    @staticmethod
    def omni_to_diff(state, vel_omni, w_max=2, guarantee_time = 0.2, tolerance = 0.1, mini_speed=0.02):
        speed = np.sqrt(vel_omni[0, 0] ** 2 + vel_omni[1, 0] ** 2)
        
        if speed <= mini_speed:
            return np.zeros(RobotDiff.vel_dim)

        vel_radians = atan2(vel_omni[1, 0], vel_omni[0, 0])
        robot_radians = state[2, 0]
        diff_radians = robot_radians - vel_radians
        # w_max = self.vel_max[1, 0]

        diff_radians = RobotDiff.wraptopi(diff_radians)

        if abs(diff_radians) < tolerance: w = 0
        else:
            w = -diff_radians / guarantee_time
            if w > w_max: w = w_max
    
        v = speed * cos(diff_radians)
        if v<0: v = 0
    
        return np.array([[v], [w]])
    
    # reference: probabilistic robotics[book], motion model P127 
    @classmethod
    def motion_diff_old(cls, current_state, vel, step_time, noise = False, alpha = [0.01, 0, 0, 0.01, 0, 0]):
        
        assert current_state.shape == cls.state_dim and vel.shape == cls.vel_dim and cls.robot_type == 'diff'

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
            next_state = current_state + np.array([ [-ratio * sin(theta) + ratio * sin(theta + omegat * step_time)], 
                                        [ratio * cos(theta) - ratio * cos(theta + omegat * step_time)], 
                                        [omegat * step_time]])
        else:
            # move in a straight line
            next_state = current_state + np.array([[vt * step_time * cos(theta)], [vt * step_time * sin(theta)], [0]])

        next_state[2, 0] =  float(cls.wraptopi(next_state[2, 0])) 
        
        return next_state 
    
   






        
    
