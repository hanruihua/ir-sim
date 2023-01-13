import os
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
from .robot_base import RobotBase
from math import sin, cos, pi, tan, inf
from matplotlib import image
from ir_sim.util.util import get_transform


class RobotAcker(RobotBase):

    robot_type = 'acker'  # omni, acker
    appearance = 'rectangle'  # shape list: ['circle', 'rectangle', 'polygon']
    state_dim = (4, 1) # the state dimension,  x,y, phi(heading direction), psi(steering angle) 
    vel_dim = (2, 1)  # the velocity dimension, linear velocity, steering angle (or angular velocity)
    goal_dim = (3, 1) # the goal dimension, x,y, phi 
    position_dim=(2,1) # the position dimension, x, y

    def __init__(self, id=0, state=np.zeros((4, 1)), vel=np.zeros((2, 1)), goal=np.zeros((3, 1)), shape=[4.6, 1.6, 3, 1.6], psi_limit = pi/4, step_time=0.1, arrive_mode='state', vel_min=[-4, -pi/4], vel_max=[4, pi/4], acce=[inf, inf], vel_type='steer', **kwargs):

        self.init_vertex = RobotAcker.cal_vertex(shape)
        # super(RobotAcker, self).state_dim = RobotAcker.state_dim
        super(RobotAcker, self).__init__(id, state, vel, goal, step_time=step_time, arrive_mode=arrive_mode, vel_min=vel_min, vel_max=vel_max, acce=acce, **kwargs)
        
        self.shape = shape # [length, width, wheelbase, wheelbase_w]
        self.wheelbase = shape[2]
        self.psi_limit = np.around(psi_limit, 2)
        self.speed_limit = float(self.vel_max[0, 0])
        self.vel_type = vel_type    # vel_tpe: 'steer': linear velocity, steer angle
                                    #          'angular': linear velocity, angular velocity of steer
                                    #          'simplify': linear velocity, rotation rate, do not consider the steer angle 
        self.update_vertex(self.state)

    def dynamics(self, state, vel, **kwargs):
        # The ackermann robot dynamics
        # l: wheel base
        # reference: Lynch, Kevin M., and Frank C. Park. Modern Robotics: Mechanics, Planning, and Control. 1st ed. Cambridge, MA: Cambridge University Press, 2017.
        # steer:  vel, speed and steer angle
        # angular: vel, speed and angular velocity of steer angle
        # simplify: vel: speed and 

        phi = state[2, 0]  
        psi = state[3, 0]

        if self.vel_type == 'steer':
            co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / self.wheelbase, 0], [0, 1] ])
            
            if vel[1, 0] > self.psi_limit or vel[1, 0] < -self.psi_limit:
                print('The steer is clipped under the psi limit ', self.psi_limit)
                vel[1, 0] = np.clip(vel[1, 0], -self.psi_limit, self.psi_limit)

            # if vel[1, 0] > self.psi_limit:
            #     vel[1,ddddd 0] = self.psi_limit
            # if vel[1, 0] < -self.psi_limit:
            #     vel[1, 0] = -self.psi_limit
                    
        elif self.vel_type == 'angular':
            co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / self.wheelbase, 0], [0, 1] ])
        elif self.vel_type == 'simplify':
            co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [0, 1], [0, 0] ])
        
        d_state = co_matrix @ vel
        new_state = state + d_state * self.step_time

        if self.vel_type == 'steer': new_state[3, 0] = vel[1, 0]

        new_state[2, 0] = RobotAcker.wraptopi(new_state[2, 0]) 

        # update vertex
        self.update_vertex(new_state)

        return new_state

    def update_vertex(self, state):
        trans, rot = get_transform(state)
        self.vertex = rot @ self.init_vertex + trans

    def calcuate_vertex(self, state):
        trans, rot = get_transform(state)
        vertex = rot @ self.init_vertex + trans
        
        return vertex
    
    def cal_des_vel(self, tolerance=0.02):
        if self.arrive_mode == 'position':
            if self.vel_type == 'steer':
                dis, radian = RobotAcker.relative_position(self.state, self.goal)
                robot_radian = self.state[2, 0] + self.state[3, 0]
                steer_opt = 0

                diff_radian = RobotAcker.wraptopi( radian - robot_radian )

                if diff_radian > -tolerance and diff_radian < tolerance: diff_radian = 0

                if dis < self.goal_threshold:
                    v_opt, steer_opt = 0, 0
                else:
                    v_opt = self.vel_max[0, 0]
                    steer_opt = diff_radian
                
                return np.array([[v_opt], [steer_opt]])

            elif self.vel_type == 'angular':
                dis, radian = RobotAcker.relative_position(self.state, self.goal)
                robot_radian = self.state[2, 0] + self.state[3, 0]
                w_opti = 0

                diff_radian = RobotAcker.wraptopi( radian - robot_radian )

                if diff_radian > tolerance: w_opti = self.vel_max[1, 0]
                elif diff_radian < - tolerance: w_opti = - self.vel_max[1, 0]
                else: w_opti = 0

                if dis < self.goal_threshold:
                    v_opti, w_opti = 0, 0
                else:
                    v_opti = self.vel_max[0, 0] * cos(diff_radian)
                    if v_opti < 0: v_opti = 0

                return np.array([[v_opti], [w_opti]])

        elif self.arrive_mode == 'state':

            # temp test, may change in the future
            if self.vel_type == 'steer':
                dis, radian = RobotAcker.relative_position(self.state, self.goal)
                robot_radian = self.state[2, 0] + self.state[3, 0]
                steer_opt = 0

                diff_radian = RobotAcker.wraptopi( radian - robot_radian )
                # diff_radian = 

                if diff_radian > -tolerance and diff_radian < tolerance: diff_radian = 0

                if dis < self.goal_threshold:
                    v_opt, steer_opt = 0, 0
                else:
                    v_opt = self.vel_max[0, 0]
                    steer_opt = np.clip(diff_radian, -self.psi_limit, self.psi_limit) 
                
                return np.array([[v_opt], [steer_opt]])
    
    def gen_inequal_global(self):
        # generalized inequality, inside: Gx <=_k h, norm2 cone at current position

        G = np.zeros((4, 2)) 
        h = np.zeros((4, 1)) 
        
        for i in range(4):
            if i + 1 < 4:
                pre_point = self.vertex[:, i]
                next_point = self.vertex[:, i+1]
            else:
                pre_point = self.vertex[:, i]
                next_point = self.vertex[:, 0]
            
            diff = next_point - pre_point
            
            a = diff[1]
            b = -diff[0]
            c = a * pre_point[0] + b * pre_point[1]

            G[i, 0] = a
            G[i, 1] = b
            h[i, 0] = c 

        return G, h

    def plot_robot(self, ax, show_goal=True, goal_color='c', goal_l=2, show_text=False, show_traj=False, traj_type='-g', show_trail=False, edgecolor='y', trail_type='rectangle', **kwargs):
        # cur_vertex = 
        start_x = self.vertex[0, 0]
        start_y = self.vertex[1, 0]
        r_phi = self.state[2, 0]
        r_phi_ang = 180*r_phi/pi

        # car_image_path = Path(current_file_frame).parent / 'car0.png'
        car_image_path = os.path.dirname(__file__) + '/' + 'car0.png'
        car_img_read = image.imread(car_image_path)

        car_img = ax.imshow(car_img_read, extent=[start_x, start_x+self.shape[0], start_y, start_y+self.shape[1]])
        trans_data = mtransforms.Affine2D().rotate_deg_around(start_x, start_y, r_phi_ang) + ax.transData
        car_img.set_transform(trans_data)
        self.plot_patch_list.append(car_img)
        
        if show_goal:
            goal_arrow = mpl.patches.Arrow(x=self.goal[0, 0], y=self.goal[1, 0], dx=goal_l*cos(self.goal[2, 0]), dy=goal_l*sin(self.goal[2, 0]), color=goal_color)
            ax.add_patch(goal_arrow)
            self.plot_patch_list.append(goal_arrow)

        if show_trail:
            if trail_type == 'rectangle':
                car_rect = mpl.patches.Rectangle(xy=(start_x, start_y), width=self.shape[0], height=self.shape[1], angle=r_phi_ang, edgecolor=edgecolor, fill=False, alpha=0.8, linewidth=0.8)
                ax.add_patch(car_rect)

            elif trail_type == 'circle':
                x = (min(self.vertex[0, :]) + max(self.vertex[0, :])) / 2
                y = (min(self.vertex[1, :]) + max(self.vertex[1, :])) / 2

                car_circle = mpl.patches.Circle(xy=(x, y), radius = self.shape[0] / 2, edgecolor='red', fill=False)
                ax.add_patch(car_circle)
            
        if show_text:
            t1 = ax.text(start_x - 0.5, start_y, 'c'+ str(self.id), fontsize = 10, color = 'k')
            t2 = ax.text(self.goal[0, 0] + 0.3, self.goal[1, 0], 'cg'+ str(self.id), fontsize = 12, color = 'k')
            self.plot_text_list.append(t1)
            self.plot_text_list.append(t2)

        if show_traj:
            x_list = [t[0, 0] for t in self.trajectory]
            y_list = [t[1, 0] for t in self.trajectory]
            self.plot_line_list.append(ax.plot(x_list, y_list, traj_type))


    def reset(self):
        self.state = self.init_state
        self.center = self.init_state[0:2]
        self.goal = self.init_goal_state
        self.vel = self.init_vel

        self.collision_flag = False
        self.arrive_flag = False
        self.stop_flag = False

        self.trajectory = []

        # update vertex
        self.update_vertex(self.state)
            
    @staticmethod
    def cal_vertex(shape):        
        # angular point when the robot is in the zeros point
        # counterclockwise
        # shape [length, width, wheelbase, wheelbase_w]
        start_x = -(shape[0] - shape[2])/2
        start_y = -shape[1]/2

        point0 = np.array([ [start_x], [start_y] ]) # left bottom point
        point1 = np.array([ [start_x+shape[0]], [start_y] ])
        point2 = np.array([ [start_x+shape[0]], [start_y+shape[1]]])
        point3 = np.array([ [start_x], [start_y+shape[1]]])

        return np.hstack((point0, point1, point2, point3))
