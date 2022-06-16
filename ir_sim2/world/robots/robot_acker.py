import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from .robot_base import RobotBase
from math import sin, cos, pi, tan

class RobotAcker(RobotBase):

    robot_type = 'acker'  # omni, acker
    robot_shape = 'rectangle'  # shape list: ['circle', 'rectangle', 'polygon']
    state_dim = (4, 1) # the state dimension,  x,y, phi(heading direction), psi(steering angle) 
    vel_dim = (2, 1)  # the velocity dimension, linear velocity, steering angle (or angular velocity)
    goal_dim = (3, 1) # the goal dimension, x,y, phi 
    position_dim=(2,1) # the position dimension, x, y

    def __init__(self, id=0, state=np.zeros((4, 1)), vel=np.zeros((2, 1)), goal=np.zeros((3, 1)), shape=[4.6, 1.6, 3, 1.6], psi_limit = pi/4, step_time=0.1, arrive_mode='state', vel_min=[-4, -4], vel_max=[4, 4], vel_type='steer', **kwargs):

        self.init_angular_point = RobotAcker.cal_angular_point(shape)

        # super(RobotAcker, self).state_dim = RobotAcker.state_dim
        super(RobotAcker, self).__init__(id, state, vel, goal, step_time=step_time, arrive_mode=arrive_mode, vel_min=vel_min, vel_max=vel_max, **kwargs)
        
        self.shape = shape # [length, width, wheelbase, wheelbase_w]
        self.wheelbase = shape[2]
        self.psi_limit = psi_limit
        self.vel_type = vel_type    # vel_tpe: 'steer': linear velocity, steer angle
                                    #          'angular': linear velocity, angular velocity of steer
                                    #          'simplify': linear velocity, rotation rate, do not consider the steer angle 
        # self.plot_patch_list = []
        # self.plot_line_list = []

    def dynamics(self, state, vel, **kwargs):
        # The ackermann robot dynamics
        # l: wheel base
        # reference: Lynch, Kevin M., and Frank C. Park. Modern Robotics: Mechanics, Planning, and Control. 1st ed. Cambridge, MA: Cambridge University Press, 2017.
       
        phi = state[2, 0]  
        psi = state[3, 0]

        if self.vel_type == 'steer':
            co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / self.wheelbase, 0], [0, 1] ])
            state[3, 0] = 0
    
            if vel[1, 0] > self.psi_limit or vel[1, 0] < -self.psi_limit:
                self.log.logger.info('The psi is clipped to be %s', vel[1, 0])
                vel[1, 0] = np.clip(vel[1, 0], -self.psi_limit, self.psi_limit)

        elif self.vel_type == 'angular':
            co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / self.wheelbase, 0], [0, 1] ])

        elif self.vel_type == 'simplify':
            co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [0, 1], [0, 0] ])
        
        d_state = co_matrix @ vel
        new_state = state + d_state * self.step_time

        new_state[2, 0] = RobotAcker.wraptopi(new_state[2, 0]) 

        # update angular_point
        rot, trans = self.get_transform(self.state[0:2, 0:1], self.state[2, 0])
        self.angular_point = rot @ self.init_angular_point + trans

        return new_state

#     # reference: Modern Robotics: Mechanics, Planning, and Control[book], 13.3.1.3 car-like robot
# def motion_ackermann(state, wheelbase=1, vel=np.zeros((2, 1)), steer_limit=pi/2, step_time=0.1, ack_mode='default', theta_trans=True):
    
#     # motion_mode: default: vel: linear velocity, angular velocity of steer
#     #              steer:   vel：linear velocity, steer angle
#     #              simplify: vel: linear velocity, rotation rate, do not consider the steer angle    
    
#     phi = state[2, 0]  
#     psi = state[3, 0] 
    
#     if ack_mode == 'default':
#         co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / wheelbase, 0], [0, 1] ])
#         d_state = co_matrix @ vel
#         new_state = state + d_state * step_time
    
#     elif ack_mode == 'steer':
#         co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / wheelbase, 0], [0, 0] ])
#         d_state = co_matrix @ vel
#         new_state = state + d_state * step_time
#         new_state[3, 0] = np.clip(vel[1, 0], -steer_limit, steer_limit)

#     elif ack_mode == 'simplify':

#         new_state = np.zeros((4, 1))
#         co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [0, 1] ])
#         d_state = co_matrix @ vel
#         new_state[0:3] = state[0:3] + d_state * step_time

#     if theta_trans:
#         new_state[2, 0] = wraptopi(new_state[2, 0]) 
        
#     new_state[3, 0] = np.clip(new_state[3, 0], -steer_limit, steer_limit) 

#     return new_state

    def cal_des_vel(self, tolerance=0.02):
        if self.arrive_mode == 'position':
            if self.vel_type == 'steer':
                dis, radian = RobotAcker.relative_position(self.state, self.goal)
                robot_radian = self.state[2, 0]
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
                robot_radian = self.state[2, 0]
                w_opti = 0

                diff_radian = RobotAcker.wraptopi( radian - robot_radian )

                if diff_radian > tolerance: w_opti = self.vel_max[1, 0]
                elif diff_radian < - tolerance: w_opti = - self.vel_max[1, 0]

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
                robot_radian = self.state[2, 0]
                steer_opt = 0

                diff_radian = RobotAcker.wraptopi( radian - robot_radian )

                if diff_radian > -tolerance and diff_radian < tolerance: diff_radian = 0

                if dis < self.goal_threshold:
                    v_opt, steer_opt = 0, 0
                else:
                    v_opt = self.vel_max[0, 0]
                    steer_opt = diff_radian
                
                return np.array([[v_opt], [steer_opt]])

    def gen_inequal(self):

        G = np.zeros((4, 2)) 
        h = np.zeros((4, 1)) 
        
        for i in range(4):
            if i + 1 < 4:
                pre_point = self.init_angular_point[:, i]
                next_point = self.init_angular_point[:, i+1]
            else:
                pre_point = self.init_angular_point[:, i]
                next_point = self.init_angular_point[:, 0]
            
            diff = next_point - pre_point
            
            a = diff[1]
            b = -diff[0]
            c = a * pre_point[0] + b * pre_point[1]

            G[i, 0] = a
            G[i, 1] = b
            h[i, 0] = c 

        return G, h

    def plot(self, ax, ):
        # cur_angular_point = 
        start_x = self.angular_point[0, 0]
        start_y = self.angular_point[1, 0]
        r_phi = self.state[2, 0]




    def draw_car(self, car, goal_color='c', goal_l=2, text=False, show_lidar=True, show_traj=False, traj_type='-g', show_goal=True, **kwargs):

        x = car.ang_pos[0, 0]
        y = car.ang_pos[1, 0]
        r_phi=car.state[2, 0]

        r_phi_ang = 180*r_phi/pi

        line_rad_f = car.state[3, 0] + car.state[2, 0]
        line_rad_b = car.state[2, 0]

        gx = car.goal[0, 0]
        gy = car.goal[1, 0]
        gdx = goal_l*cos(car.goal[2, 0])
        gdy = goal_l*sin(car.goal[2, 0])
        # self.car_line_list = []
        self.car_img_show_list = []

        # for i in range(4):

        #     if 0 < i < 3:
        #         # wx = car.wheel_pos[0, i]
        #         # wy = car.wheel_pos[1, i]
        #         wx = car.ang_pos[0, i]
        #         wy = car.ang_pos[1, i]

        #         lx0 = wx + line_length * cos(line_rad_f) / 2
        #         ly0 = wy + line_length * sin(line_rad_f) / 2

        #         lx1 = wx - line_length * cos(line_rad_f) / 2
        #         ly1 = wy - line_length * sin(line_rad_f) / 2
                
        #         self.car_line_list.append(self.ax.plot([lx0, lx1], [ly0, ly1], 'k-')) 

        #     else:
        #         # wx = car.wheel_pos[0, i]
        #         # wy = car.wheel_pos[1, i]
        #         wx = car.ang_pos[0, i]
        #         wy = car.ang_pos[1, i]

        #         lx0 = wx + line_length * cos(line_rad_b) / 2
        #         ly0 = wy + line_length * sin(line_rad_b) / 2

        #         lx1 = wx - line_length * cos(line_rad_b) / 2
        #         ly1 = wy - line_length * sin(line_rad_b) / 2

        #         self.car_line_list.append(self.ax.plot([lx0, lx1], [ly0, ly1], 'k-'))
                
        car_rect = mpl.patches.Rectangle(xy=(x, y), width=car.length, height=car.width, angle=r_phi_ang, edgecolor='y', fill=False)
        if show_goal:
            goal_arrow = mpl.patches.Arrow(x=gx, y=gy, dx=gdx, dy=gdy, color=goal_color)
            self.car_plot_list.append(goal_arrow)
            self.ax.add_patch(goal_arrow)

        self.car_plot_list.append(car_rect)
        self.ax.add_patch(car_rect)
        
        
        # car image show
        
        car_img = self.ax.imshow(self.init_car_img, extent=[x, x+car.length, y, y+car.width])
        degree = car.state[2, 0] * 180 / pi
        trans_data = mtransforms.Affine2D().rotate_deg_around(x, y, degree) + self.ax.transData
        car_img.set_transform(trans_data)
        self.car_img_show_list.append(car_img)

        if text:
            self.ax.text(x - 0.5, y, 'c'+ str(car.id), fontsize = 10, color = 'k')
            self.ax.text(car.goal[0, 0] + 0.3, car.goal[1, 0], 'cg'+ str(car.id), fontsize = 12, color = 'k')

        if show_traj:
            x_list = [car.previous_state[0, 0], car.state[0, 0]]  
            y_list = [car.previous_state[1, 0], car.state[1, 0]]   
            
            self.ax.plot(x_list, y_list, traj_type)

        if car.lidar is not None and show_lidar:
            for point in car.lidar.inter_points[:, :]:
                
                x_value = [car.state[0, 0], point[0]]
                y_value = [car.state[1, 0], point[1]]

                self.lidar_line_list.append(self.ax.plot(x_value, y_value, color = 'b', alpha=0.5))



    
    @staticmethod
    def cal_angular_point(shape):        
        # angular point when the robot is in the zeros point
        # counterclockwise
        # shape [length, width, wheelbase, wheelbase_w]
        start_x = -(shape[0] - shape[2])/2
        start_y = -(shape[1] - shape[3])/2

        point0 = np.array([ [start_x], [start_y] ]) # left bottom point
        point1 = np.array([ [start_x+shape[0]], [start_y] ])
        point2 = np.array([ [start_x+shape[0]], [start_y+shape[1]]])
        point3 = np.array([ [start_x], [start_y+shape[1]]])

        return np.hstack((point0, point1, point2, point3))
