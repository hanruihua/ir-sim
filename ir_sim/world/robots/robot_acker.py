from ir_sim.world import ObjectBase
from math import sin, cos, tan, pi
import numpy as np
from ir_sim.util.util import WrapToPi, diff_to_omni
from ir_sim.global_param import world_param 
from matplotlib import image
import matplotlib.transforms as mtransforms
from ir_sim.util.util import WrapToRegion, get_transform, get_affine_transform
import matplotlib as mpl
from ir_sim.global_param.path_param import path_manager


class RobotAcker(ObjectBase):
    def __init__(self, shape='rectangle', shape_tuple=None, color='y', **kwargs):
        super(RobotAcker, self).__init__(shape=shape, shape_tuple=shape_tuple, kinematics='acker', role='robot', color=color, **kwargs)

        self.wheelbase = kwargs['wheelbase']
        self.info.add_property('wheelbase', self.wheelbase)
        

    def _kinematics(self, velocity, mode='steer', noise=False, alpha=[0.03, 0, 0, 0.03, 0, 0], **kwargs):
        
        phi = self._state[2, 0]
        psi = self._state[3, 0]

        if mode == 'steer':
            psi_limit = self.vel_max[1, 0]

            co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / self.wheelbase, 0], [0, 1] ])

            velocity[1, 0] = np.clip(velocity[1, 0], -psi_limit, psi_limit)

        elif mode == 'angular':

            co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / self.wheelbase, 0], [0, 1] ])

        elif mode == 'simplify':
            co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / self.wheelbase, 0], [0, 1] ])

        d_state = co_matrix @ velocity
        new_state = self._state + d_state * world_param.step_time
        
        if mode == 'steer': new_state[3, 0] = velocity[1, 0]

        new_state[2, 0] = WrapToPi(new_state[2, 0]) 

        return new_state


    # def plot(self, ax, **kwargs):
    #     super().plot(ax, **kwargs)


    def plot_object(self, ax, **kwargs):
        
        # x = self.vertices[0, 0]
        # y = self.vertices[1, 0]

        start_x = self.vertices[0, 0]
        start_y = self.vertices[1, 0]
        r_phi = self._state[2, 0]
        r_phi_ang = 180*r_phi/pi

        # car_image_path = Path(current_file_frame).parent / 'car0.png'
        car_image_path = path_manager.root_path + '/world/description/car_green.png'
        car_img_read = image.imread(car_image_path)

        car_img = ax.imshow(car_img_read, extent=[start_x, start_x+self.length, start_y, start_y+self.width])
        trans_data = mtransforms.Affine2D().rotate_deg_around(start_x, start_y, r_phi_ang) + ax.transData
        car_img.set_transform(trans_data)

        self.plot_patch_list.append(car_img)


    def plot_goal(self, ax, goal_color='r', buffer_length=0.0, buffer_width=0.1, **kwargs):

        goal_x = self._goal[0, 0]
        goal_y = self._goal[1, 0]
        theta = self._goal[2, 0]     

        l = buffer_length + self.length
        w = buffer_width + self.width   

        arrow = mpl.patches.Arrow(goal_x, goal_y, l *cos(theta), l * sin(theta), width=w, color=goal_color)
        arrow.set_zorder(3)
        ax.add_patch(arrow)
        
<<<<<<< HEAD
        if show_goal:
            goal_arrow = mpl.patches.Arrow(x=self.goal[0, 0], y=self.goal[1, 0], dx=goal_l*cos(self.goal[2, 0]), dy=goal_l*sin(self.goal[2, 0]), color=goal_color)
            ax.add_patch(goal_arrow)
            self.plot_patch_list.append(goal_arrow)

        if show_trail:
            if trail_type == 'rectangle':
                car_rect = mpl.patches.Rectangle(xy=(start_x, start_y), width=self.shape[0], height=self.shape[1], angle=r_phi_ang, edgecolor=self.edgecolor, fill=False, alpha=0.8, linewidth=0.8)
                ax.add_patch(car_rect)
                self.plot_patch_reset_list.append(car_rect)

            elif trail_type == 'circle':
                x = (min(self.vertex[0, :]) + max(self.vertex[0, :])) / 2
                y = (min(self.vertex[1, :]) + max(self.vertex[1, :])) / 2

                car_circle = mpl.patches.Circle(xy=(x, y), radius = self.shape[0] / 2, edgecolor='red', fill=False)
                ax.add_patch(car_circle)
                self.plot_patch_reset_list.append(car_circle)
            
        if show_text:
            t1 = ax.text(start_x - 0.5, start_y, 'c'+ str(self.id), fontsize = 10, color = 'k')
            t2 = ax.text(self.goal[0, 0] + 0.3, self.goal[1, 0], 'cg'+ str(self.id), fontsize = 12, color = 'k')
            self.plot_text_list.append(t1)
            self.plot_text_list.append(t2)

        if show_traj:
            x_list = [t[0, 0] for t in self.trajectory]
            y_list = [t[1, 0] for t in self.trajectory]
            self.plot_line_list.append(ax.plot(x_list, y_list, traj_type))

    def set_edgecolor(self, edgecolor='y'):
        self.edgecolor = edgecolor

    def reset(self):
        self.state = self.init_state.copy()
        self.center = self.init_state[0:2].copy()
        self.goal = self.init_goal.copy()
        self.vel = self.init_vel.copy()

        self.collision_flag = False
        self.arrive_flag = False
        self.stop_flag = False

        self.trajectory = []

        # update vertex
        self.update_vertex(self.state)

        self.plot_clear_reset()
            
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
=======
        self.plot_patch_list.append(arrow)


    @property
    def velocity_xy(self):
        return diff_to_omni(self.state[2, 0], self._velocity)

       












>>>>>>> 4b9462c36ce9207111f7cfb2f208b59e5aba5f84
