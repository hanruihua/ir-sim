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
        super(RobotAcker, self).__init__(shape=shape, shape_tuple=shape_tuple, kinematics='acker', role='robot', color=color, state_dim=4, **kwargs)

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
        
        self.plot_patch_list.append(arrow)


    @property
    def velocity_xy(self):
        return diff_to_omni(self.state[2, 0], self._velocity)

       












