from ir_sim.world import ObjectBase
import numpy as np
from math import cos, sin, pi
from ir_sim.util.util import WrapToPi, diff_to_omni
from ir_sim.global_param import world_param
from ir_sim.global_param.path_param import path_manager
from matplotlib import image
import matplotlib.transforms as mtransforms

class RobotDiff(ObjectBase):
    def __init__(self, shape='circle', shape_tuple=None, color='g', **kwargs):
        super(RobotDiff, self).__init__(shape=shape, shape_tuple=shape_tuple, kinematics='diff', role='robot', color=color, state_dim=3, **kwargs)


    def _kinematics(self, velocity, noise=False, alpha=[0.03, 0, 0, 0.03, 0, 0],  **kwargs):
        
        # def differential_wheel_kinematics(state, velocity, step_time, noise=False, alpha = [0.03, 0, 0, 0.03, 0, 0]):

        '''
        The kinematics function for differential wheel robot

        state: [x, y, theta]   (3*1) vector
        velocity: [linear, angular]  (2*1) vector
        '''
        
        assert velocity.shape==(2, 1)
        assert self._state.shape==(3, 1)

        if noise:
            std_linear = np.sqrt(alpha[0] * (velocity[0, 0] ** 2) + alpha[1] * (velocity[1, 0] ** 2))
            std_angular = np.sqrt(alpha[2] * (velocity[0, 0] ** 2) + alpha[3] * (velocity[1, 0] ** 2))
            # gamma = alpha[4] * (velocity[0, 0] ** 2) + alpha[5] * (velocity[1, 0] ** 2)
            real_velocity = velocity + np.random.normal([[0], [0]], scale = [[std_linear], [std_angular]])  

        else:
            real_velocity = velocity

        coefficient_vel = np.zeros((3, 2))
        coefficient_vel[0, 0] = cos(self._state[2, 0])
        coefficient_vel[1, 0] = sin(self._state[2, 0])
        coefficient_vel[2, 1] = 1

        next_state = self._state + coefficient_vel @ real_velocity * world_param.step_time

        next_state[2, 0] = WrapToPi(next_state[2, 0])

        return next_state


    def plot(self, ax, **kwargs):

        show_goal = self.plot_kwargs.get('show_goal', True)
        show_arrow = self.plot_kwargs.get('show_arrow', True)

        super().plot(ax, show_goal=show_goal, show_arrow = show_arrow, **kwargs)



    def plot_object_image(self, ax, description, **kwargs):
        
        # x = self.vertices[0, 0]
        # y = self.vertices[1, 0]

        start_x = self.vertices[0, 0]
        start_y = self.vertices[1, 0]
        r_phi = self._state[2, 0]
        r_phi_ang = 180*r_phi/pi

        # car_image_path = Path(current_file_frame).parent / 'car0.png'
        robot_image_path = path_manager.root_path + '/world/description/' + description
        robot_img_read = image.imread(robot_image_path)

        robot_img = ax.imshow(robot_img_read, extent=[start_x, start_x+self.length, start_y, start_y+self.width])
        trans_data = mtransforms.Affine2D().rotate_deg_around(start_x, start_y, r_phi_ang) + ax.transData
        robot_img.set_transform(trans_data)

        self.plot_patch_list.append(robot_img)

    
    @property
    def velocity_xy(self):
        return diff_to_omni(self._state[2, 0], self._velocity)



    






        








