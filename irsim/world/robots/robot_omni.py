from irsim.world import ObjectBase
import numpy as np
from math import cos, sin, pi
from irsim.util.util import WrapToPi, diff_to_omni
from irsim.global_param import world_param
from irsim.global_param.path_param import path_manager
from matplotlib import image
import matplotlib.transforms as mtransforms

class RobotOmni(ObjectBase):
    def __init__(self, shape='circle', shape_tuple=None, color='g', state_dim=2, **kwargs):
        super(RobotOmni, self).__init__(shape=shape, shape_tuple=shape_tuple, kinematics='omni', role='robot', color=color, state_dim=state_dim, **kwargs)

    # def _kinematics_step(self, velocity, noise=False, alpha=[0.03, 0.03], **kwargs):
        
    #     '''
    #     The kinematics function for omni wheel robot

    #     state: [x, y, theta]   (3*1) vector
    #     velocity: [vx, vy]  (2*1) vector
    #     '''
        
    #     assert velocity.shape==(2, 1)
    #     assert self._state.shape==(3, 1)

    #     if noise:
    #         std_vx = np.sqrt(alpha[0])
    #         std_vy = np.sqrt(alpha[1])
    #         real_velocity = velocity + np.random.normal([[0], [0], [0]], scale = [[std_vx], [std_vy], [0]])  

    #     else:
    #         real_velocity = velocity

    #     new_position = self._state[0:2] + real_velocity * world_param.step_time

    #     next_state = np.vstack((new_position, WrapToPi(self._state[2, 0])))

    #     return next_state

    def plot(self, ax, **kwargs):

        show_goal = self.plot_kwargs.get('show_goal', True)

        super().plot(ax, show_goal=show_goal, **kwargs)


    def plot_object_image(self, ax, description, **kwargs):
        
        start_x = self.vertices[0, 0]
        start_y = self.vertices[1, 0]
        r_phi = self._state[2, 0]
        r_phi_ang = 180*r_phi/pi

        robot_image_path = path_manager.root_path + '/world/description/' + description
        robot_img_read = image.imread(robot_image_path)

        robot_img = ax.imshow(robot_img_read, extent=[start_x, start_x+self.length, start_y, start_y+self.width])
        trans_data = mtransforms.Affine2D().rotate_deg_around(start_x, start_y, r_phi_ang) + ax.transData
        robot_img.set_transform(trans_data)

        self.plot_patch_list.append(robot_img)


    @property
    def velocity_xy(self):
        return self._velocity
    






        








