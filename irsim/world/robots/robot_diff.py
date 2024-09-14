from irsim.world import ObjectBase
import numpy as np
from math import cos, sin, pi
from irsim.util.util import WrapToPi, diff_to_omni
from irsim.global_param import world_param
from irsim.global_param.path_param import path_manager
from matplotlib import image
import matplotlib.transforms as mtransforms
from irsim.lib import kinematics_factory


class RobotDiff(ObjectBase):
    def __init__(
        self, shape="circle", shape_tuple=None, color="g", state_dim=3, **kwargs
    ):
        super(RobotDiff, self).__init__(
            shape=shape,
            shape_tuple=shape_tuple,
            kinematics="diff",
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert (
            state_dim >= 3
        ), "for differential robot, the state dimension should be greater than 3"

    def plot(self, ax, **kwargs):

        show_goal = self.plot_kwargs.get("show_goal", True)
        show_arrow = self.plot_kwargs.get("show_arrow", True)

        super().plot(ax, show_goal=show_goal, show_arrow=show_arrow, **kwargs)

    def plot_object_image(self, ax, description, **kwargs):

        # x = self.vertices[0, 0]
        # y = self.vertices[1, 0]

        start_x = self.vertices[0, 0]
        start_y = self.vertices[1, 0]
        r_phi = self._state[2, 0]
        r_phi_ang = 180 * r_phi / pi

        # car_image_path = Path(current_file_frame).parent / 'car0.png'
        robot_image_path = path_manager.root_path + "/world/description/" + description
        robot_img_read = image.imread(robot_image_path)

        robot_img = ax.imshow(
            robot_img_read,
            extent=[start_x, start_x + self.length, start_y, start_y + self.width],
        )
        trans_data = (
            mtransforms.Affine2D().rotate_deg_around(start_x, start_y, r_phi_ang)
            + ax.transData
        )
        robot_img.set_transform(trans_data)

        self.plot_patch_list.append(robot_img)

    @property
    def velocity_xy(self):
        return diff_to_omni(self._state[2, 0], self._velocity)
