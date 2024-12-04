from irsim.world import ObjectBase
from math import sin, cos, tan, pi
import numpy as np
from irsim.util.util import WrapToPi, diff_to_omni
from irsim.global_param import world_param
from matplotlib import image
import matplotlib.transforms as mtransforms
from irsim.util.util import WrapToRegion, get_transform, get_affine_transform
import matplotlib as mpl
from irsim.global_param.path_param import path_manager
from irsim.lib import kinematics_factory


class RobotAcker(ObjectBase):
    def __init__(
        self, color="y", state_dim=4, description="car_green.png", **kwargs
    ):
        super(RobotAcker, self).__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            description=description,
            **kwargs,
        )

        assert (
            state_dim >= 4
        ), "for ackermann robot, the state dimension should be greater than 4"

    # def plot_goal(
    #     self, ax, goal_color="r", buffer_length=0.0, buffer_width=0.1, **kwargs
    # ):

    #     goal_x = self._goal[0, 0]
    #     goal_y = self._goal[1, 0]
    #     theta = self._goal[2, 0]

    #     l = buffer_length + self.length
    #     w = buffer_width + self.width

    #     arrow = mpl.patches.Arrow(
    #         goal_x, goal_y, l * cos(theta), l * sin(theta), width=w, color=goal_color
    #     )
    #     arrow.set_zorder(3)
    #     ax.add_patch(arrow)

    #     self.plot_patch_list.append(arrow)
