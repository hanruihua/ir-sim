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
        self, kinematics={'name': "diff"}, color="g", state_dim=3, **kwargs
    ):
        super(RobotDiff, self).__init__(
            kinematics=kinematics,
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert (
            state_dim >= 3
        ), "for differential robot, the state dimension should be greater than 3"

    # def plot(self, ax, **kwargs):

    #     show_goal = self.plot_kwargs.get("show_goal", True)
    #     show_arrow = self.plot_kwargs.get("show_arrow", True)

    #     super().plot(ax, show_goal=show_goal, show_arrow=show_arrow, **kwargs)

