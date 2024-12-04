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

