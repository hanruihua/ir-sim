from irsim.world import ObjectBase
import numpy as np
from math import cos, sin, pi
from irsim.util.util import WrapToPi, diff_to_omni
from irsim.global_param import world_param
from irsim.global_param.path_param import path_manager
from matplotlib import image
import matplotlib.transforms as mtransforms


class RobotOmni(ObjectBase):
    def __init__(
        self, color="g", state_dim=2, **kwargs
    ):
        super(RobotOmni, self).__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert (
            state_dim >= 2
        ), "for omni robot, the state dimension should be greater than 2"

    

