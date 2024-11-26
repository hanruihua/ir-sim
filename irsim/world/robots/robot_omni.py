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
        self, shape="circle", shape_tuple=None, color="g", state_dim=2, **kwargs
    ):
        super(RobotOmni, self).__init__(
            shape=shape,
            shape_tuple=shape_tuple,
            kinematics="omni",
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert (
            state_dim >= 2
        ), "for differential robot, the state dimension should be greater than 2"

    

