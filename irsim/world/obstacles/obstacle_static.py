from math import inf, pi
from irsim.world import ObjectBase


class ObstacleStatic(ObjectBase):
    def __init__(
        self, kinematics=None, color="k", **kwargs
    ):
        super(ObstacleStatic, self).__init__(
            kinematics=kinematics, color=color, static=True, role="obstacle", **kwargs
        )
        
