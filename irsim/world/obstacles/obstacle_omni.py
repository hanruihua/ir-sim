from math import inf, pi
from irsim.world import ObjectBase
from irsim.world.robots.robot_omni import RobotOmni
from irsim.util.util import diff_to_omni


class ObstacleOmni(ObjectBase):
    def __init__(
        self, kinematics={"name": "omni"}, color="k", **kwargs
    ):
        super(ObstacleOmni, self).__init__(
            kinematics=kinematics, color=color, role="obstacle", **kwargs
        )
