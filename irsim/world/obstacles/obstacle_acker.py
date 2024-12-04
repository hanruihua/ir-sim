from math import inf, pi
from irsim.world import ObjectBase
from irsim.world.robots.robot_acker import RobotAcker
from irsim.util.util import diff_to_omni


class ObstacleAcker(ObjectBase):
    def __init__(self, kinematics={"name": "acker"}, color="k", **kwargs):
        super(ObstacleAcker, self).__init__(
            kinematics=kinematics, color=color, role="obstacle", **kwargs
        )
