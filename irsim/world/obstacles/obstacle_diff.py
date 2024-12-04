from math import inf, pi
from irsim.world import ObjectBase
from irsim.world.robots.robot_diff import RobotDiff
from irsim.util.util import diff_to_omni


class ObstacleDiff(ObjectBase):
    def __init__(self, kinematics={"name": "diff"}, color="k", **kwargs):
        super(ObstacleDiff, self).__init__(
            kinematics=kinematics, color=color, role="obstacle", **kwargs
        )
