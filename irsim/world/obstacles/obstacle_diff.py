from irsim.world import ObjectBase

class ObstacleDiff(ObjectBase):
    def __init__(self, kinematics={"name": "diff"}, color="k", **kwargs):
        super(ObstacleDiff, self).__init__(
            kinematics=kinematics, color=color, role="obstacle", **kwargs
        )
