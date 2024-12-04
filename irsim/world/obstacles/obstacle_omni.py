from irsim.world import ObjectBase


class ObstacleOmni(ObjectBase):
    def __init__(
        self, kinematics={"name": "omni"}, color="k", **kwargs
    ):
        super(ObstacleOmni, self).__init__(
            kinematics=kinematics, color=color, role="obstacle", **kwargs
        )
