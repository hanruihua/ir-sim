from irsim.world import ObjectBase


class ObstacleAcker(ObjectBase):
    def __init__(self, kinematics={"name": "acker"}, color="k", **kwargs):
        super(ObstacleAcker, self).__init__(
            kinematics=kinematics, color=color, role="obstacle", **kwargs
        )
