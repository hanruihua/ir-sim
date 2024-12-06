from irsim.world import ObjectBase3D


class ObstacleAcker3D(ObjectBase3D):
    def __init__(self, kinematics={"name": "acker"}, color="k", **kwargs):
        super(ObstacleAcker3D, self).__init__(
            kinematics=kinematics, color=color, role="obstacle", **kwargs
        )
