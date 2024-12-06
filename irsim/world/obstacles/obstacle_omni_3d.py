from irsim.world import ObjectBase3D


class ObstacleOmni3D(ObjectBase3D):
    def __init__(
        self, kinematics={"name": "omni"}, color="k", **kwargs
    ):
        super(ObstacleOmni3D, self).__init__(
            kinematics=kinematics, color=color, role="obstacle", **kwargs
        )
