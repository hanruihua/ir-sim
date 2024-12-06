from irsim.world import ObjectBase3D

class ObstacleDiff3D(ObjectBase3D):
    def __init__(self, kinematics={"name": "diff"}, color="k", **kwargs):
        super(ObstacleDiff3D, self).__init__(
            kinematics=kinematics, color=color, role="obstacle", **kwargs
        )
