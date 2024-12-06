from irsim.world import ObjectBase3D


class ObjectStatic3D(ObjectBase3D):
    def __init__(
        self, kinematics=None, color="k", role="obstacle", **kwargs
    ):
        super(ObjectStatic3D, self).__init__(
            kinematics=kinematics, color=color, static=True, role=role, **kwargs
        )
        
