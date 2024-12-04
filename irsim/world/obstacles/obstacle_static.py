from irsim.world import ObjectBase


class ObjectStatic(ObjectBase):
    def __init__(
        self, kinematics=None, color="k", role="obstacle", **kwargs
    ):
        super(ObjectStatic, self).__init__(
            kinematics=kinematics, color=color, static=True, role=role, **kwargs
        )
        
