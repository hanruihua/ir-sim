from irsim.world import ObjectBase


class ObjectStatic(ObjectBase):
    def __init__(
        self, color="k", role="obstacle", **kwargs
    ):
        super(ObjectStatic, self).__init__(color=color, static=True, role=role, **kwargs
        )
        
