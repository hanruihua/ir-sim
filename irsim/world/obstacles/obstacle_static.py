from irsim.world.object_base import ObjectBase


class ObjectStatic(ObjectBase):
    def __init__(self, color="k", role="obstacle", state_dim=3, **kwargs):
        super().__init__(color=color, role=role, state_dim=state_dim, **kwargs)

        self.static = True
