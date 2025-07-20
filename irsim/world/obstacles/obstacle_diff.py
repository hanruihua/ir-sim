from irsim.world.object_base import ObjectBase

class ObstacleDiff(ObjectBase):
    def __init__(self, color="k", state_dim=3, **kwargs):
        super(ObstacleDiff, self).__init__(color=color, role="obstacle", state_dim=state_dim, **kwargs
        )
