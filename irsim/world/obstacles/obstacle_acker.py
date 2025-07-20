from irsim.world.object_base import ObjectBase


class ObstacleAcker(ObjectBase):
    def __init__(self, color="k", state_dim=4, **kwargs):
        super(ObstacleAcker, self).__init__( color=color, role="obstacle", state_dim=state_dim, **kwargs
        )
