from irsim.world import ObjectBase


class ObstacleAcker(ObjectBase):
    def __init__(self, color="k", **kwargs):
        super(ObstacleAcker, self).__init__( color=color, role="obstacle", **kwargs
        )
