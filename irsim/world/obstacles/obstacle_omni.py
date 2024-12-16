from irsim.world import ObjectBase


class ObstacleOmni(ObjectBase):
    def __init__(
        self, color="k", **kwargs
    ):
        super(ObstacleOmni, self).__init__(
            color=color, role="obstacle", **kwargs
        )
