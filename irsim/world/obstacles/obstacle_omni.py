from irsim.world import ObjectBase


class ObstacleOmni(ObjectBase):
    def __init__(
        self, color="k", state_dim=3, **kwargs
    ):
        super(ObstacleOmni, self).__init__(
            color=color, role="obstacle", state_dim=state_dim, **kwargs
        )
