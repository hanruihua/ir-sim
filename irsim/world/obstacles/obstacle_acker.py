from irsim.world.object_base import ObjectBase


class ObstacleAcker(ObjectBase):
    def __init__(self, color="k", state_dim=4, **kwargs):
        """Create an Ackermann-steered obstacle.

        Args:
            color (str): Display color. Default "k".
            state_dim (int): State vector dimension (>=4).
            **kwargs: Forwarded to ``ObjectBase``.
        """
        super().__init__(color=color, role="obstacle", state_dim=state_dim, **kwargs)
