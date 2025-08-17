from irsim.world.object_base import ObjectBase


class ObstacleDiff(ObjectBase):
    def __init__(self, color="k", state_dim=3, **kwargs):
        """Create a differential-drive obstacle.

        Args:
            color (str): Display color. Default "k".
            state_dim (int): State vector dimension (>=3).
            **kwargs: Forwarded to ``ObjectBase``.
        """
        super().__init__(color=color, role="obstacle", state_dim=state_dim, **kwargs)
