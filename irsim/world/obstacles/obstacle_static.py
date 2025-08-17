from irsim.world.object_base import ObjectBase


class ObjectStatic(ObjectBase):
    def __init__(self, color="k", role="obstacle", state_dim=3, **kwargs):
        """Create a static object (robot or obstacle).

        Args:
            color (str): Display color. Default "k".
            role (str): Role of the object ("robot" or "obstacle").
            state_dim (int): State vector dimension (>=3).
            **kwargs: Forwarded to ``ObjectBase``.
        """
        super().__init__(color=color, role=role, state_dim=state_dim, **kwargs)

        self.static = True
