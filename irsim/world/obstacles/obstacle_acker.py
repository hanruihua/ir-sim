import warnings

from irsim.world.object_base import ObjectBase


class ObstacleAcker(ObjectBase):
    """Ackermann-steered obstacle.

    .. deprecated::
        Use ``ObjectBase`` with ``kinematics={'name': 'acker'}, role='obstacle'`` directly.
        This subclass will be removed in a future version.
    """

    def __init__(self, color="k", state_dim=4, **kwargs):
        warnings.warn(
            "ObstacleAcker is deprecated. Use ObjectBase with kinematics={'name': 'acker'} directly.",
            DeprecationWarning,
            stacklevel=2,
        )
        super().__init__(color=color, role="obstacle", state_dim=state_dim, **kwargs)
