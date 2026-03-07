import warnings

from irsim.world.object_base import ObjectBase


class ObstacleDiff(ObjectBase):
    """Differential-drive obstacle.

    .. deprecated::
        Use ``ObjectBase`` with ``kinematics={name: 'diff'}, role='obstacle'`` directly.
        This subclass will be removed in a future version.
    """

    def __init__(self, color="k", state_dim=3, **kwargs):
        warnings.warn(
            "ObstacleDiff is deprecated. Use ObjectBase with kinematics={'name': 'diff'} directly.",
            DeprecationWarning,
            stacklevel=2,
        )
        super().__init__(color=color, role="obstacle", state_dim=state_dim, **kwargs)
