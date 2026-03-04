import warnings
from typing import Any

from irsim.world.object_base import ObjectBase


class RobotAcker(ObjectBase):
    """Ackermann-steered robot.

    .. deprecated::
        Use ``ObjectBase`` with ``kinematics={'name': 'acker'}`` directly.
        This subclass will be removed in a future version.
    """

    def __init__(
        self,
        color: str = "y",
        state_dim: int = 4,
        description: str = "car_green.png",
        **kwargs: Any,
    ) -> None:
        warnings.warn(
            "RobotAcker is deprecated. Use ObjectBase with kinematics={'name': 'acker'} directly.",
            DeprecationWarning,
            stacklevel=2,
        )
        super().__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            description=description,
            **kwargs,
        )

        assert state_dim >= 4, (
            "for ackermann robot, the state dimension should be greater than 4"
        )
