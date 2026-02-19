from typing import Any

from irsim.world.object_base import ObjectBase


class RobotOmniAngular(ObjectBase):
    vel_shape = (3, 1)

    def __init__(self, color: str = "g", state_dim: int = 3, **kwargs: Any) -> None:
        """Create an omnidirectional robot with angular velocity control.

        Args:
            color (str): Display color. Default "g".
            state_dim (int): State vector dimension (>=3 for [x,y,theta]).
            **kwargs: Forwarded to ``ObjectBase``.
        """
        super().__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert state_dim >= 3, (
            "for omni_angular robot, the state dimension should be greater than or equal to 3"
        )

    def _init_plot(self, ax: Any, **kwargs: Any) -> None:
        """Initialize matplotlib artists for this robot.

        Args:
            ax: Matplotlib axes.
            **kwargs: Extra plotting options.
        """
        show_goal = self.plot_kwargs.get("show_goal", True)
        show_arrow = self.plot_kwargs.get("show_arrow", True)

        super()._init_plot(ax, show_goal=show_goal, show_arrow=show_arrow, **kwargs)
