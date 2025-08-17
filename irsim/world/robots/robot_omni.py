from typing import Any

from irsim.world.object_base import ObjectBase


class RobotOmni(ObjectBase):
    def __init__(self, color: str = "g", state_dim: int = 3, **kwargs: Any) -> None:
        """Create an omnidirectional robot.

        Args:
            color (str): Display color. Default "g".
            state_dim (int): State vector dimension (>=2 for [x,y]).
            **kwargs: Forwarded to ``ObjectBase``.
        """
        super().__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert state_dim >= 2, (
            "for omni robot, the state dimension should be greater than 2"
        )

    def _init_plot(self, ax: Any, **kwargs: Any) -> None:
        """Initialize matplotlib artists for this robot.

        Args:
            ax: Matplotlib axes.
            **kwargs: Extra plotting options.
        """
        show_goal = self.plot_kwargs.get("show_goal", True)

        super()._init_plot(ax, show_goal=show_goal, **kwargs)
