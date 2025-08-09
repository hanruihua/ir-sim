from typing import Any

from irsim.world.object_base import ObjectBase


class RobotDiff(ObjectBase):
    def __init__(self, color: str = "g", state_dim: int = 3, **kwargs: Any) -> None:
        super().__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert state_dim >= 3, (
            "for differential robot, the state dimension should be greater than 3"
        )

    def _init_plot(self, ax: Any, **kwargs: Any) -> None:
        show_goal = self.plot_kwargs.get("show_goal", True)
        show_arrow = self.plot_kwargs.get("show_arrow", True)

        super()._init_plot(ax, show_goal=show_goal, show_arrow=show_arrow, **kwargs)
