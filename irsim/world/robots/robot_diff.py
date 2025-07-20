from irsim.world.object_base import ObjectBase


class RobotDiff(ObjectBase):
    def __init__(
        self, color="g", state_dim=3, **kwargs
    ):
        super(RobotDiff, self).__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert (
            state_dim >= 3
        ), "for differential robot, the state dimension should be greater than 3"


    def _init_plot(self, ax, **kwargs):

        show_goal = self.plot_kwargs.get("show_goal", True)
        show_arrow = self.plot_kwargs.get("show_arrow", True)

        super()._init_plot(ax, show_goal=show_goal, show_arrow=show_arrow, **kwargs)
