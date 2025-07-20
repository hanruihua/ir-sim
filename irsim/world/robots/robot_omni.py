from irsim.world.object_base import ObjectBase


class RobotOmni(ObjectBase):
    def __init__(
        self, color="g", state_dim=3, **kwargs
    ):
        super(RobotOmni, self).__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert (
            state_dim >= 2
        ), "for omni robot, the state dimension should be greater than 2"

    
    def _init_plot(self, ax, **kwargs):

        show_goal = self.plot_kwargs.get("show_goal", True)
        
        super()._init_plot(ax, show_goal=show_goal, **kwargs)
