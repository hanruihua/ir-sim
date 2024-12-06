from irsim.world import ObjectBase3D


class RobotOmni3D(ObjectBase3D):
    def __init__(
        self, color="g", state_dim=2, **kwargs
    ):
        super(RobotOmni3D, self).__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert (
            state_dim >= 2
        ), "for omni robot, the state dimension should be greater than 2"

    

