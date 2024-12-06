from irsim.world import ObjectBase3D


class RobotAcker3D(ObjectBase3D):
    def __init__(
        self, color="y", state_dim=4, description=None, **kwargs
    ):
        super(RobotAcker3D, self).__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            description=description,
            **kwargs,
        )

        assert (
            state_dim >= 4
        ), "for ackermann robot, the state dimension should be greater than 4"

