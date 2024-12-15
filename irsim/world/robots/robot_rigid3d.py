from irsim.world import ObjectBase3D


class RobotRigid3D(ObjectBase3D):
    def __init__(
        self, color="g", state_dim=6, **kwargs
    ):
        super(RobotRigid3D, self).__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert (
            state_dim >= 6
        ), "for rigid robot, the state dimension should be greater than 6"

        