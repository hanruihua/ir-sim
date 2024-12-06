from irsim.world import ObjectBase3D


class RobotDiff3D(ObjectBase3D):
    def __init__(
        self, kinematics={'name': "diff"}, color="g", state_dim=3, **kwargs
    ):
        super(RobotDiff3D, self).__init__(
            kinematics=kinematics,
            role="robot",
            color=color,
            state_dim=state_dim,
            **kwargs,
        )

        assert (
            state_dim >= 3
        ), "for differential robot, the state dimension should be greater than 3"
