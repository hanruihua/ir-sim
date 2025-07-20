from irsim.world.object_base import ObjectBase


class RobotAcker(ObjectBase):
    def __init__(
        self, color="y", state_dim=4, description="car_green.png", **kwargs
    ):
        super(RobotAcker, self).__init__(
            role="robot",
            color=color,
            state_dim=state_dim,
            description=description,
            **kwargs,
        )

        assert (
            state_dim >= 4
        ), "for ackermann robot, the state dimension should be greater than 4"

