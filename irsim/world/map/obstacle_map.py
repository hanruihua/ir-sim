from irsim.world import ObjectBase


class ObstacleMap(ObjectBase):

    def __init__(
        self,
        shape={'name': "points", 'reso': '0.1', 'points': None},
        color="k",
        static=True,
        **kwargs,
    ):
        super(ObstacleMap, self).__init__(
            shape=shape,
            role="obstacle",
            color=color,
            static=static,
            **kwargs,
        )
