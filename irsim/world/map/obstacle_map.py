from shapely.strtree import STRtree

from irsim.world.object_base import ObjectBase


class ObstacleMap(ObjectBase):
    def __init__(
        self,
        shape=None,
        color="k",
        static=True,
        **kwargs,
    ):
        if shape is None:
            shape = {"name": "map", "reso": "0.1", "points": None}
        super().__init__(
            shape=shape,
            role="obstacle",
            color=color,
            static=static,
            **kwargs,
        )

        self.linestrings = list(self.geometry.geoms)
        self.geometry_tree = STRtree(self.linestrings)
