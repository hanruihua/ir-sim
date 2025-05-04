from irsim.world import ObjectBase
from shapely.strtree import STRtree

class ObstacleMap(ObjectBase):

    def __init__(
        self,
        shape={"name": "map", "reso": "0.1", "points": None},
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


        self.linestrings = [line for line in self.geometry.geoms]
        self.geometry_tree = STRtree(self.linestrings)


    
