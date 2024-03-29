from math import inf, pi
from ir_sim.world import ObjectBase

class ObstacleStatic(ObjectBase):
    def __init__(self, shape: str = 'circle', shape_tuple=(0, 0, 0.2), color='k', **kwargs):
        super(ObstacleStatic, self).__init__(shape=shape, shape_tuple=shape_tuple, color=color, static=True, **kwargs)

        self.role='obstacle'

    







    