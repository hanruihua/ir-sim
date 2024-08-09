from ir_sim.world import ObjectBase



class ObstacleMap(ObjectBase):
    
    def __init__(self, shape='points', shape_tuple=None, color='k', reso=0.1, static=True, **kwargs):
        super(ObstacleMap, self).__init__(shape=shape, shape_tuple=shape_tuple, role='obstacle', color=color, reso=reso, static=static, **kwargs)

    