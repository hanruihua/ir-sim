from ir_sim.world import ObjectBase


class RobotDiff(ObjectBase):
    def __init__(self, shape='circle', shape_tuple=None, **kwargs):
        super(RobotDiff, self).__init__(shape=shape, shape_tuple=shape_tuple, kinematics='diff', role='robot', **kwargs)


    @classmethod
    def construct_with_shape(cls, shape, **kwargs):

        if shape == 'circle':

            radius = kwargs.get('radius', 0.2) 

            return cls(shape='circle', shape_tuple=(0, 0, radius), **kwargs)

        elif shape == 'rectangle':

            length = kwargs.get('length', 0.2)
            width = kwargs.get('width', 0.1)

            return cls(shape='polygon', shape_tuple=[(-length/2, -width/2), (length/2, -width/2), (length/2, width/2), (-length/2, width/2)], **kwargs)


        
# shape='circle', shape_tuple=(0, 0, radius), kinematics='diff', role='robot', **kwargs

    def plot(self):
        pass







        








