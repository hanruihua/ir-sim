import numpy as np
from math import atan2, pi, sin, cos

class ObstacleBase:
    obstacle_type = 'obstacle_circle' # circle, polygon
    appearance = 'circle'  # circle, polygon
    point_dim = (2, 1) # the point dimension, x, y
    vel_dim = (2, 1) # the velocity dimension, linear and angular velocity
    goal_dim = (2, 1) # the goal dimension, x, y, theta
    convex = False
    cone_type = 'Rpositive' # 'Rpositive'; 'norm2' 

    def __init__(self, id, resolution=0.1, step_time=0.1, dynamic=False, **kwargs):
        # self.shape
        self.id = int(id)
        self.reso = resolution
        self.step_time = step_time
        self.dynamic = dynamic
        self.A, self.b = self.gen_inequal()
        self.obstacle_matrix = self.gen_matrix()
        
    def collision_check(self):
        raise NotImplementedError
    
    def collision_check_point(self, point):
        # generalized inequality over the norm cone
        # x<=_k x_c
        assert point.shape == self.point_dim
        return ObstacleBase.InCone(self.A @ point - self.b, self.cone_type)

    def gen_matrix(self):
        # discreted model denoted by matrix
        # raise NotImplementedError
        pass

    def gen_inequal(self):
        # Calculate the matrix A and b for the Generalized inequality: G @ point <_k g, 
        # self.G, self.g = self.gen_inequal()
        raise NotImplementedError
    
    def reset(self):
        self.center = self.init_center

    def plot(self):
        raise NotImplementedError

    def plot_clear(self):
        raise NotImplementedError
    
    @staticmethod
    def InCone(point, cone_type='Rpositive'):
        if cone_type == 'Rpositive':
            return (point<=0).all()
        elif cone_type == 'norm2':
            return np.squeeze(np.linalg.norm(point[0:-1]) - point[-1]) <= 0

    @staticmethod
    def relative_position(position1, position2, topi=True):
        diff = position2[0:ObstacleBase.point_dim[0]]-position1[0:ObstacleBase.point_dim[0]]
        dis = np.linalg.norm(diff)
        radian = atan2(diff[1, 0], diff[0, 0])

        if topi: radian = ObstacleBase.wraptopi(radian)

        return dis, radian  

    @staticmethod
    def wraptopi(radian):

        while radian > pi:
            radian = radian - 2 * pi

        while radian < -pi:
            radian = radian + 2 * pi

        return radian

    @staticmethod
    def get_transform(position, orientation):
        rot = np.array([ [cos(orientation), -sin(orientation)], [sin(orientation), cos(orientation)] ])
        trans = position
        return rot, trans