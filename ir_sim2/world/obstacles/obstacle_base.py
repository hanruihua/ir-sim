import numpy as np

class ObstacleBase:

    obstacle_type = 'circle' # circle, polygon
    point_dim = (2, 1) # the point dimension, x, y
    vel_dim = (2, 1) # the velocity dimension, linear and angular velocity
    goal_dim = (2, 1) # the goal dimension, x, y, theta
    convex = False

    def __init__(self, id, points=[], resolution=0.1, cone_type='Rpositive', **kwargs):
        # self.shape
        self.points = points
        self.reso = resolution

        self.A, self.b = self.gen_inequal()
        self.cone_type = 'Rpositive' # 'Rpositive'; 'norm2' 
        self.obstacle_matrix = self.gen_matrix()
        
    def collision_check(self):
        raise NotImplementedError
    
    def collision_check_point(self, point):
        # generalized inequality over the norm cone
        # x<=_k x_c
        assert point.shape == (2, 1)
        
        return ObstacleBase.InCone(point, self.cone_type='Rpositive')

    def gen_matrix(self):
        pass

    def gen_inequal(self):
        # Calculate the matrix A and b for the Generalized inequality: G @ point <_k g, 
        # self.G, self.g = self.gen_inequal()
        raise NotImplementedError
    
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

    # def 