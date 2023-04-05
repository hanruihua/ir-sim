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


    def __init__(self, id, resolution=0.01, step_time=0.1, dynamic=False, landmark=False, **kwargs):

        '''
        id: an integer representing the ID of the obstacle or landmark.
        resolution: a float representing the resolution of the obstacle or landmark.
        step_time: a float representing the step time of the obstacle or landmark.
        dynamic: a boolean indicating whether the obstacle or landmark is dynamic or not.
        landmark: a boolean indicating whether the object is a landmark or not.
        **kwargs: a variable-length argument list that allows passing of additional arguments.
        '''

        # self.shape
        self.id = int(id)
        self.reso = resolution
        self.step_time = step_time

        self.dynamic = dynamic
        self.A, self.b = self.gen_inequal_global()
        self.obstacle_matrix = self.gen_matrix()
        self.landmark = landmark  # whether landmarks. landmarks can be detected by sensors directly with range and id
        
        # basic arrtibute
        self.vertex = None
        self.radius = None
        self.velocity = np.zeros(ObstacleBase.vel_dim)  # default: x y velocity
        self.name = 'Landmark' + str(self.id) if landmark else 'Obstacle' + str(self.id)


    def collision_check(self):
        raise NotImplementedError
    
    def collision_check_point(self, point):
        # generalized inequality over the norm cone
        # x<=_k x_c
        assert point.shape == (2, 1)
        return ObstacleBase.InCone(self.A @ point - self.b, self.cone_type)

    def collision_check_array(self, point_array):

        assert point_array.shape[0] == 2

        temp = self.A @ point_array - self.b

        if self.cone_type == 'Rpositive':
            collision_matirx = np.all(temp <= 0, axis=0)
        elif self.cone_type == 'norm2':
            collision_matirx = np.squeeze(np.linalg.norm(temp[0:-1], axis=0) - temp[-1]) <= 0

        return collision_matirx


    def gen_matrix(self):
        # discreted model denoted by matrix
        # raise NotImplementedError
        pass
    
    def gen_inequal(self):
        # Calculate the matrix A and b for the Generalized inequality: A @ point <_k b, at init position
        # A, b = self.gen_inequal()
        raise NotImplementedError
    
    def gen_inequal_global(self):
        # Calculate the matrix A and b for the Generalized inequality: A @ point <_k b,  at current position
        # self.A, self.b = self.gen_inequal_global()
        raise NotImplementedError
    
    def get_edges(self):

        edge_list = []
        ver_num = self.vertex.shape[1]

        for i in range(ver_num):
            if i < ver_num - 1:
                edge = [ self.vertex[:, i], self.vertex[:, i+1] ]
            else:
                edge = [ self.vertex[:, i], self.vertex[:, 0] ]

            edge_list.append(edge)

        return edge_list

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

        # for i, ver in enumerate(vertex):
        #     pass
        # for i in range(ver_num-1):
        #     edge = [self.vertexes[0, i], self.vertexes[1, i], self.vertexes[0, i+1], self.vertexes[1, i+1]]
        #     self.edge_list.append(edge)
        
        # edge_final = [ self.vertexes[0, self.ver_num-1], self.vertexes[1, self.ver_num-1], self.vertexes[0, 0], self.vertexes[1, 0] ]
        # self.edge_list.append(edge_final)