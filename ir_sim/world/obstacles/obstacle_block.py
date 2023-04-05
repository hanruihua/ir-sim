from .obstacle_base import ObstacleBase
import matplotlib as mpl
import numpy as np
from math import sin, cos


class ObstacleBlock(ObstacleBase):

    obstacle_type = 'obstacle_block' # circle, polygon
    appearance = 'rectangle'  # circle, polygon, rectangle
    point_dim = (3, 1) # the point dimension, x, y theta
    vel_dim = (2, 1) # the velocity dimension, linear and angular velocity
    goal_dim = (2, 1) # the goal dimension, x, y, theta
    convex = True
    cone_type = 'Rpositive' # 'Rpositive'; 'norm2' 

    def __init__(self, id, center=np.zeros((3, 1)), length=1.0, width=1.0, step_time=0.01, **kwargs):

        if isinstance(center, list): center = np.c_[center]
        # if isinstance(goal, list): goal = np.c_[goal]

        self.center = center
        self.init_center = center
        self.length = length
        self.width = width
        self.init_vertex = ObstacleBlock.cal_rectangle_vertex(length, width)
        rot, trans = ObstacleBlock.center_trans(center)
        self.vertex = rot @ self.init_vertex + trans
        self.radius = None

        assert center.shape == self.point_dim

        super(ObstacleBlock, self).__init__(id=id, step_time=step_time, **kwargs)

        self.plot_patch_list = []

    def gen_inequal_global(self):
        
        temp_vertex = np.c_[self.vertex[0:2], self.vertex[0:2, 0]]   

        point_num = self.vertex.shape[1]
        
        A = np.zeros((point_num, 2))
        b = np.zeros((point_num, 1))

        for i in range(point_num):
            cur_p = temp_vertex[0:2, i]
            next_p = temp_vertex[0:2, i+1]

            diff = next_p - cur_p

            ax = diff[1]
            by = -diff[0]
            c = ax * cur_p[0] + by * cur_p[1]

            A[i, 0] = ax
            A[i, 1] = by
            b[i, 0] = c

        return A, b
    
    def gen_inequal(self):
        
        temp_vertex = np.c_[self.init_vertex[0:2], self.init_vertex[0:2, 0]]   

        point_num = self.init_vertex.shape[1]
        
        A = np.zeros((point_num, 2))
        b = np.zeros((point_num, 1))

        for i in range(point_num):
            cur_p = temp_vertex[0:2, i]
            next_p = temp_vertex[0:2, i+1]

            diff = next_p - cur_p

            ax = diff[1]
            by = -diff[0]
            c = ax * cur_p[0] + by * cur_p[1]

            A[i, 0] = ax
            A[i, 1] = by
            b[i, 0] = c

        return A, b

    def plot(self, ax, obs_poly_color='k', **kwargs): 

        obs_poly = mpl.patches.Polygon(xy=self.vertex.T, closed=True, color=obs_poly_color, **kwargs)
        obs_poly.set_zorder(2)
        ax.add_patch(obs_poly)
        self.plot_patch_list.append(obs_poly)

    def plot_clear(self):
        for patch in self.plot_patch_list:
            patch.remove()

        self.plot_patch_list = []

    @staticmethod
    def cal_rectangle_vertex(length, width):        
        # angular point when the robot is in the zeros point
        # counterclockwise
        # shape [length, width, wheelbase, wheelbase_w]
        start_x = -length/2
        start_y = -width/2

        point0 = np.array([ [start_x], [start_y] ]) # left bottom point
        point1 = np.array([ [start_x+length], [start_y] ])
        point2 = np.array([ [start_x+length], [start_y+width]])
        point3 = np.array([ [start_x], [start_y+width]])

        return np.hstack((point0, point1, point2, point3))

    @staticmethod
    def center_trans(center):
        yaw = center[2, 0]
        trans = center[0:2]
        rot = np.array([ [cos(yaw), -sin(yaw) ], [sin(yaw), cos(yaw)] ])

        return rot, trans