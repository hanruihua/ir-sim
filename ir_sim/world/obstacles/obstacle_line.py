from .obstacle_base import ObstacleBase
import matplotlib as mpl
import numpy as np
from math import sin, cos
from ir_sim.util.util import get_transform

class ObstacleLine(ObstacleBase):

    obstacle_type = 'obstacle_line' # circle, polygon
    appearance = 'segment'  # circle, polygon, rectangle, segment
    point_dim = (2, 1) # the point dimension, x, y
    convex = True
    cone_type = 'Rpositive' # 'Rpositive'; 'norm2' 

    def __init__(self, id, center=np.zeros((3, 1)), length = 1, thickness = 0.2, step_time=0.1, **kwargs):
        
        # the segment is considered as a thin block
        # center: the transformation of the segment, 3*1, x,y,theta
        # start_point, end point: middle point of the broadside of the thin block
        # thickness: the wide of the thin block

        if isinstance(center, list): center = np.c_[center]

        self.length = length
        self.thickness = thickness

        self.init_center = np.zeros((3, 1))
        self.init_points = self.cal_init_points()
        self.init_vertex = self.cal_init_vertex()

        self.center = center
    
        self.points = self.update_points(center) # list of start and end points 
        self.vertex = self.update_vertex(center) # vertex points, 2*4
        self.radius = None

        super(ObstacleLine, self).__init__(id=id, step_time=step_time, **kwargs)

        self.plot_patch_list = []

    def cal_init_points(self):
        start_point = self.init_center[0:2]
        unit = np.array([[cos(self.init_center[2, 0])], [sin(self.init_center[2, 0])]])
        end_point = start_point + self.length * unit

        return [start_point, end_point]

    def cal_init_vertex(self):
        
        start_x = self.init_center[0, 0]
        start_y = self.init_center[1, 0] - self.thickness/2
        
        # counterclockwise
        point0 = np.array([ [start_x], [start_y] ]) # left bottom point
        point1 = np.array([ [start_x+self.length], [start_y] ])
        point2 = np.array([ [start_x+self.length], [start_y+self.thickness]])
        point3 = np.array([ [start_x], [start_y+self.thickness]])

        return np.hstack((point0, point1, point2, point3))

    def update_vertex(self, center):
        trans, rot = get_transform(center)
        vertex = rot @ self.init_vertex + trans
        return vertex

    def update_points(self, center):
        trans, rot = get_transform(center)
        start_point = rot @ self.init_points[0] + trans
        end_point = rot @ self.init_points[1] + trans

        return [start_point, end_point]

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


    def plot(self, ax, obs_seg_color='k', **kwargs): 

        obs_poly = mpl.patches.Polygon(xy=self.vertex.T, closed=True, color=obs_seg_color, **kwargs)
        obs_poly.set_zorder(2)
        ax.add_patch(obs_poly)
        self.plot_patch_list.append(obs_poly)

    def plot_clear(self):
        for patch in self.plot_patch_list:
            patch.remove()

        self.plot_patch_list = []




