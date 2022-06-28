import numpy as np
from .obstacle_base import ObstacleBase
import matplotlib as mpl

class ObstaclePolygon(ObstacleBase):
    obstacle_type = 'obstacle_polygon' # circle, polygon
    appearance = 'polygon'  # circle, polygon
    point_dim = (2, 1) # the point dimension, x, y
    vel_dim = None # the velocity dimension, linear and angular velocity
    goal_dim = None # the goal dimension, x, y, theta
    convex = True
    cone_type = 'Rpositive' # 'Rpositive'; 'norm2' 

    def __init__(self, id, state=[0, 0, 0], points=[], dynamic=False, **kwargs):
        
        if isinstance(state, list): state = np.c_[state]
        self.state = state

        self.init_vertex = np.vstack(points).T

        rot, trans = ObstaclePolygon.get_transform(self.state[0:2], self.state[2, 0])

        self.vertex =  rot @ self.init_vertex + trans 
        # self.points = 
        super(ObstaclePolygon, self).__init__(id=id, dynamic=dynamic, **kwargs)

        self.plot_patch_list = []

    def gen_inequal(self):

        temp_vertex = np.c_[self.vertex, self.vertex[0:2, 0]]   

        point_num = self.vertex.shape[1]
        
        A = np.zeros((point_num, self.point_dim[0]))
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

