import numpy as np
from .obstacle_base import ObstacleBase
import matplotlib as mpl

class ObstaclePolygon(ObstacleBase):
    obstacle_type = 'obstacle_polygon' # circle, polygon
    obstacle_shape = 'polygon'  # circle, polygon
    point_dim = (2, 1) # the point dimension, x, y
    vel_dim = None # the velocity dimension, linear and angular velocity
    goal_dim = None # the goal dimension, x, y, theta
    convex = True
    cone_type = 'Rpositive' # 'Rpositive'; 'norm2' 

    def __init__(self, id, state=[0, 0, 0], points=[], dynamic=False, **kwargs):
        
        if isinstance(state, list): state = np.c_[state]
        self.state = state
        self.init_points = [np.c_[p] for p in points]

        rot, trans = ObstaclePolygon.get_transform(self.state[0:2], self.state[2, 0])

        self.points = [rot @ p + trans for p in self.init_points]
        # self.points = 
        super(ObstaclePolygon, self).__init__(id=id, dynamic=dynamic, **kwargs)

        self.plot_patch_list = []

    def gen_inequal(self):

        point_num = len(self.points)
        
        A = np.zeros((point_num, self.point_dim[0]))
        b = np.zeros((point_num, 1))

        temp_points = self.points + [self.points[0]]

        for i in range(point_num):
            cur_p = temp_points[i] 
            next_p = temp_points[i+1]

            diff = next_p - cur_p

            ax = diff[1, 0]
            by = -diff[0, 0]
            c = ax * cur_p[0, 0] + by * cur_p[1, 0]

            A[i, 0] = ax
            A[i, 1] = by
            b[i, 0] = c 

        return A, b


    def plot(self, ax, obs_poly_color='k', **kwargs): 

        poly_np = np.hstack(self.points).T
        obs_poly = mpl.patches.Polygon(xy=poly_np, closed=True, color=obs_poly_color, **kwargs)
        obs_poly.set_zorder(2)
        ax.add_patch(obs_poly)
        self.plot_patch_list.append(obs_poly)

    def plot_clear(self):
        for patch in self.plot_patch_list:
            patch.remove()

        self.plot_patch_list = []

