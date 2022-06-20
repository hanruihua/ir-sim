from .obstacle_base import ObstacleBase
import matplotlib as mpl
import numpy as np

class ObstacleCircle(ObstacleBase):

    obstacle_type = 'circle' # circle, polygon
    point_dim = (2, 1) # the point dimension, x, y
    vel_dim = (2, 1) # the velocity dimension, linear and angular velocity
    goal_dim = (2, 1) # the goal dimension, x, y, theta
    convex = True

    def __init__(self, id, center=np.zeros((2, 1)), radius=0.2, motion='static', cone_type='norm2', **kwargs):
        super(ObstacleCircle, self).__init__(self, id=id, points=center, **kwargs)

        self.center = center
        self.radius = radius
        self.motion = motion
        self.plot_patch_list = []
    
    def gen_inequal(self):
        A = np.array([ [1, 0], [0, 1], [0, 0] ])
        b = np.array(self.center, -self.radius * np.ones((1,1)))
        return A, b
    
    def gen_matrix(self):
        pass

    def plot(self, ax, obs_cir_color='k', **kwargs): 
        obs_circle = mpl.patches.Circle(xy=(self.center[0, 0], self.center[1, 0]), radius = self.radius, color = obs_cir_color)
        obs_circle.set_zorder(2)
        ax.add_patch(obs_circle)
        self.plot_patch_list.append(obs_circle)

    