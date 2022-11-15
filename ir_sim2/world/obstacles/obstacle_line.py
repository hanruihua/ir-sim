from .obstacle_base import ObstacleBase
import matplotlib as mpl
import numpy as np
from math import sin, cos


class ObstacleLine(ObstacleBase):

    obstacle_type = 'obstacle_line' # circle, polygon
    appearance = 'line'  # circle, polygon, rectangle
    point_dim = (2, 1) # the point dimension, x, y theta
    convex = True
    cone_type = 'Rpositive' # 'Rpositive'; 'norm2' 

    
