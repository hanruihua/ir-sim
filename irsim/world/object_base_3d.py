from irsim.world import ObjectBase
import matplotlib as mpl
from matplotlib import transforms as mtransforms
from matplotlib import image
from irsim.global_param.path_param import path_manager
from math import pi, atan2, cos, sin
import mpl_toolkits.mplot3d.art3d as art3d
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3D

class ObjectBase3D(ObjectBase):
        
        def __init__(self, **kwargs):
            super().__init__(**kwargs)


        
            
            