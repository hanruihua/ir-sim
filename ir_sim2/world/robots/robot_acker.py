import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from .robot_base import RobotBase
from math import sin, cos, pi

class RobotAcker(RobotBase):
    def __init__(self, id=0, shape=[1.5, 1, 1, 1], psi_limit = pi/4, step_time=0.1, **kwargs):
        super(RobotAcker, self).__init__(id=id, step_time=step_time, **kwargs)
        self.shape = shape # length, width, wheelbase, wheelbase_w
        self.angular_point = self.cal_angular_point(shape)
        self.psi_limit = psi_limit
        

    def cal_init_angular_point(self, shape):
        pass

    def gen_inequal(self,):
        pass