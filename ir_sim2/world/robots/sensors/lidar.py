from math import pi
import numpy as np

class lidar2d:
    def __init__(self, range_min=0, range_max=10, angle_min=0, angle_max=pi, number=36, scan_time=0.1, noise=False, std=0.2, offset=np.zeros(3,), **kwargs) -> None:

        # scan data
        self.range_min = range_min
        self.range_max = range_max
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_inc = (angle_max - angle_min) / number
        self.scan_time = scan_time
        self.range_data = range_max * np.ones(self.data_num,)

        self.angle_list = np.linspace(self.angle_min, self.angle_max, num=self.data_num)
        self.inter_points = 100 * np.ones((number, 2))

        # noise
        self.noise = noise
        self.std = std

        # 


    def scan_range(self, start_point=np.zeros((2, 1))):
        pass


    def ray_casting(self):
        pass
    




