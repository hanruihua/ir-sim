from math import pi


class lidar2d:
    def __init__(self, range_min=0, range_max=10, angle_min=0, angle_max=pi, scan_time=0.1) -> None:
        self.range_min = range_min
        self.range_max = range_max
        self.angle_min = angle_min
        self.angle_max = angle_max

        self.angle_inc = (angle_max - angle_min) / number

        self.scan_time = scan_time
        