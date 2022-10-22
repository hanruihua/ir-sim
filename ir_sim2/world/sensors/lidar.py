from math import pi, sin, cos, sqrt
from msilib.schema import Component
import numpy as np
from ir_sim2.env import env_global
from ir_sim2.util.collision_dection_distance import range_cir_seg, range_seg_seg
import time

class lidar2d:
    def __init__(self, range_min=0, range_max=10, angle_range = pi, number=36, scan_time=0.1, noise=False, std=0.2, offset=[0, 0, 0], **kwargs) -> None:

        # scan data
        self.range_min = range_min
        self.range_max = range_max
        self.angle_min = - angle_range/ 2
        self.angle_max = angle_range / 2
        self.angle_inc = angle_range / number
        self.scan_time = scan_time
        self.range_data = range_max * np.ones(number,)

        # offset
        self.offset = offset

        self.number = number
        self.angle_list = np.linspace(self.angle_min, self.angle_max, num=number)
        self.intersections, self.start_ray = self.init_sections()
        
        # noise
        self.noise = noise
        self.std = std
    
    def init_sections(self):

        init_intersections = self.range_max * np.ones((2, self.number))
        init_start_ray = self.range_min * np.ones((2, self.number))

        for i, angle in enumerate(self.angle_list):
            init_intersections[:, i] = self.range_max * np.array([ cos(angle), sin(angle) ])
            init_start_ray[:, i] = self.range_min * np.array([ cos(angle), sin(angle) ])
        
        trans_matirx, rot_matrix = lidar2d.transform_matrix(*self.offset)

        intersections = rot_matrix @ init_intersections + trans_matirx
        start_ray = rot_matrix @ init_start_ray + trans_matirx

        return intersections, start_ray

    def step(self, start_point=np.zeros((3, 1))):
        # calculate the scan range data
        trans_matirx, rot_matrix = lidar2d.transform_matrix(*np.squeeze(start_point))

        self.global_intersections = rot_matrix @ self.intersections + trans_matirx
        self.global_ray = rot_matrix @ self.start_ray + trans_matirx

        Components = env_global.components.copy()

        com_list = [com for com in Components if lidar2d.distance(com.center, start_point[0:2]) >= 0.01]

        start_time = time.time()
        for i, (start, end) in enumerate(zip(self.global_ray.T, self.global_intersections.T)):
            ray = [start, end]

            collision_flag, min_int_point, lrange = self.ray_casting(ray, com_list)  
            
            if collision_flag:
                self.global_intersections[:, i] = min_int_point
                self.range_data[i] = lrange

        end_time = time.time() - start_time 
        print(end_time)

        return collision_flag, min_int_point, lrange


    def ray_casting(self, ray, com_list):
        # calculate the minimum distance and collision point between a ray and obstacles
        min_lrange = self.range_max
        min_int_point = ray[1]
        collision_flag = False
        
        for com in com_list:
            if com.appearance == 'circle': 
                start_time = time.time()
                flag, int_point, lrange = range_cir_seg(com.center, com.radius, ray)
                end_time = time.time() - start_time 
                # print('circle time,', end_time)
                if flag and lrange < min_lrange:
                    min_lrange = lrange
                    min_int_point = int_point
                    collision_flag = True

            if com.appearance == 'rectangle' or com.appearance == 'polygon':
                
                edge_list = com.get_edges()
                for edge in edge_list:
                    start_time = time.time()
                    flag, int_point, lrange = range_seg_seg(ray, edge)
                    end_time = time.time() - start_time 
                    # print('polygon time,', end_time)
                    if flag and lrange < min_lrange:
                        min_lrange = lrange
                        min_int_point = int_point
                        collision_flag = True
                
        return collision_flag, min_int_point, min_lrange
                    
    # def plot(self, ax, **kwargs):
        
    #     ax_list = []

    #     for start, end in zip(self.global_ray.T, self.global_intersections.T):
    #         ax_list.append(ax.plot(start, end, color = 'r', alpha=0.5, **kwargs))

    #     return ax_list

    # for point in robot.lidar.inter_points[:, :]:
        
    #     x_value = [x, point[0]]
    #     y_value = [y, point[1]]

    #     self.lidar_line_list.append(self.ax.plot(x_value, y_value, color = 'b', alpha=0.5))


    
    @staticmethod
    def transform_matrix(x, y, theta):
        trans_matrix = np.array([[x], [y]])
        rot_matrix = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

        return trans_matrix, rot_matrix

        # for point in self.intersections[:, :]:
        #     x_value = [x, point[0]]
        #     y_value = [y, point[1]]
        #     lidar_line_list.append(ax.plot(x_value, y_value, color = 'b', alpha=0.5))

    @staticmethod
    def distance(point1, point2):
        return sqrt( (point1[0, 0] - point2[0, 0]) ** 2 + (point1[1, 0] - point2[1, 0]) ** 2 )




