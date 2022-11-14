from math import pi, sin, cos, sqrt
import numpy as np
from ir_sim2.env import env_global
from ir_sim2.util.collision_dection_distance import range_cir_seg, range_seg_seg
import time
# from 

class lidar2d:
    def __init__(self, robot_state=np.zeros((3, 1)), range_min=0, range_max=10, angle_range = pi, number=36, scan_time=0.1, noise=False, std=0.2, offset=[0, 0, 0], reso=0.05, **kwargs) -> None:

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

        # self.intersections, self.start_ray = self.init_sections() # 
        self.reso = reso
        self.sample_num = int( (self.range_max - self.range_min) / reso )
        self.scan_matrix = self.init_sections()

        if robot_state.shape[0] == 2 :
            robot_state = np.vstack((robot_state, [0]))

        trans_matirx, rot_matrix = lidar2d.transform_matrix(*np.squeeze(robot_state[0:3]))

        # transform to the global coordinate
        self.global_scan_matrix = rot_matrix @ self.scan_matrix + trans_matirx 
        self.global_intersections = rot_matrix @ self.intersections + trans_matirx 
        self.global_ray = rot_matrix @ self.ray + trans_matirx

        # noise
        self.noise = noise
        self.std = std
    
    def init_sections(self):
        # discrete the scan to generate a scan matrix 
        temp_scan_matrix = self.range_max * np.ones((2, self.sample_num, self.number))

        for i, angle in enumerate(self.angle_list):

            cur_range = self.range_min

            for j in range(self.sample_num):                
                temp_scan_matrix[:, j, i] = cur_range * np.array([ cos(angle), sin(angle) ])
                cur_range += self.reso

        self.intersections = temp_scan_matrix[:, -1, :]
        self.ray = temp_scan_matrix[:, 0, :]

        init_scan_matrix = np.reshape(temp_scan_matrix, (2, self.sample_num*self.number))   
        trans_matirx, rot_matrix = lidar2d.transform_matrix(*self.offset)
        scan_matrix = rot_matrix @ init_scan_matrix + trans_matirx

        return scan_matrix

    def step(self, robot_state=np.zeros((3, 1))):
        # calculate the scan range data
        if robot_state.shape[0] == 2 :
            robot_state = np.vstack((robot_state, [0]))

        trans_matirx, rot_matrix = lidar2d.transform_matrix(*np.squeeze(robot_state))
        self.global_scan_matrix = rot_matrix @ self.scan_matrix + trans_matirx
        self.global_ray = rot_matrix @ self.ray + trans_matirx

        Components = env_global.components.copy()
        com_list = [com for com in Components if lidar2d.distance(com.center, robot_state[0:2]) >= 0.01]

        closest_index_array = self.ray_casting(com_list)

        temp_global_scan_matrix = np.reshape(self.global_scan_matrix, (2, self.sample_num, self.number)) 
        
        for i in range(len(closest_index_array)):   
            self.global_intersections[:, i] = temp_global_scan_matrix[:, closest_index_array[i], i]
            self.range_data[i] = self.range_min + (closest_index_array[i] + 2) * self.reso

        if self.noise:
            self.range_data = self.range_data + np.random.normal(0, self.std, self.range_data.shape)


    def ray_casting(self, com_list):
        # calculate the minimum distance index between global_scan_matrix and obstacles

        index_array = np.zeros((len(com_list), self.number))

        for i, com in enumerate(com_list):
            temp_collision_matrix = com.collision_check_array(self.global_scan_matrix)  # check the collision of the scan array and 
            collision_matrix = np.reshape(temp_collision_matrix, (self.sample_num, self.number))
            collision_matrix[-1, :] = True  # set the end point true
            index = np.argmax(collision_matrix == True, axis = 0) - 1  # find the cloest collision point
            index = np.clip(index, 0, self.sample_num)  # be positive
            index_array[i, :] = index

        closest_index_array = np.min(index_array, axis = 0).astype(int)
            
        return closest_index_array

    # def init_sections(self):

    #     init_intersections = self.range_max * np.ones((2, self.number))
    #     init_start_ray = self.range_min * np.ones((2, self.number))

    #     for i, angle in enumerate(self.angle_list):
    #         init_intersections[:, i] = self.range_max * np.array([ cos(angle), sin(angle) ])
    #         init_start_ray[:, i] = self.range_min * np.array([ cos(angle), sin(angle) ])
        
    #     trans_matirx, rot_matrix = lidar2d.transform_matrix(*self.offset)

    #     intersections = rot_matrix @ init_intersections + trans_matirx
    #     start_ray = rot_matrix @ init_start_ray + trans_matirx

    #     return intersections, start_ray    

    # def step(self, robot_state=np.zeros((3, 1))):
    #     # calculate the scan range data
    #     trans_matirx, rot_matrix = lidar2d.transform_matrix(*np.squeeze(robot_state))

    #     self.global_intersections = rot_matrix @ self.intersections + trans_matirx
    #     self.global_ray = rot_matrix @ self.start_ray + trans_matirx

    #     Components = env_global.components.copy()

    #     com_list = [com for com in Components if lidar2d.distance(com.center, robot_state[0:2]) >= 0.01]

    #     for i, (start, end) in enumerate(zip(self.global_ray.T, self.global_intersections.T)):
    #         ray = [start, end]

    #         collision_flag, min_int_point, lrange = self.ray_casting(ray, com_list)  
            
    #         if collision_flag:
    #             self.global_intersections[:, i] = min_int_point
    #             self.range_data[i] = lrange
    #         else:
    #             self.range_data[i] = self.range_max
            

    # def ray_casting(self, ray, com_list):
    #     # calculate the minimum distance and collision point between a ray and obstacles
    #     min_lrange = self.range_max
    #     min_int_point = ray[1]
    #     collision_flag = False
        
    #     for com in com_list:
    #         if com.appearance == 'circle': 
    #             flag, int_point, lrange = range_cir_seg(com.center, com.radius, ray)
    #             if flag and lrange < min_lrange:
    #                 min_lrange = lrange
    #                 min_int_point = int_point
    #                 collision_flag = True

    #         if com.appearance == 'rectangle' or com.appearance == 'polygon':
                
    #             edge_list = com.get_edges()
    #             for edge in edge_list:
    #                 flag, int_point, lrange = range_seg_seg(ray, edge)
    #                 if flag and lrange < min_lrange:
    #                     min_lrange = lrange
    #                     min_int_point = int_point
    #                     collision_flag = True
                
    #     return collision_flag, min_int_point, min_lrange

    def plot(self, ax, lidar_color='r'):
        
        plot_patch_list = []
        plot_line_list = []

        for start, end in zip(self.global_ray.T, self.global_intersections.T):
            plot_line_list.append(ax.plot([start[0], end[0]], [start[1], end[1]], color = lidar_color, alpha=0.5))

        return plot_line_list, plot_patch_list
 
    @staticmethod
    def transform_matrix(x, y, theta):
        trans_matrix = np.array([[x], [y]])
        rot_matrix = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

        return trans_matrix, rot_matrix

    @staticmethod
    def distance(point1, point2):
        return sqrt( (point1[0, 0] - point2[0, 0]) ** 2 + (point1[1, 0] - point2[1, 0]) ** 2 )




