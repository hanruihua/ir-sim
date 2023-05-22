import numpy as np
from math import sqrt, inf, pi, sin, cos
import os

def random_points(number, low, high, point_distance, max_iter=500):
    # random distribute some points into a selected area with a minimum distance
    # low: n*1   high: n*1
    point_list = []
    iter_loop = 0

    while len(point_list) < number:
        point = np.random.uniform(low, high)
        if min_distance(point, point_list) >= point_distance:
            point_list.append(point)
        
        iter_loop += 1

        if iter_loop > max_iter:
            print('error of the minimum distance')
            break

    return point_list

def random_value(number, low=0.1, high=1):

    if isinstance(low, list):
        low = np.array(low)
        high = np.array(high)
        return [np.random.uniform(low=low, high = high) for i in range(number)]
    else:
        return np.random.uniform(low=low, high = high, size = (number, ))


def min_distance(point, point_list):

    if len(point_list) == 0:
        return inf
    else:
        dis_list = [distance(p, point) for p in point_list] 
        return min(dis_list) 
    

def distance(point1, point2):
    return sqrt(  (point1[0, 0] - point2[0, 0]) **2 + (point1[1, 0] - point2[1, 0]) ** 2 )  

    
def WrapToPi(rad):
    # transform the rad to the range [-pi, pi]
    while rad > pi:
        rad = rad - 2 * pi
    
    while rad < -pi:
        rad = rad + 2 * pi
    
    return rad

def WrapToRegion(rad, range):
    # transform the rad to defined range, 
    # the length of range should be 2 * pi
    assert(len(range) >= 2 and range[1] - range[0] == 2*pi)
    while rad > range[1]:
        rad = rad - 2 * pi
    
    while rad < range[0]:
        rad = rad + 2 * pi
    
    return rad
    
def get_transform(state):
    # from state to rotation and transition matrix
    # state: (3, 1) or (2 ,1)
    if state.shape == (2, 1):
        rot = np.array([ [1, 0], [0, 1] ])
        trans = state[0:2]

    else:
        rot = np.array([ [cos(state[2, 0]), -sin(state[2, 0])], [sin(state[2, 0]), cos(state[2, 0])] ])
        trans = state[0:2]

    return trans, rot 

def repeat_mk_dirs(path, max_num):

    if not os.path.exists(path):
        os.makedirs(path)

        return path

    else: 
        if len(os.listdir(path)) == 0: # empty dir
            return path
        else:
            i = 0
            while i < max_num:
                new_path = path + '_' + str(i)
                i = i + 1
                if not os.path.exists(new_path):
                    break
    
            os.makedirs(new_path)

            return new_path



