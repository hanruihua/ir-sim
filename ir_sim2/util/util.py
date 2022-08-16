import numpy as np
from math import sqrt, inf

def random_points(number, low, high, point_distance, max_iter=500):
    # random distribute some points into a selected area with a minimum distance
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

def min_distance(point, point_list):

    if len(point_list) == 0:
        return inf
    else:
        dis_list = [distance(p, point) for p in point_list] 
        return min(dis_list) 
    

def distance(point1, point2):
    return sqrt(  (point1[0, 0] - point2[0, 0]) **2 + (point1[1, 0] - point2[1, 0]) ** 2 )  


    


