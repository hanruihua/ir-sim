from ir_sim.util.util import relative_position, WrapToPi, omni_to_diff
import numpy as np
from math import atan2, asin, cos, sin


def DiffDash(state, goal, max_vel, angle_tolerance=0.2, goal_threshold=0.1):

    distance, radian = relative_position(state, goal) 

    if distance < goal_threshold:
        return np.zeros((2, 1))

    diff_radian = WrapToPi( radian - state[2, 0] )

    linear = max_vel[0, 0] * np.cos(diff_radian)

    if abs(diff_radian) < angle_tolerance:
        angular = 0
    else:
        angular = max_vel[1, 0] * np.sign(diff_radian)

    return np.array([[linear], [angular]])

def AckerDash(state, goal, max_vel, angle_tolerance, goal_threshold):

    dis, radian = relative_position(state, goal)

    steer_opt = 0.0

    diff_radian = WrapToPi( radian - state[2, 0] )
    
    if diff_radian > -angle_tolerance and diff_radian < angle_tolerance: diff_radian = 0

    if dis < goal_threshold:
        v_opt, steer_opt = 0, 0
    else:
        v_opt = max_vel[0, 0]
        steer_opt = np.clip(diff_radian, -max_vel[1, 0], max_vel[1, 0])   

    return np.array([[v_opt], [steer_opt]])


def OmniDash(state, goal, max_vel, goal_threshold=0.1):

    distance, radian = relative_position(state, goal) 
    # speed = np.sqrt( max_vel[0, 0] **2 + max_vel[1, 0] **2 )

    if distance > goal_threshold:
        vx = max_vel[0, 0] * cos(radian)
        vy = max_vel[1, 0] * sin(radian)
    else:
        vx = 0
        vy = 0

    return np.array([[vx], [vy]])


# rvo
def DiffRVO(state_tuple, neighbor_list=None, vxmax = 1.5, vymax = 1.5, acceler = 1, factor=1.0, mode='rvo'):

    # state_tuple: [x, y, vx, vy, radius, vx_des, vy_des, theta]
    # neighbor_list: a list of the tuple: [x, y, vx, vy, radius]
    # rvo_vel_diff = omni_to_diff(state_tuple[-1], [state_tuple[-3], state_tuple[-2]])
    # return rvo_vel_diff

    if neighbor_list is None: neighbor_list = [] 

    if mode=='rvo':
        rvo_list = config_rvo_list(state_tuple, neighbor_list)

    vo_outside, vo_inside = vel_candidate(state_tuple, rvo_list, acceler, vxmax, vymax)
    rvo_vel = vel_select(state_tuple, neighbor_list, vo_outside, vo_inside, factor)

    rvo_vel_diff = omni_to_diff(state_tuple[-1], rvo_vel)

    return rvo_vel_diff

def OmniRVO(state_tuple, neighbor_list=None, vxmax = 1.5, vymax = 1.5, acceler = 1, factor=1.0, mode='rvo'):

    # state_tuple: [x, y, vx, vy, radius, vx_des, vy_des, theta]
    # neighbor_list: a list of the tuple: [x, y, vx, vy, radius]
    # rvo_vel_diff = omni_to_diff(state_tuple[-1], [state_tuple[-3], state_tuple[-2]])
    # return rvo_vel_omni

    if neighbor_list is None: neighbor_list = [] 

    if mode=='rvo':
        rvo_list = config_rvo_list(state_tuple, neighbor_list)

    vo_outside, vo_inside = vel_candidate(state_tuple, rvo_list, acceler, vxmax, vymax)
    rvo_vel = vel_select(state_tuple, neighbor_list, vo_outside, vo_inside, factor)

    
    # rvo_vel_diff = omni_to_diff(state_tuple[-1], rvo_vel)

    return np.array([[rvo_vel[0]], [rvo_vel[1]]])


def config_rvo_list(state_tuple, neighbor_list):
    
    rvo_list = [config_rvo(state_tuple, n) for n in neighbor_list]

    return rvo_list

def config_rvo(state_tuple, obstacle):

    x, y, vx, vy, r, _, _, _ = state_tuple 
    
    mx, my, mvx, mvy, mr = obstacle        
    rvo_apex = [(vx + mvx)/2, (vy + mvy)/2]

    if mvx <= 0.001 and mvy <= 0.001:
        rvo_apex = [0, 0]

    dis_mr = np.sqrt((my - y)**2 + (mx - x)**2)
    angle_mr = atan2(my - y, mx - x)
    
    if dis_mr < r + mr: dis_mr = r + mr
        
    ratio = (r + mr)/dis_mr

    half_angle = asin( ratio ) 
    line_left_ori = angle_mr + half_angle
    line_right_ori = angle_mr - half_angle

    line_left_vector = [cos(line_left_ori), sin(line_left_ori)]
    line_right_vector = [cos(line_right_ori), sin(line_right_ori)]
    
    return [rvo_apex, line_left_vector, line_right_vector]


def vel_candidate(state_tuple, rvo_list, acceler, vxmax, vymax):
        
    vo_outside = []
    vo_inside = []

    cur_vx = state_tuple[2]
    cur_vy = state_tuple[3]

    cur_vx_min = max((cur_vx - acceler), -vxmax)
    cur_vx_max = min((cur_vx + acceler), vxmax)

    cur_vy_min = max((cur_vy - acceler), -vymax)
    cur_vy_max = min((cur_vy + acceler), vymax)

    for new_vx in np.arange(cur_vx_min, cur_vx_max, 0.05):
        for new_vy in np.arange(cur_vy_min, cur_vy_max, 0.05):

            if vo_out(new_vx, new_vy, rvo_list):
                vo_outside.append([new_vx, new_vy])

            else:
                vo_inside.append([new_vx, new_vy])

    return vo_outside, vo_inside


def vo_out(vx, vy, rvo_list):
        
    for rvo in rvo_list:
        rel_vx = vx - rvo[0][0]
        rel_vy = vy - rvo[0][1]

        rel_vector = [rel_vx, rel_vy]

        if between_vector(rvo[1], rvo[2], rel_vector):
            return False
    
    return True


def between_vector(line_left_vector, line_right_vector, line_vector):

    if cross_product(line_left_vector, line_vector) <= 0 and cross_product(line_right_vector, line_vector) >= 0:
        return True
    else:
        return False
    
def cross_product(vector1, vector2): 
    return float(vector1[0] * vector2[1] - vector2[0] * vector1[1])

def vel_select(state_tuple, neighbor_list, vo_outside, vo_inside, factor=1.0):

    vel_des = [state_tuple[5], state_tuple[6]]

    if (len(vo_outside) != 0):
        return min(vo_outside, key = lambda v: distance(v, vel_des))

    else:
        return min(vo_inside, key = lambda v: penalty(state_tuple, neighbor_list, v, vel_des, factor) )


def distance(point1, point2):
    return np.sqrt( (point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2 )
    

def penalty(state_tuple, neighbor_list, vel, vel_des, factor):
        
    tc_list = []

    for moving in neighbor_list:
        
        distance_temp = distance([moving[0], moving[1]], [state_tuple[0], state_tuple[1]])
        diff = distance_temp ** 2 - (state_tuple[4] + moving[4]) ** 2

        if diff < 0: diff = 0
            
        dis_vel = np.sqrt(diff)
        vel_trans = [ 2 * vel[0] - state_tuple[2] - moving[2],  2 * vel[1] - state_tuple[3] - moving[3] ] 
        vel_trans_speed = np.sqrt( vel_trans[0] ** 2 + vel_trans[1] ** 2 )

        tc = dis_vel / vel_trans_speed

        tc_list.append(tc)
        
    tc_min = min(tc_list)
    
    if tc_min == 0:
        tc_min = 0.0001

    penalty_vel = factor * (1/tc_min) + distance(vel_des, vel)

    return penalty_vel







