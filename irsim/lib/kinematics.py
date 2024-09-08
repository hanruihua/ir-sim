import numpy as np
from math import cos, sin, tan
from irsim.util.util import WrapToPi

def differential_kinematics(state, velocity, step_time, noise=False, alpha = [0.03, 0, 0, 0.03]):

    '''
    The kinematics function for differential wheel robot

    state: [x, y, theta]   (3*1) vector
    velocity: [linear, angular]  (2*1) vector
    '''

    assert state.shape[0] >= 3 and velocity.shape[0] >= 2

    if noise:

        assert len(alpha) >= 4

        std_linear = np.sqrt(alpha[0] * (velocity[0, 0] ** 2) + alpha[1] * (velocity[1, 0] ** 2))
        std_angular = np.sqrt(alpha[2] * (velocity[0, 0] ** 2) + alpha[3] * (velocity[1, 0] ** 2))
        # gamma = alpha[4] * (velocity[0, 0] ** 2) + alpha[5] * (velocity[1, 0] ** 2)
        real_velocity = velocity + np.random.normal([[0], [0]], scale = [[std_linear], [std_angular]])  

    else:
        real_velocity = velocity

    phi = state[2, 0]
    co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [0, 1] ])
    # coefficient_vel = np.zeros((3, 2))
    # coefficient_vel[0, 0] = cos(state[2, 0])
    # coefficient_vel[1, 0] = sin(state[2, 0])
    # coefficient_vel[2, 1] = 1
    next_state = state[0:3] + co_matrix @ real_velocity * step_time

    next_state[2, 0] = WrapToPi(next_state[2, 0])

    return next_state



def ackermann_kinematics(state, velocity, step_time, noise=False, alpha = [0.03, 0, 0, 0.03], mode='steer', wheelbase=1):

    # reference: Lynch, Kevin M., and Frank C. Park. Modern Robotics: Mechanics, Planning, and Control. 1st ed. Cambridge, MA: Cambridge University Press, 2017.

    assert state.shape[0] >= 4 and velocity.shape[0] >= 2

    phi = state[2, 0]
    psi = state[3, 0]

    if noise:

        assert len(alpha) >= 4

        std_linear = np.sqrt(alpha[0] * (velocity[0, 0] ** 2) + alpha[1] * (velocity[1, 0] ** 2))
        std_angular = np.sqrt(alpha[2] * (velocity[0, 0] ** 2) + alpha[3] * (velocity[1, 0] ** 2))
        # gamma = alpha[4] * (velocity[0, 0] ** 2) + alpha[5] * (velocity[1, 0] ** 2)
        real_velocity = velocity + np.random.normal([[0], [0]], scale = [[std_linear], [std_angular]])  
    else:
        real_velocity = velocity

    if mode == 'steer':
        # velocity = [linear_speed, steer_angle]
        co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / wheelbase, 0], [0, 1] ])

    elif mode == 'angular':
        # velocity = [linear_speed, angular_speed of steering]
        co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / wheelbase, 0], [0, 1] ])

    # elif mode == 'simplify':
    #     # velocity = [linear_speed, angular_speed of car]
    #     co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [0, 1] ])
        
    d_state = co_matrix @ real_velocity
    new_state = state + d_state * step_time
    
    if mode == 'steer': 
        new_state[3, 0] = real_velocity[1, 0]

    new_state[2, 0] = WrapToPi(new_state[2, 0]) 

    return new_state


def omni_kinematics(state, velocity, step_time, noise=False, alpha = [0.03, 0.03]):

    '''
    state: [x, y]   (2*1) vector
    velocity: [vx, vy]  (2*1) vector
    '''    

    assert velocity.shape[0]>=2 and state.shape[0] >= 2

    if noise:

        assert len(alpha) >= 2

        std_vx = np.sqrt(alpha[0])
        std_vy = np.sqrt(alpha[1])
        # real_velocity = velocity + np.random.normal([[0], [0], [0]], scale = [[std_vx], [std_vy], [0]])  
        real_velocity = velocity + np.random.normal([[0], [0]], scale = [[std_vx], [std_vy]])  

    else:
        real_velocity = velocity

    new_position = state[0:2] + real_velocity * step_time

    return new_position


