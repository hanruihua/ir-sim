import numpy as np
from math import cos, sin, tan
from irsim.util.util import WrapToPi

def differential_wheel_kinematics(state, velocity, step_time, noise=False, alpha = [0.03, 0, 0, 0.03, 0, 0]):

    '''
    The kinematics function for differential wheel robot

    state: [x, y, theta]   (3*1) vector
    velocity: [linear, angular]  (2*1) vector
    '''

    assert state.shape == (3, 1) and velocity.shape==(2, 1)

    if noise:
        std_linear = np.sqrt(alpha[0] * (velocity[0, 0] ** 2) + alpha[1] * (velocity[1, 0] ** 2))
        std_angular = np.sqrt(alpha[2] * (velocity[0, 0] ** 2) + alpha[3] * (velocity[1, 0] ** 2))
        # gamma = alpha[4] * (velocity[0, 0] ** 2) + alpha[5] * (velocity[1, 0] ** 2)
        real_velocity = velocity + np.random.normal([[0], [0]], scale = [[std_linear], [std_angular]])  

    else:
        real_velocity = velocity

    coefficient_vel = np.zeros((3, 2))
    coefficient_vel[0, 0] = cos(state[2, 0])
    coefficient_vel[1, 0] = sin(state[2, 0])
    coefficient_vel[2, 1] = 1

    next_state = state + coefficient_vel @ real_velocity * step_time

    next_state[2, 0] = WrapToPi(next_state[2, 0])

    return next_state

def ackermann_kinematics(state, velocity, step_time, mode, wheelbase, psi_limit, noise=False):

    # reference: Lynch, Kevin M., and Frank C. Park. Modern Robotics: Mechanics, Planning, and Control. 1st ed. Cambridge, MA: Cambridge University Press, 2017.

    phi = state[2, 0]
    psi = state[3, 0]

    if mode == 'steer':
        co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / wheelbase, 0], [0, 1] ])

        velocity[1, 0] = np.clip(velocity[1, 0], -psi_limit, psi_limit)

    elif mode == 'angular':

        co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / wheelbase, 0], [0, 1] ])

    elif mode == 'simplify':
        co_matrix = np.array([ [cos(phi), 0],  [sin(phi), 0], [tan(psi) / wheelbase, 0], [0, 1] ])

    d_state = co_matrix @ velocity
    new_state = state + d_state * step_time
    
    if mode == 'steer': new_state[3, 0] = velocity[1, 0]

    new_state[2, 0] = WrapToPi(new_state[2, 0]) 

    return new_state


