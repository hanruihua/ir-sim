import numpy as np
from math import cos, sin, tan
from irsim.util.util import WrapToPi
# from transforms3d import euler


# reference: Lynch, Kevin M., and Frank C. Park. Modern Robotics: Mechanics, Planning, and Control. 1st ed. Cambridge, MA: Cambridge University Press, 2017.


def differential_kinematics(
    state, velocity, step_time, noise=False, alpha=[0.03, 0, 0, 0.03]
):
    """
    Calculate the next state for a differential wheel robot.

    Args:
        state: A 3x1 vector [x, y, theta] representing the current position and orientation.
        velocity: A 2x1 vector [linear, angular] representing the current velocities.
        step_time: The time step for the simulation.
        noise: Boolean indicating whether to add noise to the velocity (default False).
        alpha: List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]). alpha[0] and alpha[1] are for linear velocity, alpha[2] and alpha[3] are for angular velocity.

    Returns:
        next_state: A 3x1 vector [x, y, theta] representing the next state.
    """
    assert state.shape[0] >= 3 and velocity.shape[0] >= 2

    if noise:
        assert len(alpha) >= 4
        std_linear = np.sqrt(
            alpha[0] * (velocity[0, 0] ** 2) + alpha[1] * (velocity[1, 0] ** 2)
        )
        std_angular = np.sqrt(
            alpha[2] * (velocity[0, 0] ** 2) + alpha[3] * (velocity[1, 0] ** 2)
        )
        real_velocity = velocity + np.random.normal(
            [[0], [0]], scale=[[std_linear], [std_angular]]
        )
    else:
        real_velocity = velocity

    phi = state[2, 0]
    co_matrix = np.array([[cos(phi), 0], [sin(phi), 0], [0, 1]])
    next_state = state[0:3] + co_matrix @ real_velocity * step_time
    next_state[2, 0] = WrapToPi(next_state[2, 0])

    return next_state


def ackermann_kinematics(
    state,
    velocity,
    step_time,
    noise=False,
    alpha=[0.03, 0, 0, 0.03],
    mode="steer",
    wheelbase=1,
):
    """
    Calculate the next state for an Ackermann steering vehicle.

    Args:
        state: A 4x1 vector [x, y, theta, steer_angle] representing the current state.
        velocity: A 2x1 vector representing the current velocities, format depends on mode.
            For "steer" mode, [linear, steer_angle] is expected.
            For "angular" mode, [linear, angular] is expected.

        step_time: The time step for the simulation.
        noise: Boolean indicating whether to add noise to the velocity (default False).
        alpha: List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]). alpha[0] and alpha[1] are for linear velocity, alpha[2] and alpha[3] are for angular velocity.
        mode: The kinematic mode, either "steer" or "angular" (default "steer").
        wheelbase: The distance between the front and rear axles (default 1).

    Returns:
        new_state: A 4x1 vector representing the next state.
    """
    assert state.shape[0] >= 4 and velocity.shape[0] >= 2

    phi = state[2, 0]
    psi = state[3, 0]

    if noise:
        assert len(alpha) >= 4
        std_linear = np.sqrt(
            alpha[0] * (velocity[0, 0] ** 2) + alpha[1] * (velocity[1, 0] ** 2)
        )
        std_angular = np.sqrt(
            alpha[2] * (velocity[0, 0] ** 2) + alpha[3] * (velocity[1, 0] ** 2)
        )
        real_velocity = velocity + np.random.normal(
            [[0], [0]], scale=[[std_linear], [std_angular]]
        )
    else:
        real_velocity = velocity

    if mode == "steer":
        co_matrix = np.array(
            [[cos(phi), 0], [sin(phi), 0], [tan(psi) / wheelbase, 0], [0, 1]]
        )
    elif mode == "angular":
        co_matrix = np.array(
            [[cos(phi), 0], [sin(phi), 0], [tan(psi) / wheelbase, 0], [0, 1]]
        )

    d_state = co_matrix @ real_velocity
    new_state = state + d_state * step_time

    if mode == "steer":
        new_state[3, 0] = real_velocity[1, 0]

    new_state[2, 0] = WrapToPi(new_state[2, 0])

    return new_state


def omni_kinematics(state, velocity, step_time, noise=False, alpha=[0.03, 0, 0, 0.03]):
    """
    Calculate the next position for an omnidirectional robot.

    Args:
        state: A 2x1 vector [x, y] representing the current position.
        velocity: A 2x1 vector [vx, vy] representing the current velocities.
        step_time: The time step for the simulation.
        noise: Boolean indicating whether to add noise to the velocity (default False).
        alpha: List of noise parameters for the velocity model (default [0.03, 0.03]). alpha[0] is for x velocity, alpha[1] is for y velocity.

    Returns:
        new_position: A 2x1 vector [x, y] representing the next position.
    """

    assert velocity.shape[0] >= 2 and state.shape[0] >= 2

    if noise:
        assert len(alpha) >= 2
        std_vx = np.sqrt(alpha[0])
        std_vy = np.sqrt(alpha[-1])
        real_velocity = velocity + np.random.normal(
            [[0], [0]], scale=[[std_vx], [std_vy]]
        )
    else:
        real_velocity = velocity

    new_position = state[0:2] + real_velocity * step_time

    return new_position


# def rigid3d_kinematics(state, velocity, step_time, noise, alpha):

#     assert velocity.shape[0] >= 6 and state.shape[0] >= 6

#     state_HT = state_to_homo_trans(state[:3], state[3:])
#     vel_HT = twist_to_homo_trans(velocity, step_time)
    
#     if noise:
#         print("Noise is not supported for rigid3d kinematics Now.")
    


# def state_to_homo_trans(position, euler_angles):
#     """
#     Create a homogeneous transformation matrix from state (position and Euler angles). 
    
#     Parameters:
#     - position: List or array of [x, y, z] coordinates.
#     - euler_angles: List or array of [roll, pitch, yaw] in radians.
    
#     Returns:
#     - 4x4 Homogeneous transformation matrix. (SE3)
#     """
#     # Convert Euler angles to rotation matrix
#     R = euler.euler2mat(*euler_angles, axes='sxyz')  # 'sxyz' specifies the axis order
#     T = np.identity(4)
#     T[:3, :3] = R
#     T[:3, 3] = position

#     return T


# def twist_to_homo_trans(twist, dt):
#     """
#     Convert a twist (linear and angular velocity) to an incremental transformation matrix.
    
#     Parameters:
#     - twist: List or array of [v_x, v_y, v_z, omega_x, omega_y, omega_z]
#     - dt: Time step duration (seconds)
    
#     Returns:
#     - 4x4 Incremental transformation matrix.
#     """
#     v = np.array(twist[:3]) * dt  # Linear displacement
#     omega = np.array(twist[3:]) * dt  # Angular displacement
#     theta = np.linalg.norm(omega)
    
#     if theta < 1e-6:
#         # No rotation
#         R = np.identity(3)
#         t = v
#     else:
#         # Normalize rotation axis
#         axis = omega / theta
#         R = axangles.axangle2mat(axis, theta)
#         t = v  # Assuming small angles where rotation and translation commute for simplicity
    
#     T_inc = np.identity(4)
#     T_inc[:3, :3] = R
#     T_inc[:3, 3] = t
#     return T_inc





    


    
    
