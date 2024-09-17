from irsim.util.util import relative_position, WrapToPi, omni_to_diff
import numpy as np
from math import atan2, asin, cos, sin
from irsim.lib.algorithm.rvo import reciprocal_vel_obs


def DiffDash(state, goal, max_vel, angle_tolerance=0.2, goal_threshold=0.1):
    """
    Calculate the differential drive velocity to reach a goal.

    Args:
        state (np.array): Current state [x, y, theta] (3x1).
        goal (np.array): Goal position [x, y, theta] (3x1).
        max_vel (np.array): Maximum velocity [linear, angular] (2x1).
        angle_tolerance (float): Allowable angular deviation (default 0.2).
        goal_threshold (float): Distance threshold to consider goal reached (default 0.1).

    Returns:
        np.array: Velocity [linear, angular] (2x1).
    """
    distance, radian = relative_position(state, goal)

    if distance < goal_threshold:
        return np.zeros((2, 1))

    diff_radian = WrapToPi(radian - state[2, 0])
    linear = max_vel[0, 0] * np.cos(diff_radian)

    if abs(diff_radian) < angle_tolerance:
        angular = 0
    else:
        angular = max_vel[1, 0] * np.sign(diff_radian)

    return np.array([[linear], [angular]])


def AckerDash(state, goal, max_vel, angle_tolerance, goal_threshold):
    """
    Calculate the Ackermann steering velocity to reach a goal.

    Args:
        state (np.array): Current state [x, y, theta] (3x1).
        goal (np.array): Goal position [x, y, theta] (3x1).
        max_vel (np.array): Maximum velocity [linear, steering angle] (2x1).
        angle_tolerance (float): Allowable angular deviation.
        goal_threshold (float): Distance threshold to consider goal reached.

    Returns:
        np.array: Velocity [linear, steering angle] (2x1).
    """
    dis, radian = relative_position(state, goal)
    steer_opt = 0.0
    diff_radian = WrapToPi(radian - state[2, 0])

    if diff_radian > -angle_tolerance and diff_radian < angle_tolerance:
        diff_radian = 0

    if dis < goal_threshold:
        v_opt, steer_opt = 0, 0
    else:
        v_opt = max_vel[0, 0]
        steer_opt = np.clip(diff_radian, -max_vel[1, 0], max_vel[1, 0])

    return np.array([[v_opt], [steer_opt]])


def OmniDash(state, goal, max_vel, goal_threshold=0.1):
    """
    Calculate the omnidirectional velocity to reach a goal.

    Args:
        state (np.array): Current state [x, y] (2x1).
        goal (np.array): Goal position [x, y] (2x1).
        max_vel (np.array): Maximum velocity [vx, vy] (2x1).
        goal_threshold (float): Distance threshold to consider goal reached (default 0.1).

    Returns:
        np.array: Velocity [vx, vy] (2x1).
    """
    distance, radian = relative_position(state, goal)

    if distance > goal_threshold:
        vx = max_vel[0, 0] * cos(radian)
        vy = max_vel[1, 0] * sin(radian)
    else:
        vx = 0
        vy = 0

    return np.array([[vx], [vy]])


def DiffRVO(
    state_tuple,
    neighbor_list=None,
    vxmax=1.5,
    vymax=1.5,
    acceler=1,
    factor=1.0,
    mode="rvo",
):
    """
    Calculate the differential drive velocity using RVO.

    Args:
        state_tuple (tuple): Current state and orientation.
        neighbor_list (list): List of neighboring agents (default None).
        vxmax (float): Maximum x velocity (default 1.5).
        vymax (float): Maximum y velocity (default 1.5).
        acceler (float): Acceleration factor (default 1).
        factor (float): Additional scaling factor (default 1.0).
        mode (str): RVO calculation mode (default "rvo").

    Returns:
        np.array: Velocity [linear, angular] (2x1).
    """
    if neighbor_list is None:
        neighbor_list = []

    rvo_behavior = reciprocal_vel_obs(
        state_tuple, neighbor_list, vxmax, vymax, acceler, factor
    )
    rvo_vel = rvo_behavior.cal_vel(mode)
    rvo_vel_diff = omni_to_diff(state_tuple[-1], rvo_vel)

    return rvo_vel_diff


def OmniRVO(
    state_tuple,
    neighbor_list=None,
    vxmax=1.5,
    vymax=1.5,
    acceler=1,
    factor=1.0,
    mode="rvo",
):
    """
    Calculate the omnidirectional velocity using RVO.

    Args:
        state_tuple (tuple): Current state and orientation.
        neighbor_list (list): List of neighboring agents (default None).
        vxmax (float): Maximum x velocity (default 1.5).
        vymax (float): Maximum y velocity (default 1.5).
        acceler (float): Acceleration factor (default 1).
        factor (float): Additional scaling factor (default 1.0).
        mode (str): RVO calculation mode (default "rvo").

    Returns:
        np.array: Velocity [vx, vy] (2x1).
    """
    if neighbor_list is None:
        neighbor_list = []

    rvo_behavior = reciprocal_vel_obs(
        state_tuple, neighbor_list, vxmax, vymax, acceler, factor
    )
    rvo_vel = rvo_behavior.cal_vel(mode)

    return np.array([[rvo_vel[0]], [rvo_vel[1]]])
