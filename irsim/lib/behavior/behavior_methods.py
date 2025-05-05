from irsim.lib import register_behavior
from irsim.lib import reciprocal_vel_obs
from irsim.util.util import relative_position, WrapToPi, omni_to_diff
import numpy as np
from math import cos, sin


@register_behavior("diff", "rvo")
def beh_diff_rvo(ego_object, external_objects, **kwargs):

    rvo_neighbor = [obj.rvo_neighbor_state for obj in external_objects]
    rvo_state = ego_object.rvo_state
    vxmax = kwargs.get("vxmax", 1.5)
    vymax = kwargs.get("vymax", 1.5)
    acceler = kwargs.get("acceler", 1.0)
    factor = kwargs.get("factor", 1.0)
    mode = kwargs.get("mode", "rvo")
    neighbor_threshold = kwargs.get("neighbor_threshold", 3.0)
    behavior_vel = DiffRVO(rvo_state, rvo_neighbor, vxmax, vymax, acceler, factor, mode, neighbor_threshold)

    return behavior_vel


@register_behavior("diff", "dash")
def beh_diff_dash(ego_object, external_objects, **kwargs):

    state = ego_object.state
    goal = ego_object.goal
    goal_threshold = ego_object.goal_threshold
    _, max_vel = ego_object.get_vel_range()
    angle_tolerance = kwargs.get("angle_tolerance", 0.1)

    behavior_vel = DiffDash(state, goal, max_vel, goal_threshold, angle_tolerance)

    return behavior_vel


@register_behavior("omni", "dash")
def beh_omni_dash(ego_object, external_objects, **kwargs):

    state = ego_object.state
    goal = ego_object.goal
    goal_threshold = ego_object.goal_threshold
    _, max_vel = ego_object.get_vel_range()
    behavior_vel = OmniDash(state, goal, max_vel, goal_threshold)

    return behavior_vel


@register_behavior("omni", "rvo")
def beh_omni_rvo(ego_object, external_objects, **kwargs):

    rvo_neighbor = [obj.rvo_neighbor_state for obj in external_objects]
    rvo_state = ego_object.rvo_state
    vxmax = kwargs.get("vxmax", 1.5)
    vymax = kwargs.get("vymax", 1.5)
    acceler = kwargs.get("acceler", 1.0)
    factor = kwargs.get("factor", 1.0)
    mode = kwargs.get("mode", "rvo")
    neighbor_threshold = kwargs.get("neighbor_threshold", 3.0)
    behavior_vel = OmniRVO(rvo_state, rvo_neighbor, vxmax, vymax, acceler, factor, mode, neighbor_threshold)

    return behavior_vel


@register_behavior("acker", "dash")
def beh_acker_dash(ego_object, external_objects, **kwargs):

    state = ego_object.state
    goal = ego_object.goal
    goal_threshold = ego_object.goal_threshold
    _, max_vel = ego_object.get_vel_range()
    angle_tolerance = kwargs.get("angle_tolerance", 0.1)

    behavior_vel = AckerDash(state, goal, max_vel, goal_threshold, angle_tolerance)

    return behavior_vel


def OmniRVO(
    state_tuple,
    neighbor_list=None,
    vxmax=1.5,
    vymax=1.5,
    acceler=1,
    factor=1.0,
    mode="rvo",
    neighbor_threshold=3.0,
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
    
    x, y = state_tuple[0], state_tuple[1]

    filtered_neighbor_list = [neighbor for neighbor in neighbor_list if (x - neighbor[0])**2 + (y - neighbor[1])**2 < neighbor_threshold**2]

    rvo_behavior = reciprocal_vel_obs(
        state_tuple, filtered_neighbor_list, vxmax, vymax, acceler, factor
    )
    rvo_vel = rvo_behavior.cal_vel(mode)

    return np.array([[rvo_vel[0]], [rvo_vel[1]]])


def DiffRVO(
    state_tuple,
    neighbor_list=None,
    vxmax=1.5,
    vymax=1.5,
    acceler=1,
    factor=1.0,
    mode="rvo",
    neighbor_threshold=3.0,
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

    x, y = state_tuple[0], state_tuple[1]

    filtered_neighbor_list = [neighbor for neighbor in neighbor_list if (x - neighbor[0])**2 + (y - neighbor[1])**2 < neighbor_threshold**2]

    rvo_behavior = reciprocal_vel_obs(
        state_tuple, filtered_neighbor_list, vxmax, vymax, acceler, factor
    )
    rvo_vel = rvo_behavior.cal_vel(mode)
    rvo_vel_diff = omni_to_diff(state_tuple[-1], rvo_vel)

    return rvo_vel_diff


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


def DiffDash(state, goal, max_vel, goal_threshold=0.1, angle_tolerance=0.2):
    """
    Calculate the differential drive velocity to reach a goal.

    Args:
        state (np.array): Current state [x, y, theta] (3x1).
        goal (np.array): Goal position [x, y, theta] (3x1).
        max_vel (np.array): Maximum velocity [linear, angular] (2x1).
        goal_threshold (float): Distance threshold to consider goal reached (default 0.1).
        angle_tolerance (float): Allowable angular deviation (default 0.2).

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


def AckerDash(state, goal, max_vel, goal_threshold, angle_tolerance):
    """
    Calculate the Ackermann steering velocity to reach a goal.

    Args:
        state (np.array): Current state [x, y, theta] (3x1).
        goal (np.array): Goal position [x, y, theta] (3x1).
        max_vel (np.array): Maximum velocity [linear, steering angle] (2x1).
        goal_threshold (float): Distance threshold to consider goal reached.
        angle_tolerance (float): Allowable angular deviation.

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
