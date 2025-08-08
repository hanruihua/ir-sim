from math import cos, sin
from typing import Any, Optional

import numpy as np

from irsim.config import env_param, world_param
from irsim.lib import reciprocal_vel_obs, register_behavior
from irsim.util.util import WrapToPi, omni_to_diff, relative_position

"""
Behavior Methods Module

This module contains behavior functions for different types of robots in the IR-Sim environment.
It provides implementations for:

1. Registered behavior functions (decorated with @register_behavior):
   - beh_diff_rvo: Differential drive robot with RVO behavior
   - beh_diff_dash: Differential drive robot with dash-to-goal behavior
   - beh_omni_dash: Omnidirectional robot with dash-to-goal behavior
   - beh_omni_rvo: Omnidirectional robot with RVO behavior
   - beh_acker_dash: Ackermann steering robot with dash-to-goal behavior

2. Core behavior calculation functions:
   - OmniRVO: Omnidirectional RVO velocity calculation
   - DiffRVO: Differential drive RVO velocity calculation
   - OmniDash: Omnidirectional dash-to-goal velocity calculation
   - DiffDash: Differential drive dash-to-goal velocity calculation
   - AckerDash: Ackermann steering dash-to-goal velocity calculation

These functions are designed to be used with the robot behavior system to control
robot movements in various scenarios including collision avoidance (RVO) and
goal-reaching behaviors (dash).

"""


@register_behavior("diff", "rvo")
def beh_diff_rvo(
    ego_object: Any, external_objects: list[Any], **kwargs: Any
) -> np.ndarray:
    """
    Behavior function for differential drive robot using RVO (Reciprocal Velocity Obstacles).

    Args:
        ego_object: The ego robot object.
        external_objects (list): List of external objects in the environment.
        **kwargs: Additional keyword arguments:
            - vxmax (float): Maximum x velocity, default 1.5.
            - vymax (float): Maximum y velocity, default 1.5.
            - acce (float): Acceleration factor, default 1.0.
            - factor (float): Additional scaling factor, default 1.0.
            - mode (str): RVO calculation mode, default "rvo".
            - neighbor_threshold (float): Neighbor threshold distance, default 10.0.

    Returns:
        np.array: Velocity [linear, angular] (2x1) for differential drive.
    """

    if ego_object.goal is None:
        if world_param.count % 10 == 0:
            env_param.logger.warning(
                "Goal is currently None. This rvo behavior is waiting for goal configuration"
            )
        return np.zeros((2, 1))

    rvo_neighbor = [obj.rvo_neighbor_state for obj in external_objects]
    rvo_state = ego_object.rvo_state
    vxmax = kwargs.get("vxmax", 1.5)
    vymax = kwargs.get("vymax", 1.5)
    acce = kwargs.get("acce", 1.0)
    factor = kwargs.get("factor", 1.0)
    mode = kwargs.get("mode", "rvo")
    neighbor_threshold = kwargs.get("neighbor_threshold", 10.0)
    return DiffRVO(
        rvo_state, rvo_neighbor, vxmax, vymax, acce, factor, mode, neighbor_threshold
    )


@register_behavior("diff", "dash")
def beh_diff_dash(
    ego_object: Any, external_objects: list[Any], **kwargs: Any
) -> np.ndarray:
    """
    Behavior function for differential drive robot using dash-to-goal behavior.

    Args:
        ego_object: The ego robot object.
        external_objects (list): List of external objects in the environment.
        **kwargs: Additional keyword arguments:
            - angle_tolerance (float): Allowable angular deviation, default 0.1.

    Returns:
        np.array: Velocity [linear, angular] (2x1) for differential drive.
    """

    state = ego_object.state
    goal = ego_object.goal
    goal_threshold = ego_object.goal_threshold
    _, max_vel = ego_object.get_vel_range()
    angle_tolerance = kwargs.get("angle_tolerance", 0.1)

    if goal is None:
        if world_param.count % 10 == 0:
            env_param.logger.warning(
                "Goal is currently None. This dash behavior is waiting for goal configuration"
            )

        return np.zeros((2, 1))

    return DiffDash(state, goal, max_vel, goal_threshold, angle_tolerance)


@register_behavior("omni", "dash")
def beh_omni_dash(
    ego_object: Any, external_objects: list[Any], **kwargs: Any
) -> np.ndarray:
    """
    Behavior function for omnidirectional robot using dash-to-goal behavior.

    Args:
        ego_object: The ego robot object.
        external_objects (list): List of external objects in the environment.
        **kwargs: Additional keyword arguments (currently unused).

    Returns:
        np.array: Velocity [vx, vy] (2x1) for omnidirectional drive.
    """

    if ego_object.goal is None:
        if world_param.count % 10 == 0:
            env_param.logger.warning(
                "Goal is currently None. This dash behavior is waiting for goal configuration"
            )
        return np.zeros((2, 1))

    state = ego_object.state
    goal = ego_object.goal
    goal_threshold = ego_object.goal_threshold
    _, max_vel = ego_object.get_vel_range()
    return OmniDash(state, goal, max_vel, goal_threshold)


@register_behavior("omni", "rvo")
def beh_omni_rvo(
    ego_object: Any, external_objects: list[Any], **kwargs: Any
) -> np.ndarray:
    """
    Behavior function for omnidirectional robot using RVO (Reciprocal Velocity Obstacles).

    Args:
        ego_object: The ego robot object.
        external_objects (list): List of external objects in the environment.
        **kwargs: Additional keyword arguments:
            - vxmax (float): Maximum x velocity, default 1.5.
            - vymax (float): Maximum y velocity, default 1.5.
            - acce (float): Acceleration factor, default 1.0.
            - factor (float): Additional scaling factor, default 1.0.
            - mode (str): RVO calculation mode, default "rvo".
            - neighbor_threshold (float): Neighbor threshold distance, default 3.0.

    Returns:
        np.array: Velocity [vx, vy] (2x1) for omnidirectional drive.
    """

    if ego_object.goal is None:
        if world_param.count % 10 == 0:
            env_param.logger.warning(
                "Goal is currently None. This rvo behavior is waiting for goal configuration"
            )
        return np.zeros((2, 1))

    rvo_neighbor = [obj.rvo_neighbor_state for obj in external_objects]
    rvo_state = ego_object.rvo_state
    vxmax = kwargs.get("vxmax", 1.5)
    vymax = kwargs.get("vymax", 1.5)
    acce = kwargs.get("acce", 1.0)
    factor = kwargs.get("factor", 1.0)
    mode = kwargs.get("mode", "rvo")
    neighbor_threshold = kwargs.get("neighbor_threshold", 3.0)
    return OmniRVO(
        rvo_state, rvo_neighbor, vxmax, vymax, acce, factor, mode, neighbor_threshold
    )


@register_behavior("acker", "dash")
def beh_acker_dash(
    ego_object: Any, external_objects: list[Any], **kwargs: Any
) -> np.ndarray:
    """
    Behavior function for Ackermann steering robot using dash-to-goal behavior.

    Args:
        ego_object: The ego robot object.
        external_objects (list): List of external objects in the environment.
        **kwargs: Additional keyword arguments:
            - angle_tolerance (float): Allowable angular deviation, default 0.1.

    Returns:
        np.array: Velocity [linear, steering angle] (2x1) for Ackermann drive.
    """

    if ego_object.goal is None:
        if world_param.count % 10 == 0:
            env_param.logger.warning(
                "Goal is currently None. This rvo behavior is waiting for goal configuration"
            )
        return np.zeros((2, 1))

    state = ego_object.state
    goal = ego_object.goal
    goal_threshold = ego_object.goal_threshold
    _, max_vel = ego_object.get_vel_range()
    angle_tolerance = kwargs.get("angle_tolerance", 0.1)

    return AckerDash(state, goal, max_vel, goal_threshold, angle_tolerance)


def OmniRVO(
    state_tuple: Any,
    neighbor_list: Optional[list[Any]] = None,
    vxmax: float = 1.5,
    vymax: float = 1.5,
    acce: float = 1,
    factor: float = 1.0,
    mode: str = "rvo",
    neighbor_threshold: float = 3.0,
) -> np.ndarray:
    """
    Calculate the omnidirectional velocity using RVO.

    Args:
        state_tuple (tuple): Current state and orientation.
        neighbor_list (list): List of neighboring agents (default None).
        vxmax (float): Maximum x velocity (default 1.5).
        vymax (float): Maximum y velocity (default 1.5).
        acce (float): Acceleration factor (default 1).
        factor (float): Additional scaling factor (default 1.0).
        mode (str): RVO calculation mode (default "rvo").
        neighbor_threshold (float): Neighbor threshold (default 3.0).

    Returns:
        np.array: Velocity [vx, vy] (2x1).
    """

    if neighbor_list is None:
        neighbor_list = []

    x, y = state_tuple[0], state_tuple[1]

    filtered_neighbor_list = [
        neighbor
        for neighbor in neighbor_list
        if (x - neighbor[0]) ** 2 + (y - neighbor[1]) ** 2 < neighbor_threshold**2
    ]

    rvo_behavior = reciprocal_vel_obs(
        state_tuple, filtered_neighbor_list, vxmax, vymax, acce, factor
    )
    rvo_vel = rvo_behavior.cal_vel(mode)

    return np.array([[rvo_vel[0]], [rvo_vel[1]]])


def DiffRVO(
    state_tuple: Any,
    neighbor_list: Optional[list[Any]] = None,
    vxmax: float = 1.5,
    vymax: float = 1.5,
    acce: float = 1,
    factor: float = 1.0,
    mode: str = "rvo",
    neighbor_threshold: float = 3.0,
) -> np.ndarray:
    """
    Calculate the differential drive velocity using RVO.

    Args:
        state_tuple (tuple): Current state and orientation.
        neighbor_list (list): List of neighboring agents (default None).
        vxmax (float): Maximum x velocity (default 1.5).
        vymax (float): Maximum y velocity (default 1.5).
        acce (float): Acceleration factor (default 1).
        factor (float): Additional scaling factor (default 1.0).
        mode (str): RVO calculation mode (default "rvo").
        neighbor_threshold (float): Neighbor threshold (default 3.0).

    Returns:
        np.array: Velocity [linear, angular] (2x1).
    """
    if neighbor_list is None:
        neighbor_list = []

    x, y = state_tuple[0], state_tuple[1]

    filtered_neighbor_list = [
        neighbor
        for neighbor in neighbor_list
        if (x - neighbor[0]) ** 2 + (y - neighbor[1]) ** 2 < neighbor_threshold**2
    ]

    rvo_behavior = reciprocal_vel_obs(
        state_tuple, filtered_neighbor_list, vxmax, vymax, acce, factor
    )
    rvo_vel = rvo_behavior.cal_vel(mode)
    return omni_to_diff(state_tuple[-1], rvo_vel)


def OmniDash(
    state: np.ndarray,
    goal: np.ndarray,
    max_vel: np.ndarray,
    goal_threshold: float = 0.1,
) -> np.ndarray:
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


def DiffDash(
    state: np.ndarray,
    goal: np.ndarray,
    max_vel: np.ndarray,
    goal_threshold: float = 0.1,
    angle_tolerance: float = 0.2,
) -> np.ndarray:
    """
    Calculate the differential drive velocity to reach a goal.

    Args:
        state (np.array): Current state [x, y, theta] (3x1).
        goal (np.array): Goal position [x, y] (2x1).
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


def AckerDash(
    state: np.ndarray,
    goal: np.ndarray,
    max_vel: np.ndarray,
    goal_threshold: float,
    angle_tolerance: float,
) -> np.ndarray:
    """
    Calculate the Ackermann steering velocity to reach a goal.

    Args:
        state (np.array): Current state [x, y, theta] (3x1).
        goal (np.array): Goal position [x, y] (2x1).
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
