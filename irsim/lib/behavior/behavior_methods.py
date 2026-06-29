from math import cos, sin
from typing import Any

import numpy as np

from irsim.lib import reciprocal_vel_obs, register_behavior, social_force_model
from irsim.util.util import WrapToPi, omni_to_diff, relative_position

"""
Behavior Methods Module

This module contains behavior functions for different types of robots in the IR-Sim environment.
It provides implementations for:

1. Registered behavior functions (decorated with @register_behavior):
   - beh_diff_rvo: Differential drive robot with RVO behavior
   - beh_diff_dash: Differential drive robot with dash-to-goal behavior
   - beh_diff_sfm: Differential drive robot with Social Force Model behavior
   - beh_omni_dash: Omnidirectional robot with dash-to-goal behavior
   - beh_omni_rvo: Omnidirectional robot with RVO behavior
   - beh_omni_sfm: Omnidirectional robot with Social Force Model behavior
   - beh_acker_dash: Ackermann steering robot with dash-to-goal behavior

2. Core behavior calculation functions:
   - OmniRVO: Omnidirectional RVO velocity calculation
   - DiffRVO: Differential drive RVO velocity calculation
   - OmniDash: Omnidirectional dash-to-goal velocity calculation
   - DiffDash: Differential drive dash-to-goal velocity calculation
   - AckerDash: Ackermann steering dash-to-goal velocity calculation
   - SFMVelocity: Social Force Model velocity calculation

These functions are designed to be used with the robot behavior system to control
robot movements in various scenarios including collision avoidance (RVO) and
goal-reaching behaviors (dash).

See :mod:`irsim.lib.algorithm.social_force_model` for the SFM algorithmic
details.
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
        if ego_object._world_param.count % 10 == 0:
            ego_object.logger.warning(
                "Goal is currently None. This rvo behavior is waiting for goal configuration"
            )
        return np.zeros((2, 1))

    rvo_neighbor = []
    line_segments = []
    for obj in external_objects:
        segs = obj.rvo_line_segments
        if segs:
            line_segments.extend(segs)
        else:
            rvo_neighbor.append(obj.rvo_neighbor_state)
    rvo_state = ego_object.rvo_state
    vxmax = kwargs.get("vxmax", 1.5)
    vymax = kwargs.get("vymax", 1.5)
    acce = kwargs.get("acce", 1.0)
    factor = kwargs.get("factor", 1.0)
    mode = kwargs.get("mode", "rvo")
    neighbor_threshold = kwargs.get("neighbor_threshold", 3.0)
    return DiffRVO(
        rvo_state,
        rvo_neighbor,
        vxmax,
        vymax,
        acce,
        factor,
        mode,
        neighbor_threshold,
        line_segments=line_segments,
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
    max_vel = ego_object.vel_max
    angle_tolerance = kwargs.get("angle_tolerance", 0.1)
    angular_acce = ego_object.info.acce[1, 0]
    dt = ego_object._world_param.step_time

    if goal is None:
        if ego_object._world_param.count % 10 == 0:
            ego_object.logger.warning(
                "Goal is currently None. This dash behavior is waiting for goal configuration"
            )

        return np.zeros((2, 1))

    vel = DiffDash(
        state, goal, max_vel, goal_threshold, angle_tolerance, angular_acce, dt
    )
    min_step, max_step = ego_object.get_vel_range()
    return np.clip(vel, min_step, max_step)


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
        if ego_object._world_param.count % 10 == 0:
            ego_object.logger.warning(
                "Goal is currently None. This dash behavior is waiting for goal configuration"
            )
        return np.zeros((2, 1))

    state = ego_object.state
    goal = ego_object.goal
    goal_threshold = ego_object.goal_threshold
    max_vel = ego_object.vel_max
    vel = OmniDash(state, goal, max_vel, goal_threshold)
    min_step, max_step = ego_object.get_vel_range()
    return np.clip(vel, min_step, max_step)


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
        if ego_object._world_param.count % 10 == 0:
            ego_object.logger.warning(
                "Goal is currently None. This rvo behavior is waiting for goal configuration"
            )
        return np.zeros((2, 1))

    rvo_neighbor = []
    line_segments = []
    for obj in external_objects:
        segs = obj.rvo_line_segments
        if segs:
            line_segments.extend(segs)
        else:
            rvo_neighbor.append(obj.rvo_neighbor_state)
    rvo_state = ego_object.rvo_state
    vxmax = kwargs.get("vxmax", 1.5)
    vymax = kwargs.get("vymax", 1.5)
    acce = kwargs.get("acce", 1.0)
    factor = kwargs.get("factor", 1.0)
    mode = kwargs.get("mode", "rvo")
    neighbor_threshold = kwargs.get("neighbor_threshold", 3.0)
    return OmniRVO(
        rvo_state,
        rvo_neighbor,
        vxmax,
        vymax,
        acce,
        factor,
        mode,
        neighbor_threshold,
        line_segments=line_segments,
    )


@register_behavior("diff", "sfm")
def beh_diff_sfm(
    ego_object: Any, external_objects: list[Any], **kwargs: Any
) -> np.ndarray:
    """
    Behavior function for differential drive robot using the Social Force Model.

    Args:
        ego_object: The ego robot object.
        external_objects (list): List of external objects in the environment.
        **kwargs: SFM tuning parameters; see :func:`SFMVelocity`.

    Returns:
        np.array: Velocity [linear, angular] (2x1) for differential drive.
    """
    if ego_object.goal is None:
        if ego_object._world_param.count % 10 == 0:
            ego_object.logger.warning(
                "Goal is currently None. This sfm behavior is waiting for goal configuration"
            )
        return np.zeros((2, 1))

    neighbors = []
    line_segments = []
    for obj in external_objects:
        segs = obj.rvo_line_segments
        if segs:
            line_segments.extend(segs)
        else:
            neighbors.append(obj.rvo_neighbor_state)

    vx, vy = SFMVelocity(
        ego_object.rvo_state,
        neighbors,
        line_segments,
        step_time=ego_object._world_param.step_time,
        **kwargs,
    )
    # SFM produces a holonomic ``(vx, vy)``; track it tightly so the small
    # sideways component from anisotropic social/obstacle forces doesn't
    # get clipped by ``omni_to_diff``'s default deadband.
    _, vmax_pair = ego_object.get_vel_range()
    w_max = float(vmax_pair[1, 0])
    return omni_to_diff(
        ego_object.rvo_state[-1],
        [vx, vy],
        w_max=w_max,
        guarantee_time=ego_object._world_param.step_time,
        tolerance=0.02,
    )


@register_behavior("omni", "sfm")
def beh_omni_sfm(
    ego_object: Any, external_objects: list[Any], **kwargs: Any
) -> np.ndarray:
    """
    Behavior function for omnidirectional robot using the Social Force Model.

    Args:
        ego_object: The ego robot object.
        external_objects (list): List of external objects in the environment.
        **kwargs: SFM tuning parameters; see :func:`SFMVelocity`.

    Returns:
        np.array: Velocity [vx, vy] (2x1) for omnidirectional drive.
    """
    if ego_object.goal is None:
        if ego_object._world_param.count % 10 == 0:
            ego_object.logger.warning(
                "Goal is currently None. This sfm behavior is waiting for goal configuration"
            )
        return np.zeros((2, 1))

    neighbors = []
    line_segments = []
    for obj in external_objects:
        segs = obj.rvo_line_segments
        if segs:
            line_segments.extend(segs)
        else:
            neighbors.append(obj.rvo_neighbor_state)

    vx, vy = SFMVelocity(
        ego_object.rvo_state,
        neighbors,
        line_segments,
        step_time=ego_object._world_param.step_time,
        **kwargs,
    )
    return np.array([[vx], [vy]])


@register_behavior("omni_angular", "dash")
def beh_omni_angular_dash(
    ego_object: Any, external_objects: list[Any], **kwargs: Any
) -> np.ndarray:
    """
    Behavior function for omnidirectional-angular robot using dash-to-goal behavior.

    Args:
        ego_object: The ego robot object.
        external_objects (list): List of external objects in the environment.
        **kwargs: Additional keyword arguments:
            - angle_tolerance (float): Allowable angular deviation, default 0.1.

    Returns:
        np.array: Velocity [forward, lateral, yaw_rate] (3x1) in body frame.
    """

    if ego_object.goal is None:
        if ego_object._world_param.count % 10 == 0:
            ego_object.logger.warning(
                "Goal is currently None. This dash behavior is waiting for goal configuration"
            )
        return np.zeros((3, 1))

    state = ego_object.state
    goal = ego_object.goal
    goal_threshold = ego_object.goal_threshold
    max_vel = ego_object.vel_max
    angle_tolerance = kwargs.get("angle_tolerance", 0.1)
    acce = ego_object.info.acce
    angular_acce = acce[2, 0] if acce.shape[0] > 2 else float("inf")
    dt = ego_object._world_param.step_time
    vel = OmniAngularDash(
        state, goal, max_vel, goal_threshold, angle_tolerance, angular_acce, dt
    )
    min_step, max_step = ego_object.get_vel_range()
    return np.clip(vel, min_step, max_step)


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
        if ego_object._world_param.count % 10 == 0:
            ego_object.logger.warning(
                "Goal is currently None. This dash behavior is waiting for goal configuration"
            )
        return np.zeros((2, 1))

    state = ego_object.state
    goal = ego_object.goal
    goal_threshold = ego_object.goal_threshold
    max_vel = ego_object.vel_max
    angle_tolerance = kwargs.get("angle_tolerance", 0.1)
    vel = AckerDash(state, goal, max_vel, goal_threshold, angle_tolerance)
    min_step, max_step = ego_object.get_vel_range()
    return np.clip(vel, min_step, max_step)


def SFMVelocity(
    state_tuple: Any,
    neighbor_list: list[Any] | None = None,
    line_segments: list[list[float]] | None = None,
    vmax: float = 1.5,
    step_time: float = 0.1,
    neighbor_threshold: float = 10.0,
    relaxation_time: float = 0.5,
    force_factor_desired: float = 1.0,
    force_factor_social: float = 2.1,
    force_factor_obstacle: float = 10.0,
    sigma_obstacle: float = 0.8,
    lambda_importance: float = 2.0,
    gamma: float = 0.35,
    n_angular: float = 2.0,
    n_velocity: float = 3.0,
    safety_radius: float = 0.0,
    **_: Any,
) -> tuple[float, float]:
    """
    Compute the next world-frame velocity using the Social Force Model.

    Anisotropic Moussaid-Helbing (2009) variant.

    Args:
        state_tuple: Full RVO-style state
            ``[x, y, vx, vy, radius, vx_des, vy_des, theta]``.
        neighbor_list: Neighbour states ``[[x, y, vx, vy, radius], ...]``.
        line_segments: Line obstacles ``[[x1, y1, x2, y2], ...]``.
        vmax: Speed cap.
        step_time: Integration step.
        neighbor_threshold: Spatial cutoff for social interactions.
        relaxation_time, force_factor_*, sigma_obstacle, lambda_importance, gamma, n_angular, n_velocity:
            SFM tuning parameters (see :class:`social_force_model`).

    Returns:
        tuple[float, float]: Updated world-frame velocity ``(vx, vy)``.
    """
    if neighbor_list is None:
        neighbor_list = []
    if line_segments is None:
        line_segments = []

    x, y = state_tuple[0], state_tuple[1]
    filtered_neighbors = [
        nb
        for nb in neighbor_list
        if (x - nb[0]) ** 2 + (y - nb[1]) ** 2 < neighbor_threshold**2
    ]

    # ``rvo_state`` provides ``(vx_des, vy_des)`` with components scaled by
    # the kinematics-specific ``vel_max`` (anisotropic for diff/acker).
    # SFM expects ``desired_direction * vmax``, so renormalise here.
    state_tuple = list(state_tuple)
    vx_des, vy_des = state_tuple[5], state_tuple[6]
    norm = (vx_des * vx_des + vy_des * vy_des) ** 0.5
    if norm > 1e-9:
        state_tuple[5] = vmax * vx_des / norm
        state_tuple[6] = vmax * vy_des / norm

    sfm = social_force_model(
        state=state_tuple,
        neighbor_list=filtered_neighbors,
        line_obs_list=line_segments,
        vmax=vmax,
        step_time=step_time,
        relaxation_time=relaxation_time,
        force_factor_desired=force_factor_desired,
        force_factor_social=force_factor_social,
        force_factor_obstacle=force_factor_obstacle,
        sigma_obstacle=sigma_obstacle,
        lambda_importance=lambda_importance,
        gamma=gamma,
        n_angular=n_angular,
        n_velocity=n_velocity,
        neighbor_range=neighbor_threshold,
        safety_radius=safety_radius,
    )
    vx, vy = sfm.cal_vel()
    return vx, vy


def OmniRVO(
    state_tuple: Any,
    neighbor_list: list[Any] | None = None,
    vxmax: float = 1.5,
    vymax: float = 1.5,
    acce: float = 1,
    factor: float = 1.0,
    mode: str = "rvo",
    neighbor_threshold: float = 3.0,
    line_segments: list[list[float]] | None = None,
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
        line_segments (list): Line segments [[x1, y1, x2, y2], ...] (default None).

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
        state_tuple,
        filtered_neighbor_list,
        vxmax,
        vymax,
        acce,
        factor,
        line_obs_list=line_segments,
    )
    rvo_vel = rvo_behavior.cal_vel(mode)

    return np.array([[rvo_vel[0]], [rvo_vel[1]]])


def DiffRVO(
    state_tuple: Any,
    neighbor_list: list[Any] | None = None,
    vxmax: float = 1.5,
    vymax: float = 1.5,
    acce: float = 1,
    factor: float = 1.0,
    mode: str = "rvo",
    neighbor_threshold: float = 3.0,
    line_segments: list[list[float]] | None = None,
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
        line_segments (list): Line segments [[x1, y1, x2, y2], ...] (default None).

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
        state_tuple,
        filtered_neighbor_list,
        vxmax,
        vymax,
        acce,
        factor,
        line_obs_list=line_segments,
    )
    rvo_vel = rvo_behavior.cal_vel(mode)
    diff_vel = omni_to_diff(state_tuple[-1], rvo_vel)

    if not diff_vel.any() and not filtered_neighbor_list and not line_segments:
        # With no neighbors or obstacles in range, a fully frozen command means
        # the holonomic RVO solution collapsed toward zero velocity because the
        # goal lies behind the robot (it cannot represent a reversal), so the
        # diff robot would freeze forever instead of turning around. Rotate in
        # place toward the desired heading; forward speed stays zero and
        # ``omni_to_diff`` naturally stops the rotation once the robot faces its
        # goal. When neighbors are present the freeze is collision avoidance and
        # the robot must keep waiting, so this branch is skipped entirely.
        diff_vel = omni_to_diff(state_tuple[-1], [state_tuple[5], state_tuple[6]])
        diff_vel[0, 0] = 0.0

    return diff_vel


def OmniDash(
    state: np.ndarray,
    goal: np.ndarray,
    max_vel: np.ndarray,
    goal_threshold: float = 0.1,
) -> np.ndarray:
    """
    Calculate the body-frame velocity to reach a goal.

    Args:
        state (np.array): Current state [x, y, theta] (3x1 or 2x1).
        goal (np.array): Goal position [x, y] (2x1).
        max_vel (np.array): Maximum velocity [forward, lateral] (2x1).
        goal_threshold (float): Distance threshold to consider goal reached (default 0.1).

    Returns:
        np.array: Body-frame velocity [forward, lateral] (2x1).
    """
    distance, radian = relative_position(state, goal)
    theta = state[2, 0] if state.shape[0] > 2 else 0.0

    if distance > goal_threshold:
        diff_angle = WrapToPi(radian - theta)
        forward = max_vel[0, 0] * cos(diff_angle)
        lateral = max_vel[1, 0] * sin(diff_angle)
    else:
        forward = 0
        lateral = 0

    return np.array([[forward], [lateral]])


def OmniAngularDash(
    state: np.ndarray,
    goal: np.ndarray,
    max_vel: np.ndarray,
    goal_threshold: float = 0.3,
    angle_tolerance: float = 0.1,
    angular_acce: float = float("inf"),
    dt: float = 0.0,
) -> np.ndarray:
    """
    Calculate body-frame velocity to reach a goal.

    Drives forward and strafes laterally toward the goal while turning
    to face it. After arriving at the goal position, rotates in place
    to match the goal orientation.

    The yaw rate is ramped down near the target so the robot can decelerate
    to a stop within the remaining angle. The ramp uses the exact discrete-time
    formula ``ω ≤ -a·dt + √(a²·dt² + 2·a·remaining)`` so the robot stops
    cleanly within one step of ``angle_tolerance`` without overshoot.

    Args:
        state (np.array): Current state [x, y, theta] (3x1).
        goal (np.array): Goal state [x, y, theta] (3x1).
        max_vel (np.array): Absolute maximum velocity [forward, lateral, yaw_rate] (3x1).
        goal_threshold (float): Distance threshold to consider goal reached (default 0.3).
        angle_tolerance (float): Allowable angular deviation (default 0.1).
        angular_acce (float): Angular acceleration limit (rad/s^2). Used to cap
            yaw rate so the robot can always stop within the remaining angle.
            ``inf`` (default) gives bang-bang behaviour.
        dt (float): Simulation time step (s). Used for exact discrete-time decel
            ramp. 0 (default) falls back to the continuous-time approximation.

    Returns:
        np.array: Body-frame velocity [forward, lateral, yaw_rate] (3x1).
    """
    distance, radian = relative_position(state, goal)
    theta = state[2, 0]

    if distance > goal_threshold:
        diff_angle = WrapToPi(radian - theta)
        forward = max_vel[0, 0] * cos(diff_angle)
        lateral = max_vel[1, 0] * sin(diff_angle)
        target_angle = radian
    else:
        forward = 0
        lateral = 0
        target_angle = goal[2, 0] if goal.shape[0] > 2 else theta

    diff_angle = WrapToPi(target_angle - theta)
    if abs(diff_angle) <= angle_tolerance:
        yaw_rate = 0.0
    else:
        if np.isfinite(angular_acce) and angular_acce > 0:
            remaining = abs(diff_angle) - angle_tolerance
            # Exact discrete-time limit: robot moves ω·dt this step before braking,
            # so the true stopping condition is ω·dt + ω²/(2a) ≤ remaining.
            # Solving: ω ≤ -a·dt + √(a²·dt² + 2·a·remaining)
            a_dt = angular_acce * dt
            max_yaw = -a_dt + np.sqrt(a_dt**2 + 2.0 * angular_acce * remaining)
            # The decel ramp grows unbounded with ``remaining``; never exceed
            # the configured yaw-rate limit.
            max_yaw = min(abs(max_vel[2, 0]), max_yaw)
        else:
            max_yaw = abs(max_vel[2, 0])
        yaw_rate = np.sign(diff_angle) * max_yaw

    return np.array([[forward], [lateral], [yaw_rate]])


def DiffDash(
    state: np.ndarray,
    goal: np.ndarray,
    max_vel: np.ndarray,
    goal_threshold: float = 0.1,
    angle_tolerance: float = 0.2,
    angular_acce: float = float("inf"),
    dt: float = 0.0,
) -> np.ndarray:
    """
    Calculate the differential drive velocity to reach a goal.

    Args:
        state (np.array): Current state [x, y, theta] (3x1).
        goal (np.array): Goal position [x, y] (2x1).
        max_vel (np.array): Absolute maximum velocity [linear, angular] (2x1).
        goal_threshold (float): Distance threshold to consider goal reached (default 0.1).
        angle_tolerance (float): Allowable angular deviation (default 0.2).
        angular_acce (float): Angular acceleration limit (rad/s^2) for the decel ramp.
            ``inf`` (default) gives bang-bang behaviour.
        dt (float): Simulation time step (s) for exact discrete-time decel ramp.

    Returns:
        np.array: Velocity [linear, angular] (2x1).
    """
    distance, radian = relative_position(state, goal)

    if distance < goal_threshold:
        return np.zeros((2, 1))

    diff_radian = WrapToPi(radian - state[2, 0])
    linear = max_vel[0, 0] * np.cos(diff_radian)

    if abs(diff_radian) < angle_tolerance:
        angular = 0.0
    else:
        if np.isfinite(angular_acce) and angular_acce > 0:
            remaining = abs(diff_radian) - angle_tolerance
            a_dt = angular_acce * dt
            max_ang = -a_dt + np.sqrt(a_dt**2 + 2.0 * angular_acce * remaining)
            # The decel ramp grows unbounded with ``remaining``; never exceed
            # the configured angular-speed limit.
            max_ang = min(abs(max_vel[1, 0]), max_ang)
        else:
            max_ang = abs(max_vel[1, 0])
        angular = np.sign(diff_radian) * max_ang

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
