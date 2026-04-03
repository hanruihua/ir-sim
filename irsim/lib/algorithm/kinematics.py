"""
This file is the implementation of the kinematics for different robots.

reference: Lynch, Kevin M., and Frank C. Park. Modern Robotics: Mechanics, Planning, and Control. 1st ed. Cambridge, MA: Cambridge University Press, 2017.
"""

from math import cos, sin, tan

import numpy as np

from irsim.util.random import rng
from irsim.util.util import WrapToPi, validate_shape


@validate_shape(state=3, velocity=2)
def differential_kinematics(
    state: np.ndarray,
    velocity: np.ndarray,
    step_time: float,
    noise: bool = False,
    alpha: list[float] | None = None,
) -> np.ndarray:
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
    if alpha is None:
        alpha = [0.03, 0, 0, 0.03]

    if noise:
        if len(alpha) < 4:
            raise ValueError("Parameter 'alpha' must have length >= 4 when noise=True")
        std_linear = np.sqrt(
            alpha[0] * (velocity[0, 0] ** 2) + alpha[1] * (velocity[1, 0] ** 2)
        )
        std_angular = np.sqrt(
            alpha[2] * (velocity[0, 0] ** 2) + alpha[3] * (velocity[1, 0] ** 2)
        )
        real_velocity = velocity + rng.normal(
            [[0], [0]], scale=[[std_linear], [std_angular]]
        )
    else:
        real_velocity = velocity

    phi = state[2, 0]
    co_matrix = np.array([[cos(phi), 0], [sin(phi), 0], [0, 1]])
    next_state = state[0:3] + co_matrix @ real_velocity * step_time
    next_state[2, 0] = WrapToPi(next_state[2, 0])

    return next_state


@validate_shape(state=4, velocity=2)
def ackermann_kinematics(
    state: np.ndarray,
    velocity: np.ndarray,
    step_time: float,
    noise: bool = False,
    alpha: list[float] | None = None,
    mode: str = "steer",
    wheelbase: float = 1,
) -> np.ndarray:
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
    if alpha is None:
        alpha = [0.03, 0, 0, 0.03]

    phi = state[2, 0]
    psi = state[3, 0]

    if noise:
        if len(alpha) < 4:
            raise ValueError("Parameter 'alpha' must have length >= 4 when noise=True")
        std_linear = np.sqrt(
            alpha[0] * (velocity[0, 0] ** 2) + alpha[1] * (velocity[1, 0] ** 2)
        )
        std_angular = np.sqrt(
            alpha[2] * (velocity[0, 0] ** 2) + alpha[3] * (velocity[1, 0] ** 2)
        )
        real_velocity = velocity + rng.normal(
            [[0], [0]], scale=[[std_linear], [std_angular]]
        )
    else:
        real_velocity = velocity

    if mode == "steer" or mode == "angular":
        co_matrix = np.array(
            [[cos(phi), 0], [sin(phi), 0], [tan(psi) / wheelbase, 0], [0, 1]]
        )

    d_state = co_matrix @ real_velocity
    new_state = state + d_state * step_time

    if mode == "steer":
        new_state[3, 0] = real_velocity[1, 0]

    new_state[2, 0] = WrapToPi(new_state[2, 0])

    return new_state


@validate_shape(state=3, velocity=2)
def omni_kinematics(
    state: np.ndarray,
    velocity: np.ndarray,
    step_time: float,
    noise: bool = False,
    alpha: list[float] | None = None,
) -> np.ndarray:
    """
    Calculate the next position for an omnidirectional robot.

    Uses body-frame velocity: the two components are forward and lateral
    speeds relative to the robot heading (theta). Since omni robots have
    no yaw control, theta remains unchanged.

    Args:
        state: A 3x1 vector [x, y, theta] representing the current state.
        velocity: A 2x1 vector [forward, lateral] in body frame.
        step_time: The time step for the simulation.
        noise: Boolean indicating whether to add noise to the velocity (default False).
        alpha: List of noise parameters for the velocity model (default [0.03, 0.03]).

    Returns:
        next_state: A 3x1 vector [x, y, theta] representing the next state.
            Theta is preserved unchanged.
    """
    if alpha is None:
        alpha = [0.03, 0.03]

    if noise:
        if len(alpha) < 2:
            raise ValueError("Parameter 'alpha' must have length >= 2 when noise=True")
        std_fwd = np.sqrt(alpha[0])
        std_lat = np.sqrt(alpha[1])
        real_velocity = velocity + rng.normal([[0], [0]], scale=[[std_fwd], [std_lat]])
    else:
        real_velocity = velocity

    theta = state[2, 0]
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    fwd = real_velocity[0, 0]
    lat = real_velocity[1, 0]

    dx = fwd * cos_t - lat * sin_t
    dy = fwd * sin_t + lat * cos_t

    next_state = state[0:3].astype(float)
    next_state[0:2] += np.array([[dx], [dy]]) * step_time

    return next_state


@validate_shape(state=3, velocity=3)
def omni_angular_kinematics(
    state: np.ndarray,
    velocity: np.ndarray,
    step_time: float,
    noise: bool = False,
    alpha: list[float] | None = None,
) -> np.ndarray:
    """
    Calculate the next state for an omnidirectional robot with angular velocity.

    Uses body-frame velocity: the first two components are forward and lateral
    speeds relative to the robot heading, and the third is yaw rate.

    Args:
        state: A 3x1 vector [x, y, theta] representing the current position and orientation.
        velocity: A 3x1 vector [forward, lateral, yaw_rate] in body frame.
        step_time: The time step for the simulation.
        noise: Boolean indicating whether to add noise to the velocity (default False).
        alpha: List of noise parameters [alpha_fwd, alpha_lat, alpha_yaw] (default [0.03, 0.03, 0.03]).
            Each value scales the standard deviation for the corresponding velocity channel.

    Returns:
        next_state: A 3x1 vector [x, y, theta] representing the next state.
    """
    if alpha is None:
        alpha = [0.03, 0.03, 0.03]

    if noise:
        if len(alpha) < 3:
            raise ValueError("Parameter 'alpha' must have length >= 3 when noise=True")
        std_fwd = np.sqrt(alpha[0])
        std_lat = np.sqrt(alpha[1])
        std_yaw = np.sqrt(alpha[2])
        real_velocity = velocity + rng.normal(
            [[0], [0], [0]], scale=[[std_fwd], [std_lat], [std_yaw]]
        )
    else:
        real_velocity = velocity

    theta = state[2, 0]
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    fwd = real_velocity[0, 0]
    lat = real_velocity[1, 0]

    dx = fwd * cos_t - lat * sin_t
    dy = fwd * sin_t + lat * cos_t
    dtheta = real_velocity[2, 0]

    next_state = state[0:3] + np.array([[dx], [dy], [dtheta]]) * step_time
    next_state[2, 0] = WrapToPi(next_state[2, 0])

    return next_state
