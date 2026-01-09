# custom_behavior_methods.py
"""
Custom individual behavior examples for IR-SIM.

This module demonstrates two patterns for custom individual behaviors:
1. Function-based (register_behavior) - for simple stateless behaviors
2. Class-based (register_behavior_class) - for behaviors with state/initialization
"""
from typing import Any

import numpy as np

from irsim.lib import register_behavior
from irsim.lib.behavior.behavior_registry import register_behavior_class
from irsim.util.util import WrapToPi, relative_position

# =============================================================================
# Function-based Individual Behavior Example (stateless)
# =============================================================================


@register_behavior("diff", "dash_custom")
def beh_diff_dash(ego_object, external_objects=None, **kwargs):
    if external_objects is None:
        external_objects = []
    print("This is a custom behavior example for differential drive with dash2")

    state = ego_object.state
    goal = ego_object.goal
    goal_threshold = ego_object.goal_threshold
    _, max_vel = ego_object.get_vel_range()
    angle_tolerance = kwargs.get("angle_tolerance", 0.1)

    return DiffDash2(state, goal, max_vel, goal_threshold, angle_tolerance)


def DiffDash2(state, goal, max_vel, goal_threshold=0.1, angle_tolerance=0.2):
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


# =============================================================================
# Class-based Individual Behavior Example (with state)
# =============================================================================


@register_behavior_class("diff", "smooth_dash")
def init_smooth_dash(object_info, **kwargs):
    """Registered initializer returning a class-based handler."""
    return SmoothDashBehavior(object_info, **kwargs)


class SmoothDashBehavior:
    """
    Class-based smooth dash behavior with velocity smoothing.

    This behavior maintains previous velocity for smooth acceleration,
    demonstrating stateful behavior with the @register_behavior_class decorator.

    Use this pattern when your behavior needs:
    - One-time initialization (loading models, setting up planners)
    - State tracking between simulation steps
    - Cached computations for efficiency
    """

    def __init__(self, object_info, smoothing: float = 0.3, **kwargs):
        """
        Initialize smooth dash behavior.

        Args:
            object_info: Object information from ObjectBase
            smoothing: Smoothing factor (0-1), higher = smoother
        """
        self._smoothing = smoothing
        self._prev_vel = np.zeros((2, 1))
        self._object_info = object_info

    def __call__(
        self,
        ego_object,
        external_objects: list,
        **kwargs: Any,
    ) -> np.ndarray:
        """
        Generate smoothed velocity toward goal.

        Args:
            ego_object: The controlled object
            external_objects: Other objects in environment
            **kwargs: Additional parameters

        Returns:
            Smoothed velocity array (2x1)
        """
        if ego_object.goal is None:
            return np.zeros((2, 1))

        # Calculate desired velocity (simple dash)
        state = ego_object.state
        goal = ego_object.goal
        _, max_vel = ego_object.get_vel_range()

        diff = goal[:2, 0] - state[:2, 0]
        distance = np.linalg.norm(diff)

        if distance < ego_object.goal_threshold:
            target_vel = np.zeros((2, 1))
        else:
            # Convert to diff drive velocities
            angle_to_goal = np.arctan2(diff[1], diff[0])
            diff_angle = angle_to_goal - state[2, 0]
            diff_angle = np.arctan2(np.sin(diff_angle), np.cos(diff_angle))

            linear = max_vel[0, 0] * np.cos(diff_angle)
            angular = max_vel[1, 0] * np.sign(diff_angle) if abs(diff_angle) > 0.1 else 0

            target_vel = np.array([[linear], [angular]])

        # Apply smoothing (exponential moving average)
        smoothed_vel = self._smoothing * self._prev_vel + (1 - self._smoothing) * target_vel
        self._prev_vel = smoothed_vel

        return smoothed_vel
