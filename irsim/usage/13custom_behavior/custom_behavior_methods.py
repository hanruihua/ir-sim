from irsim.lib import register_behavior
from irsim.util.util import relative_position, WrapToPi
import numpy as np


@register_behavior("diff", "dash_custom")
def beh_diff_dash(ego_object, external_objects=[], **kwargs):

    print("This is a custom behavior example for differential drive with dash2")

    state = ego_object.state
    goal = ego_object.goal
    goal_threshold = ego_object.goal_threshold
    _, max_vel = ego_object.get_vel_range()
    angle_tolerance = kwargs.get("angle_tolerance", 0.1)

    behavior_vel = DiffDash2(state, goal, max_vel, goal_threshold, angle_tolerance)

    return behavior_vel


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