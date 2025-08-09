import numpy as np

from irsim.lib import register_behavior
from irsim.util.util import WrapToPi, relative_position


@register_behavior("diff", "dash_custom")
def beh_diff_dash(ego_object, objects, **kwargs):
    state = ego_object.state
    goal = ego_object.goal
    goal_threshold = ego_object.goal_threshold
    _, max_vel = ego_object.get_vel_range()
    angle_tolerance = kwargs.get("angle_tolerance", 0.1)

    return DiffDash2(state, goal, max_vel, goal_threshold, angle_tolerance)


def DiffDash2(state, goal, max_vel, goal_threshold=0.1, angle_tolerance=0.2):
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
