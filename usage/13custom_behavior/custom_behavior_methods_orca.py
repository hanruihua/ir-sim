import numpy as np

from irsim.lib import register_behavior
from irsim.util.util import WrapToPi, relative_position
import pyrvo
import math 

def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def v_neg(a):
    return (-a[0], -a[1])


def v_abs_sq(a):
    return a[0] * a[0] + a[1] * a[1]


def v_norm(a):
    return math.sqrt(v_abs_sq(a))


def v_normalize(a):
    n = v_norm(a)
    if n == 0.0:
        return (0.0, 0.0)
    return (a[0] / n, a[1] / n)


@register_behavior("omni", "orca")
def beh_omni_orca(ego_object, external_objects=None, **kwargs):
    if external_objects is None:
        external_objects = []

    neighborDist = kwargs.get("neighborDist", 15.0)
    maxNeighbors = kwargs.get("maxNeighbors", 10)
    timeHorizon = kwargs.get("timeHorizon", 20.0)
    timeHorizonObst = kwargs.get("timeHorizonObst", 10.0)
    # radius = kwargs.get("radius", 1.5)
    # maxSpeed = kwargs.get("maxSpeed", 2.0)
    orca = pyrvo.RVOSimulator()

    for i, obj in enumerate([ego_object] + external_objects):
        position = [obj.state[0, 0], obj.state[1, 0]]
        orca.add_agent(position, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, obj.radius, obj.vel_max[0, 0])

        goal_vec = v_sub(obj.goal, position)
        if v_abs_sq(goal_vec) > 1.0:
            goal_vec = v_normalize(goal_vec)
        orca.set_agent_pref_velocity(i, goal_vec)

    orca.do_step()
    a = np.array(orca.get_agent_velocity(0).to_tuple()).reshape(2, 1)

    return a

    
    



