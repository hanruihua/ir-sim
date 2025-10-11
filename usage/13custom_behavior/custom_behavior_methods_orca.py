import numpy as np
import pyrvo

from irsim.lib import register_behavior

orca = pyrvo.RVOSimulator()


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

    for i, obj in enumerate([ego_object, *external_objects]):
        position = [obj.state[0, 0], obj.state[1, 0]]
        orca.add_agent(
            position,
            neighborDist,
            maxNeighbors,
            timeHorizon,
            timeHorizonObst,
            obj.radius,
            obj.vel_max[0, 0],
        )
        orca.set_agent_pref_velocity(
            i, ego_object.get_desired_omni_vel(normalized=True)
        )

    orca.do_step()
    return np.array(orca.get_agent_velocity(0).to_tuple()).reshape(2, 1)
