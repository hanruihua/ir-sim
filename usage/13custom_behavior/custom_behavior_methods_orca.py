import numpy as np
import pyrvo

from irsim.config import env_param, world_param
from irsim.lib import register_behavior

orca = pyrvo.RVOSimulator()
objects = env_param.objects

orca.set_time_step(world_param.step_time)
neighborDist = 10.0
maxNeighbors = 10
timeHorizon = 10.0
timeHorizonObst = 10.0
radius = 0.25
maxSpeed = 2.0

orca.set_agent_defaults(
    neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed
)

for i, obj in enumerate(objects):
    position = obj.state[:2, 0].tolist()
    orca.add_agent(position)
    orca.set_agent_pref_velocity(i, obj.get_desired_omni_vel(normalized=True))


@register_behavior("omni", "orca")
def beh_omni_orca(ego_object, external_objects, **kwargs):
    for i, robot in enumerate(env_param.objects):
        orca.set_agent_pref_velocity(
            i, robot.get_desired_omni_vel(normalized=True).flatten().tolist()
        )
        orca.set_agent_position(i, robot.state[:2, 0].tolist())

    obj_id = ego_object.id
    orca.do_step()

    return np.array(orca.get_agent_velocity(obj_id).to_tuple()).reshape(2, 1)
