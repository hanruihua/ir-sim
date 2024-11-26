from irsim.lib.kinematics import (
    differential_kinematics,
    ackermann_kinematics,
    omni_kinematics,
)

kinematics_factory = {
    "diff": differential_kinematics,
    "acker": ackermann_kinematics,
    "omni": omni_kinematics,
}

from irsim.lib.behavior.behavior_registry import register_behavior
from irsim.lib.behavior.behavior import Behavior

from irsim.lib.generation import random_generate_polygon