from irsim.lib.algorithm.kinematics import (
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

from irsim.lib.algorithm.generation import random_generate_polygon, generate_polygon
from irsim.lib.algorithm.rvo import reciprocal_vel_obs

from irsim.lib.handler.kinematics_handler import KinematicsFactory