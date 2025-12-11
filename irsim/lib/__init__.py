from typing import Any, Callable

from irsim.lib.algorithm.generation import generate_polygon, random_generate_polygon
from irsim.lib.algorithm.kinematics import (
    ackermann_kinematics,
    differential_kinematics,
    omni_kinematics,
)
from irsim.lib.algorithm.rvo import reciprocal_vel_obs
from irsim.lib.behavior.behavior import Behavior
from irsim.lib.behavior.behavior_registry import (
    register_behavior,
    register_group_behavior,
)
from irsim.lib.handler.geometry_handler import GeometryFactory
from irsim.lib.handler.kinematics_handler import KinematicsFactory

kinematics_factory: dict[str, Callable[..., Any]] = {
    "diff": differential_kinematics,
    "acker": ackermann_kinematics,
    "omni": omni_kinematics,
}
