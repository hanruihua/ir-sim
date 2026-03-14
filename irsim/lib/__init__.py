from collections.abc import Callable
from typing import Any

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
    register_behavior_class,
    register_group_behavior,
    register_group_behavior_class,
)
from irsim.lib.handler.geometry_handler import GeometryFactory
from irsim.lib.handler.kinematics_handler import (
    KinematicsFactory,
    KinematicsHandler,
    register_kinematics,
)

kinematics_factory: dict[str, Callable[..., Any]] = {
    "diff": differential_kinematics,
    "acker": ackermann_kinematics,
    "omni": omni_kinematics,
}


def __getattr__(name: str):
    if name == "GroupBehavior":
        from irsim.lib.behavior.group_behavior import GroupBehavior

        return GroupBehavior

    _path_planners = {
        "AStarPlanner",
        "InformedRRTStar",
        "JPSPlanner",
        "PRMPlanner",
        "RRT",
        "RRTStar",
    }
    if name in _path_planners:
        import irsim.lib.path_planners as _pp

        return getattr(_pp, name)

    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
