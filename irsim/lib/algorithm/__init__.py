"""
Core algorithms for IR-SIM simulation.

This package contains:
- kinematics: Robot kinematics functions
- rvo: Reciprocal Velocity Obstacle algorithm
- social_force_model: Anisotropic Social Force Model (Moussaïd 2009), per-agent and batched with Moussaïd 2010 social groups
- generation: Polygon generation utilities
- ray_casting_2d: Vectorized 2D ray casting for lidar sensors
"""

from .generation import generate_polygon, random_generate_polygon
from .kinematics import (
    ackermann_kinematics,
    differential_kinematics,
    omni_angular_kinematics,
    omni_kinematics,
)
from .ray_casting_2d import cast_rays
from .rvo import reciprocal_vel_obs
from .social_force_model import SocialForceModelBatch, social_force_model

__all__ = [
    "SocialForceModelBatch",
    "ackermann_kinematics",
    "cast_rays",
    "differential_kinematics",
    "generate_polygon",
    "omni_angular_kinematics",
    "omni_kinematics",
    "random_generate_polygon",
    "reciprocal_vel_obs",
    "social_force_model",
]
