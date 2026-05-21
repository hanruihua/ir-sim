"""
Core algorithms for IR-SIM simulation.

This package contains:
- kinematics: Robot kinematics functions
- rvo: Reciprocal Velocity Obstacle algorithm
- social_force_model: Anisotropic Social Force Model (Moussaïd 2009)
- generation: Polygon generation utilities
"""

from .generation import generate_polygon, random_generate_polygon
from .kinematics import (
    ackermann_kinematics,
    differential_kinematics,
    omni_angular_kinematics,
    omni_kinematics,
)
from .rvo import reciprocal_vel_obs
from .social_force_model import social_force_model

__all__ = [
    "ackermann_kinematics",
    "differential_kinematics",
    "generate_polygon",
    "omni_angular_kinematics",
    "omni_kinematics",
    "random_generate_polygon",
    "reciprocal_vel_obs",
    "social_force_model",
]
