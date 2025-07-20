"""
Core algorithms for IR-SIM simulation.

This package contains:
- kinematics: Robot kinematics functions
- rvo: Reciprocal Velocity Obstacle algorithm
- generation: Polygon generation utilities
"""

from .kinematics import differential_kinematics, ackermann_kinematics, omni_kinematics
from .rvo import reciprocal_vel_obs
from .generation import random_generate_polygon, generate_polygon

__all__ = [
    'differential_kinematics', 'ackermann_kinematics', 'omni_kinematics',
    'reciprocal_vel_obs', 'random_generate_polygon', 'generate_polygon'
]
