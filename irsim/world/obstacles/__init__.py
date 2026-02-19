"""
Obstacle classes for IR-SIM simulation.

This package contains different obstacle types:
- obstacle_diff: Differential drive obstacle
- obstacle_omni: Omnidirectional obstacle
- obstacle_omni_angular: Omnidirectional obstacle with angular velocity
- obstacle_acker: Ackermann steering obstacle
- obstacle_static: Static obstacle
"""

from .obstacle_acker import ObstacleAcker
from .obstacle_diff import ObstacleDiff
from .obstacle_omni import ObstacleOmni
from .obstacle_omni_angular import ObstacleOmniAngular
from .obstacle_static import ObjectStatic

__all__ = ["ObjectStatic", "ObstacleAcker", "ObstacleDiff", "ObstacleOmni", "ObstacleOmniAngular"]
