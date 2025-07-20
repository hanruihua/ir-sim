"""
Obstacle classes for IR-SIM simulation.

This package contains different obstacle types:
- obstacle_diff: Differential drive obstacle
- obstacle_omni: Omnidirectional obstacle
- obstacle_acker: Ackermann steering obstacle
- obstacle_static: Static obstacle
"""

from .obstacle_diff import ObstacleDiff
from .obstacle_omni import ObstacleOmni
from .obstacle_acker import ObstacleAcker
from .obstacle_static import ObjectStatic

__all__ = ['ObstacleDiff', 'ObstacleOmni', 'ObstacleAcker', 'ObjectStatic'] 