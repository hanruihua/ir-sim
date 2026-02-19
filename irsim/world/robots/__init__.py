"""
Robot classes for IR-SIM simulation.

This package contains different robot types:
- robot_diff: Differential drive robot
- robot_omni: Omnidirectional robot
- robot_omni_angular: Omnidirectional robot with angular velocity
- robot_acker: Ackermann steering robot
"""

from .robot_acker import RobotAcker
from .robot_diff import RobotDiff
from .robot_omni import RobotOmni
from .robot_omni_angular import RobotOmniAngular

__all__ = ["RobotAcker", "RobotDiff", "RobotOmni", "RobotOmniAngular"]
