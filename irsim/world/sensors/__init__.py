"""
Sensor classes for IR-SIM simulation.

This package contains:
- fmcw_lidar2d: 2D FMCW LiDAR sensor implementation
- lidar2d: 2D LiDAR sensor implementation
- sensor_factory: Sensor factory for creating sensors
"""

from .fmcw_lidar2d import FMCWLidar2D
from .lidar2d import Lidar2D
from .sensor_factory import SensorFactory

__all__ = ["FMCWLidar2D", "Lidar2D", "SensorFactory"]
