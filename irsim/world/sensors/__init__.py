"""
Sensor classes for IR-SIM simulation.

This package contains:
- contact2d: 2D contact sensor (contact bookkeeping of one object)
- fmcw_lidar2d: 2D FMCW LiDAR sensor implementation
- lidar2d: 2D LiDAR sensor implementation
- sensor_factory: Sensor factory for creating sensors
"""

from .contact2d import Contact2D
from .fmcw_lidar2d import FMCWLidar2D
from .lidar2d import Lidar2D
from .sensor_factory import SensorFactory

__all__ = ["Contact2D", "FMCWLidar2D", "Lidar2D", "SensorFactory"]
