"""
Sensor classes for IR-SIM simulation.

This package contains:
- lidar2d: 2D LiDAR sensor implementation
- sensor_factory: Sensor factory for creating sensors
"""

from .lidar2d import Lidar2D
from .sensor_factory import SensorFactory

__all__ = ['Lidar2D', 'SensorFactory'] 