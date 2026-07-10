from typing import Any

import numpy as np

from irsim.world.sensors.fmcw_lidar2d import FMCWLidar2D
from irsim.world.sensors.lidar2d import Lidar2D


class SensorFactory:
    """Factory for sensors declared in YAML object configurations.

    The factory reads the ``name`` or ``type`` key from a sensor dictionary and
    creates the matching concrete sensor class. Currently supported names are
    ``"lidar2d"`` and ``"fmcw_lidar2d"``.
    """

    def create_sensor(self, state: np.ndarray, obj_id: int, **kwargs: Any) -> Any:
        """Create a sensor instance from configuration kwargs.

        Args:
            state (np.ndarray): Initial sensor state.
            obj_id (int): Associated object id.
            **kwargs: Sensor configuration; expects 'name' or 'type'.

        Returns:
            Any: A concrete sensor instance (e.g., Lidar2D).

        Raises:
            NotImplementedError: If the requested sensor type is not supported.
        """
        sensor_type = kwargs.get("name", kwargs.get("type", "lidar2d"))

        if sensor_type == "lidar2d":
            return Lidar2D(state, obj_id, **kwargs)
        if sensor_type == "fmcw_lidar2d":
            return FMCWLidar2D(state, obj_id, **kwargs)
        raise NotImplementedError(f"Sensor type {sensor_type} not implemented")
