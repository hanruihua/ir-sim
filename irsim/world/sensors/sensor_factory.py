from typing import Any

import numpy as np

from irsim.world.sensors.lidar2d import Lidar2D


class SensorFactory:
    def create_sensor(self, state: np.ndarray, obj_id: int, **kwargs: Any) -> Any:
        sensor_type = kwargs.get("name", kwargs.get("type", "lidar2d"))

        if sensor_type == "lidar2d":
            return Lidar2D(state, obj_id, **kwargs)
        raise NotImplementedError(f"Sensor types {type} not implemented")
