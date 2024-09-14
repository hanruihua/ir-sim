from irsim.world.sensors.lidar2d import Lidar2D


class SensorFactory:

    def create_sensor(self, state, obj_id, **kwargs):

        sensor_type = kwargs.get("type", "lidar2d")

        if sensor_type == "lidar2d":
            return Lidar2D(state, obj_id, **kwargs)
        else:
            raise NotImplementedError(f"Sensor types {type} not implemented")

    # def __init__(self, type='diff', shape='circle', **kwargs) -> None:
