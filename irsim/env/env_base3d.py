from irsim.env import EnvBase
from .env_plot3d import EnvPlot3D

class EnvBase3D(EnvBase):

    def __init__(self, world_name, **kwargs):
        super().__init__(world_name, **kwargs)

        self._env_plot = EnvPlot3D(
            self._world.grid_map,
            self.objects,
            self._world.x_range,
            self._world.y_range,
            **self.env_config.parse["plot"],
        )



    
    
