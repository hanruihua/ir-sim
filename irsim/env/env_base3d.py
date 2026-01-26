from __future__ import annotations

import itertools
from typing import Any, Optional

from irsim.config import env_param
from irsim.env import EnvBase
from irsim.world.object_base import ObjectBase
from irsim.world.object_factory import ObjectFactory
from irsim.world.world3d import World3D

from .env_plot3d import EnvPlot3D


class EnvBase3D(EnvBase):
    """
    This class is the 3D version of the environment class. It inherits from the :py:class:`.EnvBase` class to provide the 3D plot environment.
    """

    def __init__(self, world_name: Optional[str], **kwargs: Any):
        """Initialize a 3D environment with world and objects parsed from YAML.

        Args:
            world_name (str | None): Path to the world configuration YAML.
            **kwargs: Additional environment options forwarded to ``EnvBase``.
        """
        super().__init__(world_name, **kwargs)

        object_factory = ObjectFactory()

        self._world = World3D(world_name, **self.env_config.parse["world"])

        ObjectBase.id_iter = itertools.count()

        self._robot_collection = object_factory.create_from_parse(
            self.env_config.parse["robot"], "robot"
        )
        self._obstacle_collection = object_factory.create_from_parse(
            self.env_config.parse["obstacle"], "obstacle"
        )
        self._map_collection = object_factory.create_from_map(
            self._world.obstacle_positions,
            self._world.buffer_reso,
            grid_map=self._world.grid_map,
            grid_reso=self._world.reso,
            world_offset=self._world.offset,
        )

        self._env_plot.close()

        self._env_plot = EnvPlot3D(self._world, self.objects, **self._world.plot_parse)

        env_param.objects = self.objects
