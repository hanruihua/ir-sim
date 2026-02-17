import os
import warnings
from typing import TYPE_CHECKING, Any, ClassVar, Optional

import numpy as np

from irsim.util.util import check_unknown_kwargs
from irsim.world.map import Map, _downsample_occupancy_grid, resolve_obstacle_map

if TYPE_CHECKING:
    from irsim.config.world_param import WorldParam


class World:
    """
    Represents the main simulation environment, managing objects and maps.

    Attributes:
        name (str): Name of the world.
        height (float): Height of the world.
        width (float): Width of the world.
        step_time (float): Time interval between steps.
        sample_time (float): Time interval between samples.
        offset (list): Offset for the world's position.
        control_mode (str): Control mode ('auto' or 'keyboard').
        collision_mode (str): Collision mode ('stop',  , 'unobstructed').
        obstacle_map: ``None``, image path (str), grid ndarray, or generator spec dict.
        mdownsample (int): Downsampling factor for the obstacle map.
        status: Status of the world and objects.
        plot: Plot configuration for the world.
    """

    _VALID_PARAMS: ClassVar[set[str]] = {
        "name",
        "height",
        "width",
        "step_time",
        "sample_time",
        "offset",
        "control_mode",
        "collision_mode",
        "obstacle_map",
        "mdownsample",
        "plot",
        "status",
    }

    def __init__(
        self,
        name: str | None = "world",
        height: float = 10,
        width: float = 10,
        step_time: float = 0.1,
        sample_time: float | None = None,
        offset: list[float] | None = None,
        control_mode: str = "auto",
        collision_mode: str = "stop",
        obstacle_map: Any | None = None,
        mdownsample: int = 1,
        plot: dict[str, Any] | None = None,
        status: str = "None",
        world_param_instance: Optional["WorldParam"] = None,
        **kwargs: Any,
    ) -> None:
        """
        Initialize the world object.

        Parameters:
            name (str): Name of the world.
            height (float): Height of the world.
            width (float): Width of the world.
            step_time (float): Time interval between steps.
            sample_time (float): Time interval between samples.
            offset (list): Offset for the world's position.
            control_mode (str): Control mode ('auto' or 'keyboard').
            collision_mode (str): Collision mode ('stop',  , 'unobstructed').
            obstacle_map: ``None``, image path (str), grid ndarray, or generator spec dict.
            mdownsample (int): Downsampling factor for the obstacle map.
            plot (dict): Plot configuration.
            status (str): Initial simulation status.
            world_param_instance: Optional WorldParam instance. If provided, uses
                this instance for param storage; otherwise falls back to global.
        """

        # Environment reference for accessing params (set by EnvBase after creation)
        self._env = None

        # Store world_param instance or fallback to global
        if world_param_instance is not None:
            self._wp = world_param_instance
        else:
            from irsim.config import world_param

            self._wp = world_param

        # basic properties
        if offset is None:
            offset = [0, 0]
        if plot is None:
            plot = {}

        self.name = os.path.basename(name or "world").split(".")[0]
        self.height = height
        self.width = width
        self.step_time = step_time
        self.sample_time = sample_time if sample_time is not None else step_time
        self.offset = offset

        self.count = 0
        self.sampling = True

        self._wp.step_time = step_time

        self.x_range = [self.offset[0], self.offset[0] + self.width]
        self.y_range = [self.offset[1], self.offset[1] + self.height]

        # obstacle map
        self.grid_map, self.obstacle_index, self.obstacle_positions = self.gen_grid_map(
            obstacle_map, mdownsample
        )

        # visualization
        self.plot_parse = plot

        self.status = status

        # mode
        self._wp.control_mode = control_mode
        self._wp.collision_mode = collision_mode

        check_unknown_kwargs(
            kwargs, self._VALID_PARAMS, context=" in 'world' config", logger=self.logger
        )

    def step(self) -> None:
        """
        Advance the simulation by one step.
        """
        self.count += 1
        self.sampling = self.count % (int(self.sample_time / self.step_time)) == 0

        self._wp.time = self.time
        self._wp.count = self.count

    def gen_grid_map(
        self,
        obstacle_map: Any | None = None,
        mdownsample: int = 1,
    ) -> tuple:
        """Generate a grid map for obstacles.

        The *obstacle_map* value is resolved to a float64 ndarray by
        :pyfunc:`irsim.world.map.resolve_obstacle_map`.  Accepted types:
        ``None``, path string (image), ndarray, or generator spec dict.

        Args:
            obstacle_map: ``None``, path string, ndarray, or generator spec dict.
            mdownsample (int): Downsampling factor.

        Returns:
            tuple: ``(grid_map, obstacle_index, obstacle_positions)``.
        """
        grid_map = resolve_obstacle_map(
            obstacle_map,
            world_width=self.width,
            world_height=self.height,
        )

        if grid_map is not None:
            grid_map = grid_map[::mdownsample, ::mdownsample]
            x_reso = self.width / grid_map.shape[0]
            y_reso = self.height / grid_map.shape[1]
            self.reso = np.array([[x_reso], [y_reso]])
            obstacle_index = np.array(np.where(grid_map > 50))
            obstacle_positions = obstacle_index * self.reso
        else:
            obstacle_index = None
            obstacle_positions = None
            self.reso = np.zeros((2, 1))

        return grid_map, obstacle_index, obstacle_positions

    def get_map(
        self, resolution: float = 0.1, obstacle_list: list[Any] | None = None
    ) -> "Map":
        """
        Get the map of the world with the given resolution.

        When *resolution* is coarser than the obstacle grid, the grid is
        downsampled (conservative: any obstacle in a block marks the block) so
        that planning and collision use the same grid. When *resolution* is
        finer than the grid, no upsampling is done; planning uses the grid
        resolution and a warning is emitted.
        """
        world_offset = (float(self.offset[0]), float(self.offset[1]))
        grid = self.grid_map
        res = resolution

        if grid is not None:
            if not (np.isfinite(res) and res > 0):
                res = float(self.width / grid.shape[0])
                warnings.warn(
                    f"resolution must be positive and finite; using grid "
                    f"resolution {res}.",
                    stacklevel=2,
                )
            else:
                current_rx = self.width / grid.shape[0]
                current_ry = self.height / grid.shape[1]
                thresh_coarser = 1.05
                thresh_finer = 0.95
                if res > max(current_rx, current_ry) * thresh_coarser:
                    grid = _downsample_occupancy_grid(
                        grid, self.width, self.height, res
                    )
                    warnings.warn(
                        f"Planning grid downsampled from ({self.grid_map.shape[0]}, "
                        f"{self.grid_map.shape[1]}) at {current_rx:.4f}m/cell to "
                        f"({grid.shape[0]}, {grid.shape[1]}) at {res}m/cell.",
                        stacklevel=2,
                    )
                elif res < min(current_rx, current_ry) * thresh_finer:
                    warnings.warn(
                        f"Requested resolution ({res}) is finer than the "
                        f"obstacle grid ({current_rx:.4f} x {current_ry:.4f}). "
                        "Planning will use the grid resolution.",
                        stacklevel=2,
                    )

        return Map(
            self.width,
            self.height,
            res,
            obstacle_list,
            grid,
            world_offset=world_offset,
        )

    def reset(self) -> None:
        """
        Reset the world simulation.
        """

        self._wp.count = 0
        self.count = 0

    @property
    def time(self) -> float:
        """
        Get the current simulation time.

        Returns:
            float: Current time based on steps and step_time.
        """
        return round(self.count * self.step_time, 2)

    @property
    def buffer_reso(self) -> float:
        """
        Get the maximum resolution of the world.

        Returns:
            float: Maximum resolution.
        """
        return np.max(self.reso)

    @property
    def _env_param(self):
        """
        Access env_param via env instance if available, otherwise fallback to global.

        Returns:
            EnvParam: The env param instance.
        """
        if self._env is not None:
            return self._env._env_param
        from irsim.config import env_param

        return env_param

    @property
    def logger(self):
        """
        Get the logger of the env_param.

        Returns:
            Logger: The logger associated in the env_param.
        """

        return self._env_param.logger
