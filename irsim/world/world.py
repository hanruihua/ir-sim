import os
from typing import Optional

import numpy as np
import matplotlib.image as mpimg

from irsim.util.util import file_check
from irsim.global_param import world_param
from irsim.global_param.path_param import path_manager as pm
from irsim.world.map import Map

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
        collision_mode (str): Collision mode ('stop', 'reactive', 'unobstructed').
        obstacle_map: Image file for the obstacle map.
        mdownsample (int): Downsampling factor for the obstacle map.
        status: Status of the world and objects.
        plot: Plot configuration for the world.
    """

    def __init__(
        self,
        name: Optional[str] = "world",
        height: float = 10,
        width: float = 10,
        step_time: float = 0.1,
        sample_time: float = 0.1,
        offset: list = [0, 0],
        control_mode: str = "auto",
        collision_mode: str = "stop",
        obstacle_map=None,
        mdownsample: int = 1,
        plot: dict = dict(),
        status: str = "None",
        **kwargs,
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
            collision_mode (str): Collision mode ('stop', 'reactive', 'unobstructed').
            obstacle_map: Image file for the obstacle map.
            mdownsample (int): Downsampling factor for the obstacle map.
            plot (dict): Plot configuration.
            status (str): Initial simulation status.
        """

        self.name = os.path.basename(name or "world").split(".")[0]
        self.height = height
        self.width = width
        self.step_time = step_time
        self.sample_time = sample_time
        self.offset = offset

        self.count = 0
        self.sampling = True

        self.x_range = [self.offset[0], self.offset[0] + self.width]
        self.y_range = [self.offset[1], self.offset[1] + self.height]

        self.grid_map, self.obstacle_index, self.obstacle_positions = self.gen_grid_map(
            obstacle_map, mdownsample
        )

        self.plot_parse = plot

        self.status = status

        # Set world parameters
        world_param.step_time = step_time
        world_param.control_mode = control_mode
        world_param.collision_mode = collision_mode

    def step(self):
        """
        Advance the simulation by one step.
        """
        self.count += 1
        self.sampling = self.count % (self.sample_time / self.step_time) == 0

        world_param.time = self.time
        world_param.count = self.count

    def gen_grid_map(self, obstacle_map, mdownsample=1):
        """
        Generate a grid map for obstacles.

        Args:
            obstacle_map: Path to the obstacle map image.
            mdownsample (int): Downsampling factor.

        Returns:
            tuple: Grid map, obstacle indices, and positions.
        """
        abs_obstacle_map = file_check(obstacle_map, root_path=pm.root_path + "/world/map")

        if abs_obstacle_map is not None:
            grid_map = mpimg.imread(abs_obstacle_map)
            grid_map = grid_map[::mdownsample, ::mdownsample]

            if len(grid_map.shape) > 2:
                print("convert to grayscale")
                grid_map = self.rgb2gray(grid_map)

            grid_map = 100 * (1 - grid_map)  # range: 0 - 100
            grid_map = np.fliplr(grid_map.T)

            x_reso = self.width / grid_map.shape[0]
            y_reso = self.height / grid_map.shape[1]
            self.reso = np.array([[x_reso], [y_reso]])

            obstacle_index = np.array(np.where(grid_map > 50))
            obstacle_positions = obstacle_index * self.reso

        else:
            grid_map = None
            obstacle_index = None
            obstacle_positions = None
            self.reso = np.zeros((2, 1))

        return grid_map, obstacle_index, obstacle_positions


    def get_map(self, resolution: float = 0.1, obstacle_list: list = []):
        """
        Get the map of the world with the given resolution.
        """
        return Map(self.width, self.height, resolution, obstacle_list, self.grid_map)


    def reset(self):
        """
        Reset the world simulation.
        """

        world_param.count = 0
        self.count = 0

    @property
    def time(self):
        """
        Get the current simulation time.

        Returns:
            float: Current time based on steps and step_time.
        """
        return self.count * self.step_time

    @property
    def buffer_reso(self):
        """
        Get the maximum resolution of the world.

        Returns:
            float: Maximum resolution.
        """
        return np.max(self.reso)

    def rgb2gray(self, rgb):
        return np.dot(rgb[..., :3], [0.2989, 0.5870, 0.1140])
