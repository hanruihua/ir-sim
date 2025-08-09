from typing import Optional

import numpy as np

from .obstacle_map import ObstacleMap


class Map:
    def __init__(
        self,
        width: int = 10,
        height: int = 10,
        resolution: float = 0.1,
        obstacle_list: Optional[list] = None,
        grid: Optional[np.ndarray] = None,
    ):
        """
        Map class for storing map data and navigation information

        Args:
            width (int): width of the map
            height (int): height of the map
            resolution (float): resolution of the map
            obstacle_list (list): list of obstacle objects for collision detection
            grid (np.ndarray): grid map data for collision detection.
        """

        if obstacle_list is None:
            obstacle_list = []
        self.width = width
        self.height = height
        self.resolution = resolution
        self.obstacle_list = obstacle_list
        self.grid = grid


__all__ = ["Map", "ObstacleMap"]
