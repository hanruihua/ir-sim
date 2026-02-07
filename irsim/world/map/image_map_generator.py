"""
Image-based grid map generator.

Loads an obstacle map from an image file (e.g. PNG) and produces a 0-100
occupancy grid. Used when obstacle_map in YAML is a string path (default).
"""

from __future__ import annotations

from typing import Any

import matplotlib.image as mpimg
import numpy as np

from irsim.config.path_param import path_manager as pm
from irsim.util.util import file_check

from .grid_map_generator_base import GridMapGenerator


def _rgb2gray(rgb: np.ndarray) -> np.ndarray:
    """Convert RGB to grayscale for obstacle map images."""
    return np.dot(rgb[..., :3], [0.2989, 0.5870, 0.1140])


class ImageGridGenerator(GridMapGenerator):
    """Load obstacle grid from an image file (path)."""

    name = "image"
    yaml_param_names = ("path",)

    def __init__(self, path: str, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.path = path

    def _build_grid(self) -> np.ndarray:
        """Load image and convert to 0-100 occupancy grid."""
        abs_path = file_check(self.path, root_path=pm.root_path + "/world/map")
        if abs_path is None:
            raise FileNotFoundError(f"Obstacle map image not found: {self.path}")
        grid_map = mpimg.imread(abs_path)
        if len(grid_map.shape) > 2:
            grid_map = _rgb2gray(grid_map)
        if np.issubdtype(grid_map.dtype, np.integer):
            grid_map = grid_map.astype(np.float64) / np.iinfo(grid_map.dtype).max
        elif grid_map.max() > 1.0:
            grid_map = grid_map.astype(np.float64) / grid_map.max()
        else:
            grid_map = grid_map.astype(np.float64)
        grid_map = 100 * (1 - grid_map)
        grid_map = np.fliplr(grid_map.T)
        return grid_map.astype(np.float64)
