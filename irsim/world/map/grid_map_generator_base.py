"""
Abstract base class for grid map generators.

Provides a unified interface (generate(), .grid) and registry for YAML
grid_generator: name + variable parameter list. Subclasses implement
_build_grid() and set name / yaml_param_names to register.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, ClassVar

import matplotlib.pyplot as plt
import numpy as np


class GridMapGenerator(ABC):
    """Abstract base for procedural grid map generators.

    Subclasses must implement _build_grid() and set class attributes
    name (for YAML) and yaml_param_names (allowed constructor params from YAML).
    """

    registry: ClassVar[dict[str, type[GridMapGenerator]]] = {}
    name: str = ""
    yaml_param_names: tuple[str, ...] = ()

    def __init__(self, **kwargs: Any) -> None:
        """Subclasses accept their own parameters via kwargs."""
        self._grid: np.ndarray | None = None

    def __init_subclass__(cls, **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        if getattr(cls, "name", ""):
            GridMapGenerator.registry[cls.name] = cls

    @abstractmethod
    def _build_grid(self) -> np.ndarray:
        """Build the occupancy grid (0-100). Subclasses implement this."""
        ...

    def generate(self) -> GridMapGenerator:
        """Build the grid and return self. Call _build_grid() and set _grid."""
        self._grid = np.asarray(self._build_grid(), dtype=np.float64)
        return self

    @property
    def grid(self) -> np.ndarray:
        """Occupancy grid (0-100). Generates on first access if needed."""
        if self._grid is None:
            self.generate()
        if self._grid is None:
            raise RuntimeError("Grid generation failed: _build_grid() returned None")
        return self._grid

    def save_as_image(self, filepath: str, invert: bool = True) -> None:
        """Save grid as grayscale PNG for use with World.gen_grid_map()."""
        img = self.grid / 100.0
        if invert:
            img = 1.0 - img
        plt.imsave(filepath, img, cmap="gray", vmin=0, vmax=1)

    def preview(
        self,
        title: str = "Grid Map",
        cmap: str = "gray_r",
    ) -> None:
        """Display the occupancy grid with matplotlib."""
        plt.figure(figsize=(8, 8))
        plt.imshow(self.grid.T, origin="lower", cmap=cmap, vmin=0, vmax=100)
        plt.colorbar(label="Occupancy (>50 = obstacle)")
        plt.title(title)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.show()
