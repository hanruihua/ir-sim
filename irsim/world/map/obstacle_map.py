from typing import Any

import numpy as np
import shapely
from shapely.geometry import MultiLineString, Point
from shapely.strtree import STRtree

from irsim.world.object_base import ObjectBase

# Grid-based collision detection constants
# Occupancy threshold: grid values > this are considered obstacles (0-100 range)
OCCUPANCY_THRESHOLD = 50
# Offset to convert grid index to cell center (0.5 = center of cell)
CELL_CENTER_OFFSET = 0.5
# Collision radius as fraction of cell size (0.5 = half cell width)
COLLISION_RADIUS_FACTOR = 0.5


def grid_collision_geometry(
    grid: np.ndarray | None,
    grid_reso: tuple[float, float],
    geometry,
    world_offset: tuple[float, float] = (0.0, 0.0),
) -> bool:
    """Check collision of a Shapely geometry against an occupancy grid.

    Uses a two-phase approach:
    1. Quick bounding box check for early rejection
    2. Precise check: verify if geometry actually intersects occupied cells

    Args:
        grid: Occupancy grid, or None when the map carries no grid.
        grid_reso: Cell size ``(x_reso, y_reso)`` in meters.
        geometry: Shapely geometry object to check collision for.
        world_offset: World coordinates of the grid origin.

    Returns:
        bool: True if collision detected, False otherwise.
    """
    if grid is None:
        return False

    minx, miny, maxx, maxy = geometry.bounds
    x_reso, y_reso = grid_reso
    offset_x, offset_y = world_offset

    # Convert world coordinates to grid indices (clamped to valid range)
    i_min = max(0, int((minx - offset_x) / x_reso))
    i_max = min(grid.shape[0] - 1, int((maxx - offset_x) / x_reso))
    j_min = max(0, int((miny - offset_y) / y_reso))
    j_max = min(grid.shape[1] - 1, int((maxy - offset_y) / y_reso))

    if i_min > i_max or j_min > j_max:
        return False

    collision_radius = max(x_reso, y_reso) * COLLISION_RADIUS_FACTOR

    for i in range(i_min, i_max + 1):
        for j in range(j_min, j_max + 1):
            if grid[i, j] > OCCUPANCY_THRESHOLD:
                # Geometry within the collision radius of an occupied cell center
                cell_x = offset_x + (i + CELL_CENTER_OFFSET) * x_reso
                cell_y = offset_y + (j + CELL_CENTER_OFFSET) * y_reso
                if geometry.distance(Point(cell_x, cell_y)) <= collision_radius:
                    return True

    return False


class ObstacleMap(ObjectBase):
    """Static obstacle object backed by map line segments and optional grid data."""

    def __init__(
        self,
        shape: dict | None = None,
        color: str = "k",
        static: bool = True,
        grid_map: np.ndarray | None = None,
        grid_reso: np.ndarray | None = None,
        world_offset: list[float] | None = None,
        **kwargs: Any,
    ) -> None:
        """Create an obstacle map object from a set of line segments.

        Args:
            shape (dict | None): Map shape configuration with keys like
                ``{"name": "map", "reso": float, "points": array}``.
            color (str): Display color. Default "k".
            static (bool): Whether the object is static. Default True.
            grid_map (np.ndarray | None): Grid map array for fast collision detection.
            grid_reso (np.ndarray | None): Resolution [x_reso, y_reso] of the grid.
            world_offset (list | None): World offset [x, y].
            **kwargs: Forwarded to ``ObjectBase`` constructor.
        """
        if shape is None:
            shape = {"name": "map", "reso": "0.1", "points": None}
        super().__init__(
            shape=shape,
            role="obstacle",
            color=color,
            static=static,
            **kwargs,
        )

        self.linestrings = list(self.geometry.geoms)
        self.geometry_tree = STRtree(self.linestrings)

        # Grid-based collision detection data
        self.grid_map = grid_map
        self.grid_reso = (
            grid_reso if grid_reso is not None else np.array([[1.0], [1.0]])
        )
        self.world_offset = world_offset if world_offset is not None else [0.0, 0.0]

    def check_grid_collision(self, geometry) -> bool:
        """Check collision using grid array lookup.

        Args:
            geometry: Shapely geometry object to check collision for.

        Returns:
            bool: True if collision detected, False otherwise.
        """
        return grid_collision_geometry(
            self.grid_map,
            (self.grid_reso[0, 0], self.grid_reso[1, 0]),
            geometry,
            self.world_offset,
        )

    def is_collision(self, geometry) -> bool:
        """Check collision against grid (if present) and map geometry."""
        if self.grid_map is not None and self.check_grid_collision(geometry):
            return True

        candidate_indices = self.geometry_tree.query(geometry)
        filtered_lines = [self.linestrings[i] for i in candidate_indices]
        if not filtered_lines:
            return False
        filtered_multi_line = MultiLineString(filtered_lines)
        return shapely.intersects(geometry, filtered_multi_line)
