from typing import Any, Optional

import numpy as np
from shapely.geometry import Point
from shapely.strtree import STRtree

from irsim.world.object_base import ObjectBase

# Grid-based collision detection constants
# Occupancy threshold: grid values > this are considered obstacles (0-100 range)
OCCUPANCY_THRESHOLD = 50
# Offset to convert grid index to cell center (0.5 = center of cell)
CELL_CENTER_OFFSET = 0.5
# Collision radius as fraction of cell size (0.5 = half cell width)
COLLISION_RADIUS_FACTOR = 0.5


class ObstacleMap(ObjectBase):
    def __init__(
        self,
        shape: Optional[dict] = None,
        color: str = "k",
        static: bool = True,
        grid_map: Optional[np.ndarray] = None,
        grid_reso: Optional[np.ndarray] = None,
        world_offset: Optional[list[float]] = None,
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
        self.grid_reso = grid_reso if grid_reso is not None else np.array([[1.0], [1.0]])
        self.world_offset = world_offset if world_offset is not None else [0.0, 0.0]

    def check_grid_collision(self, geometry) -> bool:
        """Check collision using grid array lookup.

        Uses a two-phase approach:
        1. Quick bounding box check for early rejection
        2. Precise check: verify if geometry actually intersects occupied cells

        Args:
            geometry: Shapely geometry object to check collision for.

        Returns:
            bool: True if collision detected, False otherwise.
        """
        if self.grid_map is None:
            return False

        # Get bounding box of the geometry
        minx, miny, maxx, maxy = geometry.bounds

        # Convert world coordinates to grid indices
        x_reso = self.grid_reso[0, 0]
        y_reso = self.grid_reso[1, 0]
        offset_x, offset_y = self.world_offset

        # Calculate grid indices (clamp to valid range)
        i_min = max(0, int((minx - offset_x) / x_reso))
        i_max = min(self.grid_map.shape[0] - 1, int((maxx - offset_x) / x_reso))
        j_min = max(0, int((miny - offset_y) / y_reso))
        j_max = min(self.grid_map.shape[1] - 1, int((maxy - offset_y) / y_reso))

        if i_min > i_max or j_min > j_max:
            return False

        # Check each occupied cell in bounding box region
        collision_radius = max(x_reso, y_reso) * COLLISION_RADIUS_FACTOR

        for i in range(i_min, i_max + 1):
            for j in range(j_min, j_max + 1):
                if self.grid_map[i, j] > OCCUPANCY_THRESHOLD:
                    # Get cell center in world coordinates
                    cell_x = offset_x + (i + CELL_CENTER_OFFSET) * x_reso
                    cell_y = offset_y + (j + CELL_CENTER_OFFSET) * y_reso
                    cell_center = Point(cell_x, cell_y)

                    # Check if geometry is within or exactly at the collision radius of the cell center
                    if geometry.distance(cell_center) <= collision_radius:
                        return True

        return False
