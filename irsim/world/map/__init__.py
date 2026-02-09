from __future__ import annotations

import warnings
from typing import Any, Optional, Protocol, runtime_checkable, Union, Tuple, Dict, List

import numpy as np
import shapely
from shapely.geometry import Point

from .grid_map_generator_base import GridMapGenerator
from .image_map_generator import ImageGridGenerator
from .obstacle_map import (
    CELL_CENTER_OFFSET,
    COLLISION_RADIUS_FACTOR,
    OCCUPANCY_THRESHOLD,
    ObstacleMap,
)
from .perlin_map_generator import PerlinGridGenerator

# ---------------------------------------------------------------------------
# Typed protocol - structural contract expected by all path planners
# ---------------------------------------------------------------------------


@runtime_checkable
class EnvGridMap(Protocol):
    """Structural type accepted by all path planners.

    Any object that exposes the attributes below (including :class:`Map`)
    is a valid ``EnvGridMap``.  Planners should annotate their *env_map*
    parameter with this protocol instead of the concrete :class:`Map`
    class to support duck-typed map objects.

    Collision precedence (adopted by every planner):
      1. Grid lookup (O(1) per cell) when *grid* is not ``None``; if the grid
         reports occupied, the point is in collision.
      2. When the grid reports free or is unavailable, Shapely geometry
         intersection with *obstacle_list* is used.  Planners therefore
         combine grid and obstacle_list when both are present.
    """

    width: float
    height: float
    resolution: float
    obstacle_list: list
    grid: Optional[np.ndarray]
    world_offset: Tuple[float, float]

    @property
    def grid_resolution(self) -> Optional[Tuple[float, float]]:
        """Actual cell size ``(x_reso, y_reso)`` derived from *grid* shape and world size."""
        ...

    def grid_occupied(
        self,
        x: float,
        y: float,
        margin_x: float = 0.0,
        margin_y: float = 0.0,
        threshold: float = 50.0,
    ) -> Optional[bool]:
        """Check if any grid cell within the bounding box is occupied."""
        ...

    def is_collision(self, geometry) -> bool:
        """Check collision for a Shapely geometry against the map."""
        ...


def _grid_collision_geometry(
    grid: np.ndarray,
    grid_reso: Tuple[float, float],
    geometry,
    world_offset: Tuple[float, float] = (0.0, 0.0),
) -> bool:
    """Check collision of a Shapely geometry against an occupancy grid.

    Uses the same logic as ObstacleMap.check_grid_collision.
    """
    if grid is None:
        return False

    minx, miny, maxx, maxy = geometry.bounds
    x_reso, y_reso = grid_reso
    offset_x, offset_y = world_offset

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
                cell_x = offset_x + (i + CELL_CENTER_OFFSET) * x_reso
                cell_y = offset_y + (j + CELL_CENTER_OFFSET) * y_reso
                cell_center = Point(cell_x, cell_y)
                if geometry.distance(cell_center) <= collision_radius:
                    return True

    return False


def resolve_obstacle_map(
    obstacle_map: Optional[Union[str, np.ndarray, Dict[str, Any]]] = None,
    world_width: Optional[float] = None,
    world_height: Optional[float] = None,
) -> Optional[np.ndarray]:
    """Resolve obstacle_map to None or a float64 occupancy grid ndarray.

    Accepted types: ``None``, ndarray, or a generator spec **dict** with
    ``name`` (e.g. ``"image"`` or ``"perlin"``).  For backward compatibility,
    a **str** is treated as ``{"name": "image", "path": obstacle_map}``.

    - ``name == "image"``: only ``path`` is required; grid size comes from the image.
    - Other names (e.g. ``"perlin"``): require ``resolution`` and world size
      (``world_width`` / ``world_height``); grid size = world size / resolution.

    Returns:
        None, or ndarray (0-100 grid, dtype float64).
    """
    if obstacle_map is None:
        return None
    if isinstance(obstacle_map, np.ndarray):
        return np.asarray(obstacle_map, dtype=np.float64)
    if isinstance(obstacle_map, str):
        obstacle_map = {"name": "image", "path": obstacle_map}
    if isinstance(obstacle_map, dict) and obstacle_map.get("name"):
        name = obstacle_map.get("name")
        if name == "image":
            path = obstacle_map.get("path")
            if not path:
                raise ValueError("obstacle_map image generator requires 'path'.")
            gen = ImageGridGenerator(path=path).generate()
            return np.asarray(gen.grid, dtype=np.float64)
        if world_width is None or world_height is None:
            raise ValueError(
                "obstacle_map generator spec (non-image) requires world_width and "
                "world_height (passed by World.gen_grid_map)."
            )
        return build_grid_from_generator(
            obstacle_map,
            world_width=world_width,
            world_height=world_height,
        )
    raise TypeError(
        "obstacle_map must be None, an ndarray, or a generator spec dict with 'name'."
    )


def build_grid_from_generator(
    spec: Dict[str, Any],
    world_width: float,
    world_height: float,
) -> np.ndarray:
    """Build a grid map from a YAML grid_generator spec (name + resolution + params).

    Grid size is always computed from world size and ``resolution`` (meters per
    cell): (world_width / resolution, world_height / resolution) cells.

    Args:
        spec: Dict from YAML, e.g. ``{name: perlin, resolution: 0.1, ...}``.
        world_width: World width in meters.
        world_height: World height in meters.

    Returns:
        Occupancy grid (0-100) as float64 ndarray.
    """
    name = spec.get("name")
    if not name or name not in GridMapGenerator.registry:
        known = ", ".join(GridMapGenerator.registry)
        raise ValueError(
            f"Unknown or missing grid_generator name: {name!r}. Known: {known}"
        )
    resolution = spec.get("resolution")
    if resolution is None:
        raise ValueError(
            "obstacle_map generator spec must include 'resolution' (meters per cell)."
        )
    grid_width = max(1, round(float(world_width) / float(resolution)))
    grid_height = max(1, round(float(world_height) / float(resolution)))

    cls = GridMapGenerator.registry[name]
    params = {
        k: v
        for k, v in spec.items()
        if k not in ("name", "resolution") and k in cls.yaml_param_names
    }
    params["width"] = grid_width
    params["height"] = grid_height

    return np.asarray(cls(**params).generate().grid, dtype=np.float64)


class Map:
    """Map data container for navigation / path-planning.

    Satisfies the :class:`EnvGridMap` protocol so that it can be passed to
    any planner that expects ``EnvGridMap``.

    Collision precedence (shared by all planners):
      1. Grid lookup (O(1) per cell) when *grid* is not ``None``.
      2. Shapely geometry intersection when *grid* is unavailable.
    """

    def __init__(
        self,
        width: float = 10,
        height: float = 10,
        resolution: float = 0.1,
        obstacle_list: Optional[list] = None,
        grid: Optional[np.ndarray] = None,
        world_offset: Optional[Union[Tuple[float, float], List[float]]] = None,
    ):
        """
        Initialize the Map.

        Args:
            width: Width of the world (metres).
            height: Height of the world (metres).
            resolution: Planner discretisation cell size (metres/cell).
            obstacle_list: Obstacle objects for Shapely collision detection.
            grid: Occupancy grid (0-100) for grid-based collision detection.
            world_offset: World origin (x, y) for grid indexing. When non-zero,
                geometry and positions are interpreted in world coordinates so
                grid lookups align with ObstacleMap. Default (0, 0).
        """
        if obstacle_list is None:
            obstacle_list = []
        self.width = width
        self.height = height
        self.resolution = resolution
        self.obstacle_list = obstacle_list
        self.grid = grid
        if world_offset is None:
            world_offset = (0.0, 0.0)
        self.world_offset = (float(world_offset[0]), float(world_offset[1]))
        self._obstacles_prepared: bool = False

        # Warn when the user-specified resolution diverges from the actual
        # grid cell size by more than 5 %.
        if grid is not None:
            gr = self.grid_resolution
            if gr is not None:
                gx, _gy = gr
                if abs(resolution - gx) / max(resolution, gx) > 0.05:
                    warnings.warn(
                        f"Map.resolution ({resolution}) differs from grid "
                        f"cell size ({gx:.4f} x {_gy:.4f}). Grid-based "
                        f"planners will use grid_resolution for lookups.",
                        stacklevel=2,
                    )

    @property
    def grid_resolution(self) -> Optional[Tuple[float, float]]:
        """Actual cell size ``(x_reso, y_reso)`` derived from *grid* shape and world size.

        Returns ``None`` when no grid is present.
        """
        if self.grid is None:
            return None
        return (
            self.width / self.grid.shape[0],
            self.height / self.grid.shape[1],
        )

    def grid_occupied(
        self,
        x: float,
        y: float,
        margin_x: float = 0.0,
        margin_y: float = 0.0,
        threshold: float = 50.0,
    ) -> Optional[bool]:
        """Check if any grid cell within the bounding box around ``(x, y)`` is occupied.

        The bounding box extends *margin_x* / *margin_y* (in world metres) in
        each direction.  Grid cells whose occupancy exceeds *threshold* are
        considered occupied.

        Returns:
            ``None`` when no grid is present (caller should fall back to
            Shapely or another collision method).  ``True`` / ``False``
            otherwise.  Points outside the world bounds are treated as
            occupied so planners cannot escape the map.
        """
        if self.grid is None:
            return None
        gr = self.grid_resolution
        if gr is None:
            return None  # defensive; grid is None already caught above
        ox, oy = self.world_offset
        if x < ox or x >= ox + self.width or y < oy or y >= oy + self.height:
            return True  # out-of-bounds: treat as occupied
        rx, ry = gr
        gx = int((x - ox) / rx)
        gy = int((y - oy) / ry)
        rows, cols = self.grid.shape
        mx = max(1, int(np.ceil(margin_x / rx))) if margin_x > 0 else 0
        my = max(1, int(np.ceil(margin_y / ry))) if margin_y > 0 else 0
        return bool(
            np.any(
                self.grid[
                    max(0, gx - mx) : min(rows, gx + mx + 1),
                    max(0, gy - my) : min(cols, gy + my + 1),
                ]
                > threshold
            )
        )

    def is_collision(self, geometry) -> bool:
        """Check collision for a Shapely geometry against grid + obstacles.

        Collision precedence:
          1. Grid lookup when *grid* is not None; if occupied, collision.
          2. When the grid reports free or is unavailable, Shapely geometry
             intersection with *obstacle_list*.
        """
        minx, miny, maxx, maxy = geometry.bounds
        ox, oy = self.world_offset
        if (
            minx < ox
            or miny < oy
            or maxx >= ox + self.width
            or maxy >= oy + self.height
        ):
            return True
        if self.grid is not None:
            gr = self.grid_resolution
            if gr is not None and _grid_collision_geometry(
                self.grid, gr, geometry, world_offset=self.world_offset
            ):
                return True
        if not self.obstacle_list:
            return False
        if not self._obstacles_prepared:
            for obj in self.obstacle_list:
                shapely.prepare(obj._geometry)
            self._obstacles_prepared = True
        return any(shapely.intersects(geometry, obj._geometry) for obj in self.obstacle_list)


__all__ = [
    "EnvGridMap",
    "GridMapGenerator",
    "ImageGridGenerator",
    "Map",
    "ObstacleMap",
    "PerlinGridGenerator",
    "build_grid_from_generator",
    "resolve_obstacle_map",
]
