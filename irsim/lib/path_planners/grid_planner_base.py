"""Shared grid geometry for the occupancy-grid planners (A*, JPS)."""

from __future__ import annotations

import contextlib

import numpy as np

from irsim.world.map import EnvGridMap


class GridPlannerBase:
    """Bounds, resolution, and grid shape derived from an environment map.

    Both :class:`~irsim.lib.path_planners.a_star.AStarPlanner` and
    :class:`~irsim.lib.path_planners.jps.JPSPlanner` search the same discrete
    grid, so they derive their search space the same way.
    """

    def __init__(self, env_map: EnvGridMap) -> None:
        """
        Derive the search grid from an environment map.

        Args:
            env_map: Environment map (any :class:`~irsim.world.map.EnvGridMap`
                compatible object).
        """
        self._map = env_map
        self.obstacle_list = env_map.obstacle_list[:]

        off = np.asarray(env_map.world_offset, dtype=float).flatten()
        self.origin_x = float(off[0])
        self.origin_y = float(off[1])
        self.min_x, self.min_y = 0, 0  # grid indices are 0-based
        self.max_x = self.origin_x + env_map.width
        self.max_y = self.origin_y + env_map.height

        # When map has a grid, use its actual resolution and shape so planner grid
        # matches collision lookups (avoids "Open set is empty" on resolution mismatch).
        grid = getattr(env_map, "grid", None)
        gr = None
        if grid is not None and hasattr(env_map, "grid_resolution"):
            with contextlib.suppress(Exception):
                gr = env_map.grid_resolution

        if grid is not None and gr is not None:
            self.resolution = gr[0]  # m/cell; assume square cells (gr[0]==gr[1])
            self.x_width = grid.shape[0]
            self.y_width = grid.shape[1]
        else:
            self.resolution = env_map.resolution
            self.x_width = round((self.max_x - self.origin_x) / self.resolution)
            self.y_width = round((self.max_y - self.origin_y) / self.resolution)
