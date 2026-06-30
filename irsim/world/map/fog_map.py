from __future__ import annotations

import numpy as np

from irsim.world.map import Map


class FogMap(Map):
    """Fog-of-map overlay grid, built on :class:`Map`.

    Every cell starts *unexplored* (covered by fog). A robot's lidar reveals
    cells along each beam's line of sight, so the explored region grows as the
    robot navigates. The overlay is rendered as a grey layer that is opaque
    where unexplored and transparent where explored, so the underlying obstacle
    map shows through only once an area has been seen (free space appears white,
    obstacles black, the unknown stays grey).

    The explored mask is updated regardless of whether a display is active, so it
    is also usable headless as an exploration observation or coverage metric.
    """

    def __init__(
        self,
        width: float = 10,
        height: float = 10,
        resolution: float = 0.1,
        world_offset: tuple[float, float] | list[float] | None = None,
    ) -> None:
        """Initialize the fog grid.

        Args:
            width: Width of the world (metres) — matches the world it covers.
            height: Height of the world (metres).
            resolution: Fog cell size (metres/cell). Finer is smoother but
                heavier; coarser is faster.
            world_offset: World origin ``(x, y)`` for cell indexing.
        """
        nx = max(1, round(width / resolution))
        ny = max(1, round(height / resolution))
        super().__init__(
            width=width,
            height=height,
            resolution=resolution,
            world_offset=world_offset,
        )
        # False = unexplored (fog), True = revealed.
        self.explored = np.zeros((nx, ny), dtype=bool)
        # Actual cell size derived from the rounded grid shape.
        self._rx = width / nx
        self._ry = height / ny
        # Matplotlib artist for the overlay (created lazily in _init_plot).
        self._im = None
        # Cached RGBA buffer for rendering (RGB is constant; only alpha changes).
        self._rgba = None
        # Set when a new cell is revealed; lets _step_plot skip the costly image
        # upload on renders where nothing changed (e.g. revisiting seen area).
        self._dirty = True

    @property
    def shape(self) -> tuple[int, int]:
        """Grid shape ``(nx, ny)`` (cells along x and y)."""
        return self.explored.shape

    @property
    def explored_ratio(self) -> float:
        """Fraction of the world revealed so far, in ``[0, 1]``."""
        return float(self.explored.mean())

    def reset(self) -> None:
        """Re-cover the whole world with fog."""
        self.explored[:] = False
        self._dirty = True

    def reveal_from_lidar(
        self,
        origin: np.ndarray | list[float],
        angles: np.ndarray | list[float],
        ranges: np.ndarray | list[float],
    ) -> None:
        """Reveal cells along each lidar beam's line of sight.

        Args:
            origin: Lidar world pose ``[x, y, theta]`` (theta optional).
            angles: Per-beam angles in the lidar's local frame (radians).
            ranges: Per-beam measured ranges (metres), aligned with ``angles``.
        """
        origin = np.asarray(origin, dtype=float).ravel()
        angles = np.asarray(angles, dtype=float).ravel()
        ranges = np.asarray(ranges, dtype=float).ravel()
        if angles.size == 0 or ranges.size == 0:
            return

        x0, y0 = origin[0], origin[1]
        theta = origin[2] if origin.size > 2 else 0.0
        world_ang = theta + angles

        max_range = float(ranges.max())
        if max_range <= 0:
            return

        # Sample each ray every half-cell so no cell along it is skipped, then
        # clamp every beam's samples to its own measured range.
        sample_step = 0.5 * min(self._rx, self._ry)
        n_samples = int(max_range / sample_step) + 2
        t = np.arange(n_samples, dtype=float) * sample_step
        dist = np.minimum(t[None, :], ranges[:, None])
        xs = x0 + dist * np.cos(world_ang)[:, None]
        ys = y0 + dist * np.sin(world_ang)[:, None]
        self._mark_cells(xs.ravel(), ys.ravel())

    def reveal_fov(
        self,
        origin: np.ndarray | list[float],
        fov: float,
        fov_radius: float,
    ) -> None:
        """Reveal every cell within a field-of-view sector (no occlusion).

        Used when a sensing object has no lidar: each cell within ``fov_radius``
        of the origin and within ``±fov/2`` of the object's heading is revealed.

        Args:
            origin: Object world pose ``[x, y, theta]`` (theta optional).
            fov: Full field-of-view angle in radians (``2*pi`` is a full circle).
            fov_radius: View range in metres.
        """
        origin = np.asarray(origin, dtype=float).ravel()
        radius = float(fov_radius)
        if radius <= 0 or fov <= 0:
            return
        x0, y0 = origin[0], origin[1]
        theta = origin[2] if origin.size > 2 else 0.0

        ox, oy = self.world_offset
        nx, ny = self.explored.shape
        gx0 = max(0, int((x0 - radius - ox) / self._rx))
        gx1 = min(nx, int((x0 + radius - ox) / self._rx) + 1)
        gy0 = max(0, int((y0 - radius - oy) / self._ry))
        gy1 = min(ny, int((y0 + radius - oy) / self._ry) + 1)
        if gx0 >= gx1 or gy0 >= gy1:
            return

        cx = ox + (np.arange(gx0, gx1) + 0.5) * self._rx
        cy = oy + (np.arange(gy0, gy1) + 0.5) * self._ry
        dx = cx[:, None] - x0
        dy = cy[None, :] - y0
        dist_sq = dx * dx + dy * dy
        mask = dist_sq <= radius * radius

        # Restrict to the sector. For fov <= pi the angle test reduces to a dot
        # product against the heading (|bearing| <= fov/2 iff the projection onto
        # the heading is >= cos(fov/2) * distance), avoiding a per-cell arctan2.
        if fov < 2 * np.pi:
            half = 0.5 * fov
            cos_half = np.cos(half)
            proj = dx * np.cos(theta) + dy * np.sin(theta)
            if cos_half >= 0.0:  # fov <= pi (the common case)
                mask &= (proj >= 0.0) & (proj * proj >= cos_half * cos_half * dist_sq)
            else:  # pi < fov < 2*pi: rare, fall back to the exact angle
                bearing = np.abs(
                    (np.arctan2(dy, dx) - theta + np.pi) % (2 * np.pi) - np.pi
                )
                mask &= bearing <= half

        li, lj = np.nonzero(mask)
        gx, gy = gx0 + li, gy0 + lj
        if gx.size and not self.explored[gx, gy].all():
            self._dirty = True
        self.explored[gx, gy] = True

    def _mark_cells(self, xs: np.ndarray, ys: np.ndarray) -> None:
        """Mark the cells covering world points ``(xs, ys)`` as explored
        (flagging dirty when any of them are newly revealed)."""
        ox, oy = self.world_offset
        gx = np.floor((xs - ox) / self._rx).astype(int)
        gy = np.floor((ys - oy) / self._ry).astype(int)
        nx, ny = self.explored.shape
        valid = (gx >= 0) & (gx < nx) & (gy >= 0) & (gy < ny)
        gx, gy = gx[valid], gy[valid]
        if gx.size and not self.explored[gx, gy].all():
            self._dirty = True
        self.explored[gx, gy] = True

    def to_rgba(
        self,
        color: tuple[float, float, float] = (0.78, 0.78, 0.80),
        alpha: float = 1.0,
    ) -> np.ndarray:
        """RGBA image of the fog for rendering.

        Unexplored cells get ``color`` at ``alpha`` (a soft light grey, opaque
        by default so the fog still fully hides the map until seen); explored
        cells are fully transparent so the underlying map shows through. Shape
        is ``(nx, ny, 4)`` (transpose the first two axes for ``imshow``).

        The buffer is allocated once and reused: the constant RGB channels are
        filled on the first call, and only the alpha channel is recomputed (the
        only thing that changes as cells are revealed).
        """
        if self._rgba is None or self._rgba.shape[:2] != self.explored.shape:
            self._rgba = np.empty((*self.explored.shape, 4), dtype=float)
            self._rgba[..., 0] = color[0]
            self._rgba[..., 1] = color[1]
            self._rgba[..., 2] = color[2]
        # Only alpha changes: 0 where explored (transparent), `alpha` where fogged.
        self._rgba[..., 3] = np.where(self.explored, 0.0, alpha)
        return self._rgba

    # ------------------------------------------------------------------
    # Plotting (mirrors ObjectBase._init_plot / _step_plot / plot_clear)
    # ------------------------------------------------------------------
    def _init_plot(self, ax, zorder: int = 2, **kwargs) -> None:
        """Create the fog overlay artist on ``ax``, or refresh it if it exists.

        Drawn above obstacles (zorder 1) but below robots (zorder 3) so unseen
        areas are hidden while the robot stays visible. The axes is passed in, so
        the map layer stays free of a direct matplotlib import. When called again
        (e.g. on env.reset()) the existing artist is reused and just refreshed,
        so resets don't stack imshow artists.
        """
        if self._im is None:
            ox, oy = self.world_offset
            self._im = ax.imshow(
                self.to_rgba().transpose(1, 0, 2),
                origin="lower",
                extent=[ox, ox + self.width, oy, oy + self.height],
                zorder=zorder,
                **kwargs,
            )
            self._dirty = False  # image now reflects the current fog
        else:
            self._step_plot()

    def _step_plot(self) -> None:
        """Refresh the overlay image data after the fog has been revealed.

        Skips the (O(pixels)) image upload when no new cell was revealed since
        the last render.
        """
        if self._im is None or not self._dirty:
            return
        self._im.set_data(self.to_rgba().transpose(1, 0, 2))
        self._dirty = False

    def plot_clear(self) -> None:
        """Remove the fog overlay artist."""
        if self._im is not None:
            self._im.remove()
            self._im = None
