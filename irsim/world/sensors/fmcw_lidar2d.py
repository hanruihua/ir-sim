"""Simplified 2D FMCW LiDAR with per-beam radial Doppler velocity."""

from math import cos, sin

import numpy as np
from matplotlib.colors import to_rgb
from mpl_toolkits.mplot3d import Axes3D

from irsim.lib.algorithm.ray_casting_2d import cast_rays
from irsim.util.random import rng
from irsim.world.sensors.lidar2d import Lidar2D


class FMCWLidar2D(Lidar2D):
    """Simulate a 2D FMCW LiDAR using ray intersections plus Doppler projection.

    This sensor keeps the geometric scanning model of :class:`Lidar2D` but replaces
    the optional Cartesian target velocity output with a scalar radial velocity for
    each beam. The radial velocity is defined along the beam direction and can be
    reported either in the sensor-relative frame or motion-compensated to the world
    frame.

    In addition to scan geometry parameters inherited from ``Lidar2D``, this sensor
    accepts:

    Args:
        motion_compensate (bool): When ``False``, report target velocity relative to
            the sensor motion. When ``True``, remove the ego sensor motion and report
            world-frame radial target velocity.
        velocity_noise_std (float): Standard deviation of Gaussian noise applied to
            radial velocity measurements.
        velocity_color (bool): Whether to color beams by radial velocity when
            plotting.
        velocity_color_max (float): Magnitude at which the plotting color saturates.
        velocity_linewidth (float): Line width for valid Doppler beams.
        no_hit_linewidth (float): Line width for beams without valid returns.
        no_hit_alpha (float): Alpha for invalid beams in plots.
        show_velocity_markers (bool): Whether to draw colored endpoint markers for
            valid returns.
        velocity_marker_size (float): Scatter marker size for valid endpoints.
        zero_velocity_color (str): Plot color used near zero radial velocity.
        positive_velocity_color (str): Plot color used for positive radial velocity.
        negative_velocity_color (str): Plot color used for negative radial velocity.
        no_hit_color (str): Plot color used for invalid beams.
    """

    def __init__(
        self,
        state: np.ndarray | None = None,
        obj_id: int = 0,
        motion_compensate: bool = False,
        velocity_noise_std: float = 0.0,
        **kwargs,
    ) -> None:
        super().__init__(state=state, obj_id=obj_id, has_velocity=False, **kwargs)
        self.sensor_type = "fmcw_lidar2d"
        self.motion_compensate = motion_compensate
        self.velocity_noise_std = velocity_noise_std

        # Visualization params: prefer the `plot:` sub-dict (set on the
        # base sensor as self._plot_cfg), fall back to flat top-level
        # keys for backward compatibility, then the default.
        def _pg(key, default):
            return self._plot_cfg.get(key, kwargs.get(key, default))

        self.velocity_color = _pg("velocity_color", True)
        self.velocity_color_max = _pg("velocity_color_max", 2.0)
        self.velocity_linewidth = _pg("velocity_linewidth", 2.5)
        self.no_hit_linewidth = _pg("no_hit_linewidth", 0.8)
        self.no_hit_alpha = _pg("no_hit_alpha", 0.03)
        self.show_velocity_markers = _pg("show_velocity_markers", True)
        self.velocity_marker_size = _pg("velocity_marker_size", 36)
        self.velocity_marker_edge_color = _pg("velocity_marker_edge_color", "black")
        self.velocity_marker_edge_width = _pg("velocity_marker_edge_width", 0.6)
        self.zero_velocity_color = np.array(to_rgb(_pg("zero_velocity_color", "cyan")))
        self.positive_velocity_color = np.array(
            to_rgb(_pg("positive_velocity_color", "crimson"))
        )
        self.negative_velocity_color = np.array(
            to_rgb(_pg("negative_velocity_color", "royalblue"))
        )
        self.no_hit_color = np.array(to_rgb(_pg("no_hit_color", "lightgray")))
        self.radial_velocity = np.zeros(self.number)
        self.valid = np.zeros(self.number, dtype=bool)

    def step(self, state):
        """Update ranges, validity flags, and radial velocities for every beam.

        Beams are cast against obstacle boundary segments in one vectorized pass
        (:func:`irsim.lib.algorithm.ray_casting_2d.cast_rays`), then each valid
        beam's hit object supplies the Doppler (radial) velocity.
        """
        self._state = state
        lidar_geometry = self._world_geometry(state)
        detected_objects = self._get_detected_objects(lidar_geometry)
        ranges, hit_object_indices, origin, directions = cast_rays(
            lidar_geometry,
            detected_objects,
            self.range_max,
        )

        self.range_data[:] = self.range_max
        self.valid[:] = False
        self.radial_velocity[:] = 0.0
        self._resolve_hits(
            ranges,
            hit_object_indices,
            directions,
            detected_objects,
        )
        self._rebuild_scan_geometry(origin, directions)

    def _resolve_hits(
        self,
        ranges,
        hit_object_indices,
        directions,
        detected_objects,
    ):
        """Apply range noise, validity, and radial velocity for each hit beam.

        Draws range noise then (for in-band beams) velocity noise in beam order,
        matching the previous per-beam implementation so a seeded run reproduces
        the old noisy scan bit-for-bit. Ray casting stays vectorized; only this
        cheap arithmetic pass is per-beam, and only over beams that hit.
        """
        for beam in np.nonzero(hit_object_indices >= 0)[0]:
            distance = ranges[beam]
            if self.noise:
                distance += rng.normal(0, self.std)
            # A hit counts only if its (possibly noisy) range stays in band, so
            # range_data/valid stay consistent for get_points.
            if self.range_min <= distance <= self.range_max:
                self.range_data[beam] = distance
                self.valid[beam] = True
                velocity = self._compute_radial_velocity(
                    detected_objects[hit_object_indices[beam]], directions[beam]
                )
                if self.velocity_noise_std > 0:
                    velocity += rng.normal(0, self.velocity_noise_std)
                self.radial_velocity[beam] = velocity

    def _plot(self, ax, state, **kwargs):
        """Plot beams, then color them from radial velocity for visualization."""
        super()._plot(ax, state, **kwargs)
        self._apply_velocity_visuals()
        self._plot_velocity_markers(ax)

    def _step_plot(self):
        """Update beam geometry, then refresh velocity-based colors."""
        super()._step_plot()
        self._apply_velocity_visuals()
        self._update_velocity_markers()

    def _compute_radial_velocity(self, obj, direction: np.ndarray) -> float:
        """Project the target motion onto the beam direction."""
        target_velocity = np.asarray(obj.velocity_xy, dtype=float).reshape(-1)[:2]
        if self.motion_compensate:
            relative_velocity = target_velocity
        else:
            relative_velocity = target_velocity - self._sensor_velocity_xy()
        return float(np.dot(relative_velocity, direction))

    def _sensor_velocity_xy(self) -> np.ndarray:
        """Return the parent object's translational velocity in world XY."""
        if self.parent is None or not hasattr(self.parent, "velocity_xy"):
            return np.zeros(2)
        return np.asarray(self.parent.velocity_xy, dtype=float).reshape(-1)[:2]

    def _apply_velocity_visuals(self):
        """Apply per-beam colors based on radial velocity without changing data."""
        if not self.velocity_color or not hasattr(self, "laser_LineCollection"):
            return

        colors, alphas = self._get_velocity_visuals()
        self.laser_LineCollection.set_color(colors)
        self.laser_LineCollection.set_alpha(alphas)
        self.laser_LineCollection.set_linewidths(self._get_velocity_linewidths())

    def _get_velocity_visuals(self):
        """Return colors and alpha values derived from current radial velocities."""
        scale = max(float(self.velocity_color_max), 1e-6)
        colors = []
        alphas = []

        for is_valid, radial_velocity in zip(
            self.valid, self.radial_velocity, strict=True
        ):
            if not is_valid:
                colors.append(tuple(self.no_hit_color))
                alphas.append(self.no_hit_alpha)
                continue

            ratio = min(abs(float(radial_velocity)) / scale, 1.0)
            if radial_velocity >= 0:
                color = (
                    1.0 - ratio
                ) * self.zero_velocity_color + ratio * self.positive_velocity_color
            else:
                color = (
                    1.0 - ratio
                ) * self.zero_velocity_color + ratio * self.negative_velocity_color
            colors.append(tuple(color))
            alphas.append(1.0)

        return colors, alphas

    def _get_velocity_linewidths(self):
        """Return per-beam linewidths for clearer velocity visualization."""
        return [
            self.velocity_linewidth if is_valid else self.no_hit_linewidth
            for is_valid in self.valid
        ]

    def _plot_velocity_markers(self, ax):
        """Draw colored hit markers at valid beam endpoints for 2D plots."""
        if not self.show_velocity_markers or isinstance(ax, Axes3D):
            return

        points, colors = self._get_velocity_marker_points()
        self.velocity_marker_plot = ax.scatter(
            points[:, 0] if len(points) > 0 else [],
            points[:, 1] if len(points) > 0 else [],
            s=self.velocity_marker_size,
            c=colors if len(colors) > 0 else None,
            edgecolors=self.velocity_marker_edge_color,
            linewidths=self.velocity_marker_edge_width,
            zorder=4,
        )
        self.plot_patch_list.append(self.velocity_marker_plot)

    def _update_velocity_markers(self):
        """Refresh hit marker positions and colors without touching measurements."""
        if not self.show_velocity_markers or not hasattr(self, "velocity_marker_plot"):
            return

        points, colors = self._get_velocity_marker_points()
        self.velocity_marker_plot.set_offsets(points)
        if len(colors) > 0:
            self.velocity_marker_plot.set_facecolor(colors)
        else:
            self.velocity_marker_plot.set_facecolor(np.empty((0, 4)))

    def _get_velocity_marker_points(self):
        """Return world-space hit points and colors for valid beams."""
        if not np.any(self.valid):
            return np.empty((0, 2)), np.empty((0, 3))

        lidar_theta = (
            float(self.lidar_origin[2, 0]) if self.lidar_origin.shape[0] > 2 else 0.0
        )
        origin_xy = np.array([self.lidar_origin[0, 0], self.lidar_origin[1, 0]])
        colors, _ = self._get_velocity_visuals()

        points = []
        point_colors = []
        for is_valid, beam_angle, beam_range, color in zip(
            self.valid,
            self.angle_list,
            self.range_data,
            colors,
            strict=True,
        ):
            if not is_valid:
                continue
            world_angle = lidar_theta + beam_angle
            direction = np.array([cos(world_angle), sin(world_angle)])
            points.append(origin_xy + beam_range * direction)
            point_colors.append(color)

        return np.asarray(points), np.asarray(point_colors)

    def get_scan(self):
        """Get the FMCW scan with radial velocity and validity per beam.

        Returns:
            dict: Scan dictionary containing the standard LiDAR angular metadata plus
                ``ranges``, ``radial_velocity``, ``valid``, and ``intensities=None``.
        """
        scan_data = super().get_scan()
        scan_data.pop("velocity", None)
        scan_data["intensities"] = None
        scan_data["radial_velocity"] = self.radial_velocity
        scan_data["valid"] = self.valid
        return scan_data
