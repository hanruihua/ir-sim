from math import cos, pi, sin
from typing import TYPE_CHECKING

import matplotlib.transforms as mtransforms
import numpy as np
import shapely
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from shapely import MultiLineString

from irsim.lib.algorithm.ray_casting_2d import cast_rays
from irsim.util.random import rng
from irsim.util.util import (
    WrapTo2Pi,
    geometry_transform,
    transform_point_with_state,
)

if TYPE_CHECKING:
    from irsim.world.object_base import ObjectBase


class Lidar2D:
    """
    Simulates a 2D Lidar sensor for detecting obstacles in the environment.

    Args:
        state (np.ndarray): Initial state of the sensor.
        obj_id (int): ID of the associated object.
        range_min (float): Minimum detection range.
        range_max (float): Maximum detection range.
        angle_range (float): Total angle range of the sensor.
        number (int): Number of laser beams.
        scan_time (float): Time taken for one complete scan.
        noise (bool): Whether noise is added to measurements.
        std (float): Standard deviation for range noise.
        angle_std (float): Standard deviation for angle noise.
        offset (list): Offset of the sensor from the object's position.
        alpha (float): Transparency for plotting.
        has_velocity (bool): Whether the sensor measures velocity.
        **kwargs: Additional arguments.
            color (str): Color of the sensor.

    Attr:
        - sensor_type (str): Type of sensor ("lidar2d"). Default is "lidar2d".
        - range_min (float): Minimum detection range in meters. Default is 0.
        - range_max (float): Maximum detection range in meters. Default is 10.
        - angle_range (float): Total angle range of the sensor in radians. Default is pi. WrapTo2Pi is applied.
        - angle_min (float): Starting angle of the sensor's scan relative to the forward direction in radians. Calculated as -angle_range / 2.
        - angle_max (float): Ending angle of the sensor's scan relative to the forward direction in radians. Calculated as angle_range / 2.
        - angle_inc (float): Angular increment between each laser beam in radians. Calculated as angle_range / number.
        - number (int): Number of laser beams. Default is 100.
        - scan_time (float): Time taken to complete one full scan in seconds. Default is 0.1.
        - noise (bool): Whether to add noise to the measurements. Default is False.
        - std (float): Standard deviation for range noise in meters. Effective only if `noise` is True. Default is 0.2.
        - angle_std (float): Standard deviation for angle noise in radians. Effective only if `noise` is True. Default is 0.02.
        - offset (np.ndarray): Offset of the sensor relative to the object's position, formatted as [x, y, theta]. Default is [0, 0, 0].
        - lidar_origin (np.ndarray): Origin position of the Lidar sensor, considering offset and the object's state.
        - alpha (float): Transparency level for plotting the laser beams. Default is 0.3.
        - has_velocity (bool): Whether the sensor measures the velocity of detected points. Default is False.
        - velocity (np.ndarray): Velocity data for each laser beam, formatted as (2, number) array. Effective only if `has_velocity` is True. Initialized to zeros.
        - time_inc (float): Time increment for each scan, simulating the sensor's time resolution. Default is 5e-4.
        - range_data (np.ndarray): Array storing range data for each laser beam. Initialized to `range_max` for all beams.
        - angle_list (np.ndarray): Array of angles corresponding to each laser beam, distributed linearly from `angle_min` to `angle_max`.
        - color (str): Color of the sensor's representation in visualizations. Default is "r" (red).
        - obj_id (int): ID of the associated object, used to differentiate between multiple sensors or objects in the environment. Default is 0.
        - plot_patch_list (list): List storing plot patches (e.g., line collections) for visualization purposes.
        - plot_line_list (list): List storing plot lines for visualization purposes.
        - plot_text_list (list): List storing plot text elements for visualization purposes.
    """

    def __init__(
        self,
        state: np.ndarray | None = None,
        obj_id: int = 0,
        range_min: float = 0,
        range_max: float = 10,
        angle_range: float = pi,
        number: int = 100,
        scan_time: float = 0.1,
        noise: bool = False,
        std: float = 0.2,
        angle_std: float = 0.02,
        offset: list[float] | None = None,
        alpha: float = 0.3,
        has_velocity: bool = False,
        **kwargs,
    ) -> None:
        """
        Initialize the Lidar2D sensor.


        """
        if offset is None:
            offset = [0, 0, 0]
        self.sensor_type = "lidar2d"

        self.range_min = range_min
        self.range_max = range_max

        self.angle_range = WrapTo2Pi(angle_range)
        self.angle_min = -self.angle_range / 2
        self.angle_max = self.angle_range / 2
        self.angle_inc = self.angle_range / number

        self.number = number
        self.scan_time = scan_time
        self.noise = noise
        self.std = std
        self.angle_std = angle_std
        self.offset = np.c_[offset]

        # Visualization params may be given under a `plot:` sub-dict
        # (preferred) or as flat top-level keys (backward compatible).
        _plot = kwargs.get("plot") or {}
        self._plot_cfg = _plot
        self.alpha = _plot.get("alpha", alpha)
        self.has_velocity = has_velocity
        self.velocity = np.zeros((2, number))

        self.time_inc = (self.angle_range / (2 * pi)) * scan_time / number
        self.range_data = range_max * np.ones(number)

        self.angle_list = np.linspace(self.angle_min, self.angle_max, num=number)

        self._state = state
        self.init_geometry(self._state)

        self.color = _plot.get("color", kwargs.get("color", "r"))

        self.obj_id = obj_id

        # Parent object reference (set by ObjectBase or SensorFactory)
        self.parent: ObjectBase | None = None

        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

    @property
    def _env_param(self):
        """Access env_param via parent's env instance if available."""
        if self.parent is not None and self.parent._env is not None:
            return self.parent._env._env_param
        from irsim.config import env_param

        return env_param

    def init_geometry(self, state):
        """
        Initialize the Lidar's scanning geometry.

        Args:
            state (np.ndarray): Current state of the sensor.
        """
        segment_point_list = []

        for i in range(self.number):
            x = self.range_data[i] * cos(self.angle_list[i])
            y = self.range_data[i] * sin(self.angle_list[i])

            point0 = np.zeros((1, 2))
            point = np.array([[x], [y]]).T

            segment = np.concatenate((point0, point), axis=0)

            segment_point_list.append(segment)

        self.origin_state = self.offset
        geometry = MultiLineString(segment_point_list)
        self._original_geometry = geometry_transform(geometry, self.origin_state)
        self.lidar_origin = transform_point_with_state(self.offset, state)

        self._geometry = geometry_transform(self._original_geometry, state)
        self._init_geometry = self._geometry

    def step(self, state: np.ndarray) -> None:
        """
        Update the Lidar's state and compute per-beam ranges via ray casting.

        Each beam is intersected analytically against the boundary segments of
        nearby obstacles (polygons, linestrings, and map segments); the nearest
        hit along the beam is its range. This reproduces the previous geometry
        ``difference`` result to floating-point precision when the sensor origin
        is in free space, while avoiding the expensive GEOS overlay.

        Args:
            state (np.ndarray): New state of the sensor.
        """
        self._state = state

        lidar_geometry = self._world_geometry(state)
        env_param = self._env_param
        objects = env_param.objects
        ranges, hit_objects, origin, directions = cast_rays(
            lidar_geometry,
            objects,
            env_param.GeometryTree,
            self.obj_id,
            self.range_max,
        )

        if self.noise:
            self.range_data[:] = ranges + rng.normal(0, self.std, self.number)
        else:
            self.range_data[:] = ranges

        self._rebuild_scan_geometry(origin, directions)

        if self.has_velocity:
            self._assign_velocities(hit_objects, objects)

    def _world_geometry(self, state: np.ndarray) -> MultiLineString:
        """Build the max-range beam geometry in world coordinates."""
        self.lidar_origin = transform_point_with_state(self.offset, state)
        return geometry_transform(self._original_geometry, state)

    def _rebuild_scan_geometry(
        self, origin: np.ndarray, directions: np.ndarray
    ) -> None:
        """Rebuild each clipped beam from its origin and measured range."""
        endpoints = origin + self.range_data[:, None] * directions
        origins = np.broadcast_to(origin, endpoints.shape)
        beam_coordinates = np.stack([origins, endpoints], axis=1)
        self._geometry = shapely.multilinestrings(
            shapely.linestrings(beam_coordinates),
        )

    def _assign_velocities(
        self,
        hit_objects: np.ndarray,
        objects,
    ) -> None:
        """Assign velocity when ray casting reports an actual object hit.

        ``hit_objects`` distinguishes a hit from a max-range miss directly, so
        no range margin is needed near ``range_max``.
        """
        self.velocity[:] = 0.0
        for beam_index in np.flatnonzero(hit_objects >= 0):
            object_velocity = objects[hit_objects[beam_index]].velocity_xy
            self.velocity[:, beam_index : beam_index + 1] = object_velocity

    def get_scan(self):
        """
        Get the 2D lidar scan data. refer to the ros topic scan: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html

        Returns:
            dict: Scan data including angles, ranges, and velocities.
        """
        scan_data = {}
        scan_data["angle_min"] = self.angle_min
        scan_data["angle_max"] = self.angle_max
        scan_data["angle_increment"] = self.angle_inc
        scan_data["time_increment"] = self.time_inc
        scan_data["scan_time"] = self.scan_time
        scan_data["range_min"] = self.range_min
        scan_data["range_max"] = self.range_max
        scan_data["ranges"] = self.range_data
        scan_data["intensities"] = None
        scan_data["velocity"] = self.velocity

        return scan_data

    def get_points(self):
        """
        Convert scan data to a point cloud.

        Returns:
            np.ndarray: Point cloud (2xN).
        """
        return self.scan_to_pointcloud()

    def get_offset(self):
        """
        Get the sensor's offset.

        Returns:
            list: Offset as a list.
        """
        return np.squeeze(self.offset).tolist()

    def plot(self, ax, state: np.ndarray | None = None, **kwargs):
        """
        Plot the Lidar's detected lines on a given axis.
        """
        if state is None:
            state = self.state

        self._plot(ax, state, **kwargs)

    def _init_plot(self, ax, **kwargs):
        """
        Initialize the plot for the Lidar.
        """
        self._plot(ax, self.origin_state, **kwargs)

    @property
    def state(self) -> np.ndarray:
        """
        Get the current state of the lidar sensor.

        Returns:
            np.ndarray: Current state of the sensor.
        """
        return self._state

    def _plot(self, ax, state, **kwargs):
        """
        Plot the Lidar's detected lines using the specified state for positioning.
        Creates line segments in local coordinates and applies transforms to position them.

        Args:
            ax: Matplotlib axis.
            state: State vector [x, y, theta, ...] defining lidar position and orientation.
            **kwargs: Plotting options.
        """
        lines = []

        if isinstance(ax, Axes3D):
            # For 3D plotting, calculate actual world coordinates since transforms don't work the same way
            if state is not None and len(state) > 0:
                # Calculate lidar position based on object state and sensor offset
                lidar_x = self.lidar_origin[0, 0]
                lidar_y = self.lidar_origin[1, 0]
                lidar_theta = (
                    self.lidar_origin[2, 0] if self.lidar_origin.shape[0] > 2 else 0
                )
            else:
                lidar_x, lidar_y, lidar_theta = 0, 0, 0

            # Create line segments in world coordinates for 3D
            for i in range(self.number):
                x_local = self.range_data[i] * cos(self.angle_list[i])
                y_local = self.range_data[i] * sin(self.angle_list[i])

                # Transform to world coordinates
                x_world = (
                    lidar_x + x_local * cos(lidar_theta) - y_local * sin(lidar_theta)
                )
                y_world = (
                    lidar_y + x_local * sin(lidar_theta) + y_local * cos(lidar_theta)
                )

                start_point = np.array([lidar_x, lidar_y, 0])
                end_point = np.array([x_world, y_world, 0])
                segment = [start_point, end_point]
                lines.append(segment)

            self.laser_LineCollection = Line3DCollection(
                lines, linewidths=1, colors=self.color, alpha=self.alpha, zorder=2
            )
            ax.add_collection3d(self.laser_LineCollection)
        else:
            # For 2D plotting, create line segments in local coordinates and use transforms
            for i in range(self.number):
                x = self.range_data[i] * cos(self.angle_list[i])
                y = self.range_data[i] * sin(self.angle_list[i])
                segment = [np.array([0, 0]), np.array([x, y])]
                lines.append(segment)

            self.laser_LineCollection = LineCollection(
                lines, linewidths=1, colors=self.color, alpha=self.alpha, zorder=2
            )
            ax.add_collection(self.laser_LineCollection)

            # Apply transform for 2D case - use provided state for positioning
            if state is not None and len(state) > 0:
                lidar_x = self.lidar_origin[0, 0]
                lidar_y = self.lidar_origin[1, 0]
                lidar_theta = (
                    self.lidar_origin[2, 0] if self.lidar_origin.shape[0] > 2 else 0
                )

                # Create transform: rotate by lidar orientation, then translate to lidar position
                trans = (
                    mtransforms.Affine2D()
                    .rotate(lidar_theta)
                    .translate(lidar_x, lidar_y)
                    + ax.transData
                )
                self.laser_LineCollection.set_transform(trans)

        self.plot_patch_list.append(self.laser_LineCollection)

    def _step_plot(self):
        """
        Update the lidar visualization using matplotlib transforms based on current state.
        Creates line segments in local coordinates and applies transform to position them.
        """
        if not hasattr(self, "laser_LineCollection"):
            return

        ax = self.laser_LineCollection.axes
        lines = []

        if ax is None:
            return

        if isinstance(ax, Axes3D):
            # For 3D plotting, calculate actual world coordinates
            lidar_x = self.lidar_origin[0, 0]
            lidar_y = self.lidar_origin[1, 0]
            lidar_theta = (
                self.lidar_origin[2, 0] if self.lidar_origin.shape[0] > 2 else 0
            )

            # Create line segments in world coordinates for 3D
            for i in range(self.number):
                x_local = self.range_data[i] * cos(self.angle_list[i])
                y_local = self.range_data[i] * sin(self.angle_list[i])

                # Transform to world coordinates
                x_world = (
                    lidar_x + x_local * cos(lidar_theta) - y_local * sin(lidar_theta)
                )
                y_world = (
                    lidar_y + x_local * sin(lidar_theta) + y_local * cos(lidar_theta)
                )

                start_point = np.array([lidar_x, lidar_y, 0])
                end_point = np.array([x_world, y_world, 0])
                segment = [start_point, end_point]
                lines.append(segment)
        else:
            # For 2D plotting, create line segments in local coordinates
            for i in range(self.number):
                x = self.range_data[i] * cos(self.angle_list[i])
                y = self.range_data[i] * sin(self.angle_list[i])
                segment = [np.array([0, 0]), np.array([x, y])]
                lines.append(segment)

        # Update line segments
        self.laser_LineCollection.set_segments(lines)

        # Apply transform to position the LineCollection based on current lidar origin (2D only)
        if not isinstance(ax, Axes3D):  # 2D case
            lidar_x = self.lidar_origin[0, 0]
            lidar_y = self.lidar_origin[1, 0]
            lidar_theta = (
                self.lidar_origin[2, 0] if self.lidar_origin.shape[0] > 2 else 0
            )

            # Create transform: rotate by lidar orientation, then translate to lidar position
            trans = (
                mtransforms.Affine2D().rotate(lidar_theta).translate(lidar_x, lidar_y)
                + ax.transData
            )
            self.laser_LineCollection.set_transform(trans)

    def step_plot(self):
        """
        Public method to update the lidar visualization, calls _step_plot.
        """
        self._step_plot()

    def set_laser_color(
        self, laser_indices, laser_color: str = "blue", alpha: float = 0.3
    ):
        """
        Set a specific color of the selected lasers.

        Args:
            laser_indices (list): The indices of the lasers to set the color.
            laser_color (str): The color to set the selected lasers. Default is 'blue'.
            alpha (float): The transparency of the lasers. Default is 0.3.
        """

        current_color = [self.color] * self.number
        current_alpha = [self.alpha] * self.number

        for index in laser_indices:
            if index < self.number:
                current_color[index] = laser_color
                current_alpha[index] = alpha

        self.laser_LineCollection.set_color(current_color)
        self.laser_LineCollection.set_alpha(current_alpha)

    def plot_clear(self):
        """
        Clear the plot elements from the axis.
        """
        [patch.remove() for patch in self.plot_patch_list]
        [line.pop(0).remove() for line in self.plot_line_list]
        [text.remove() for text in self.plot_text_list]

        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

    def scan_to_pointcloud(self):
        """
        Convert the Lidar scan data to a point cloud.

        Returns:
            np.ndarray: Point cloud (2xN).
        """
        point_cloud = []

        ranges = self.range_data
        angles = np.linspace(self.angle_min, self.angle_max, len(ranges))

        for i in range(len(ranges)):
            scan_range = ranges[i]
            angle = angles[i]

            if scan_range < (self.range_max - 0.02):
                point = np.array([[scan_range * cos(angle)], [scan_range * sin(angle)]])
                point_cloud.append(point)

        if len(point_cloud) == 0:
            return None

        return np.hstack(point_cloud)
