from math import pi, cos, sin
import numpy as np
from shapely import MultiLineString, GeometryCollection, Point, is_valid, make_valid
from irsim.util.util import geometry_transform, transform_point_with_state
from irsim.global_param import env_param
from shapely import get_coordinates
from matplotlib.collections import LineCollection


class Lidar2D:
    """
    Simulates a 2D Lidar sensor for detecting obstacles in the environment.

    Attributes:
        sensor_type (str): Type of sensor ("lidar").
        range_min (float): Minimum detection range.
        range_max (float): Maximum detection range.
        angle_range (float): Total angle range of the sensor.
        number (int): Number of laser beams.
        scan_time (float): Time taken for one complete scan.
        noise (bool): Whether noise is added to measurements.
        std (float): Standard deviation for range noise.
        angle_std (float): Standard deviation for angle noise.
        offset (np.ndarray): Offset of the sensor from the object's position.
        alpha (float): Transparency for plotting.
        has_velocity (bool): Whether the sensor measures velocity.
    """

    def __init__(
        self,
        state=None,
        obj_id=0,
        range_min=0,
        range_max=10,
        angle_range=pi,
        number=100,
        scan_time=0.1,
        noise=False,
        std=0.2,
        angle_std=0.02,
        offset=[0, 0, 0],
        alpha=0.3,
        has_velocity=False,
        **kwargs,
    ) -> None:
        """
        Initialize the Lidar2D sensor.

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
        """
        self.sensor_type = "lidar"

        self.range_min = range_min
        self.range_max = range_max

        self.angle_range = angle_range
        self.angle_min = -angle_range / 2
        self.angle_max = angle_range / 2
        self.angle_inc = angle_range / number

        self.number = number
        self.scan_time = scan_time
        self.noise = noise
        self.std = std
        self.angle_std = angle_std
        self.offset = np.c_[offset]
        self.lidar_origin = self.offset

        self.alpha = alpha
        self.has_velocity = has_velocity
        self.velocity = np.zeros((2, number))

        self.time_inc = (angle_range / (2 * pi)) * scan_time / number
        self.range_data = range_max * np.ones(number)
        self.angle_list = np.linspace(self.angle_min, self.angle_max, num=number)

        self._state = state
        self.init_geometry(self._state)

        self.color = kwargs.get("color", "r")

        self.obj_id = obj_id

        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

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

        self._init_geometry = MultiLineString(segment_point_list)
        self._init_geometry = geometry_transform(self._init_geometry, self.offset)
        self.lidar_origin = transform_point_with_state(self.offset, state)
        self._geometry = geometry_transform(self._init_geometry, state)

    def step(self, state):
        """
        Update the Lidar's state and process intersections with environment objects.

        Args:
            state (np.ndarray): New state of the sensor.
        """
        self._state = state

        self.lidar_origin = transform_point_with_state(self.offset, self._state)
        new_geometry = geometry_transform(self._init_geometry, self._state)

        intersect_index = []

        for ind, obj in enumerate(env_param.objects):
            if self.obj_id != obj._id:
                if new_geometry.intersects(obj._geometry):
                    if not is_valid(obj._geometry):
                        continue

                    new_geometry = new_geometry.difference(obj._geometry)
                    intersect_index.append(ind)

        if len(intersect_index) == 0:
            self._geometry = new_geometry
            self.calculate_range()
        else:
            origin_point = Point(self.lidar_origin[0, 0], self.lidar_origin[1, 0])
            filtered_geoms = [
                g for g in new_geometry.geoms if g.intersects(origin_point)
            ]
            self._geometry = MultiLineString(filtered_geoms)
            self.calculate_range_vel(intersect_index)

    def calculate_range(self):
        """
        Calculate the range data from the current geometry.
        """
        for index, l in enumerate(self._geometry.geoms):
            self.range_data[index] = l.length

    def calculate_range_vel(self, intersect_index):
        """
        Calculate the range data and velocities from intersected geometries.

        Args:
            intersect_index (list): List of intersected object indices.
        """
        for index, l in enumerate(self._geometry.geoms):
            self.range_data[index] = l.length

            if self.has_velocity:
                if l.length < self.range_max - 0.02:
                    for index_obj in intersect_index:
                        obj = env_param.objects[index_obj]
                        if obj.geometry.distance(l) < 0.1:
                            self.velocity[:, index : index + 1] = obj.velocity_xy
                            break

    def get_scan(self):
        """
        Get the 2D lidar scan data.

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

    def plot(self, ax, **kwargs):
        """
        Plot the Lidar's detected lines on a given axis.

        Args:
            ax: Matplotlib axis.
            **kwargs: Plotting options.
        """
        coord = get_coordinates(self._geometry)

        lines = []

        for i in range(0, len(coord), 2):
            segment = [coord[i], coord[i + 1]]
            lines.append(segment)

        line_segments = LineCollection(
            lines, linewidths=1, colors="red", alpha=self.alpha, zorder=0
        )

        ax.add_collection(line_segments)

        self.plot_patch_list.append(line_segments)

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

        point_array = np.hstack(point_cloud)

        return point_array
