from math import pi, cos, sin
import numpy as np
from shapely import MultiLineString, Point, is_valid, prepare
from irsim.util.util import (
    geometry_transform,
    transform_point_with_state,
    get_transform,
)
from irsim.global_param import env_param
from matplotlib.collections import LineCollection
from shapely.strtree import STRtree
from shapely.ops import unary_union
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection

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
        blind_angles (list): List of angle ranges to be ignored, e.g. [ (-0.5, 0.5), (0.6, 0.7)]
        **kwargs: Additional arguments.
            color (str): Color of the sensor.

    Attr:
        - sensor_type (str): Type of sensor ("lidar2d"). Default is "lidar2d". 
        - range_min (float): Minimum detection range in meters. Default is 0.
        - range_max (float): Maximum detection range in meters. Default is 10.
        - angle_range (float): Total angle range of the sensor in radians. Default is pi.
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
        state: np.ndarray = None,
        obj_id: int = 0,
        range_min: float = 0,
        range_max: float = 10,
        angle_range: float = pi,
        number: int = 100,
        scan_time: float = 0.1,
        noise: bool = False,
        std: float = 0.2,
        angle_std: float = 0.02,
        offset: list[float] = [0, 0, 0],
        alpha: float = 0.3,
        has_velocity: bool = False,
        blind_angles: list[tuple] = None,
        **kwargs,
    ) -> None:
        """
        Initialize the Lidar2D sensor.

        
        """
        self.sensor_type = "lidar2d"

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
        prepare(new_geometry)

        new_geometry, intersect_indices = self.laser_geometry_process(new_geometry)

        if len(intersect_indices) == 0:
            self._geometry = new_geometry
            self.calculate_range()
        else:
            origin_point = Point(self.lidar_origin[0, 0], self.lidar_origin[1, 0])
            filtered_geoms = [
                g for g in new_geometry.geoms if g.intersects(origin_point)
            ]
            self._geometry = MultiLineString(filtered_geoms)
            self.calculate_range_vel(intersect_indices)

    def laser_geometry_process(self, lidar_geometry):

        '''
        Find the intersected objects and return the intersected indices with the lidar geometry
        
        Args:
            lidar_geometry (shapely.geometry.MultiLineString): The geometry of the lidar.

        Returns:
            list: The indices of the intersected objects.
        '''
        
        object_tree = env_param.GeometryTree
        objects = env_param.objects
        geometries = [obj._geometry for obj in objects]

        potential_geometries_index = object_tree.query(lidar_geometry)

        geometries_to_subtract = []
        intersect_indices = []

        for geom_index in potential_geometries_index:
            geo = geometries[geom_index]
            obj = objects[geom_index]

            if obj._id == self.obj_id or not is_valid(obj._geometry) or obj.unobstructed:
                continue

            if obj.shape == 'map':
                potential_intersections = obj.geometry_tree.query(lidar_geometry)
                filtered_lines = [obj.linestrings[i] for i in potential_intersections]
                filtered_multi_lines = MultiLineString(filtered_lines)
                # prepare(filtered_multi_lines)

                if lidar_geometry.intersects(filtered_multi_lines):
                    geometries_to_subtract.append(filtered_multi_lines)
                    intersect_indices.append(geom_index)

            else:
                if lidar_geometry.intersects(geo):
                    geometries_to_subtract.append(geo)
                    intersect_indices.append(geom_index)

        if geometries_to_subtract:
            merged_geometry = unary_union(geometries_to_subtract)
            lidar_geometry = lidar_geometry.difference(merged_geometry)

        return lidar_geometry, intersect_indices

    def calculate_range(self):
        """
        Calculate the range data from the current geometry.
        """
        for index, l in enumerate(self._geometry.geoms):
            # self.range_data[index] = l.length
            if self.noise:
                self.range_data[index] = l.length + np.random.normal(0, self.std)
            else:
                self.range_data[index] = l.length

    def calculate_range_vel(self, intersect_index):
        """
        Calculate the range data and velocities from intersected geometries.

        Args:
            intersect_index (list): List of intersected object indices.
        """
        for index, l in enumerate(self._geometry.geoms):
            # self.range_data[index] = l.length
            self.range_data[index] = (
                l.length + np.random.normal(0, self.std) if self.noise else l.length
            )

            if self.has_velocity:
                if l.length < self.range_max - 0.02:
                    for index_obj in intersect_index:
                        obj = env_param.objects[index_obj]
                        if obj.geometry.distance(l) < 0.1:
                            self.velocity[:, index : index + 1] = obj.velocity_xy
                            break

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

    def plot(self, ax, **kwargs):
        """
        Plot the Lidar's detected lines on a given axis.
        """
        self.init_plot(ax, **kwargs)


    def init_plot(self, ax, **kwargs):
        """
        Plot the Lidar's detected lines on a given axis.

        Args:
            ax: Matplotlib axis.
            **kwargs: Plotting options.
        """
        lines = []

        for i in range(self.number):
            x = self.range_data[i] * cos(self.angle_list[i])
            y = self.range_data[i] * sin(self.angle_list[i])

            position = self.lidar_origin[0:2, 0] 
            trans, rot = get_transform(self.lidar_origin)
            range_end_position = rot @ np.array([[x], [y]]) + trans

            if isinstance(ax, Axes3D):
                position = np.array([position[0], position[1], 0])
                end_position = np.array(
                    [range_end_position[0, 0], range_end_position[1, 0], 0]
                )
                segment = [position, end_position]
            else:
                segment = [position, range_end_position[0:2, 0]]

            lines.append(segment)

        if isinstance(ax, Axes3D):
            self.laser_LineCollection = Line3DCollection(
                lines, linewidths=1, colors=self.color, alpha=self.alpha, zorder=3
            )
            ax.add_collection3d(self.laser_2D_lines)
        else:
            self.laser_LineCollection = LineCollection(
                lines, linewidths=1, colors=self.color, alpha=self.alpha, zorder=3
            )
            ax.add_collection(self.laser_LineCollection)

        self.plot_patch_list.append(self.laser_LineCollection)
    

    def step_plot(self):
        """
        Update the plot for the Lidar.
        """
        ax = self.laser_LineCollection.axes
        lines = []

        for i in range(self.number):
            x = self.range_data[i] * cos(self.angle_list[i])
            y = self.range_data[i] * sin(self.angle_list[i])

            position = self.lidar_origin[0:2, 0] 
            trans, rot = get_transform(self.lidar_origin)
            range_end_position = rot @ np.array([[x], [y]]) + trans

            if isinstance(ax, Axes3D):
                position = np.array([position[0], position[1], 0])
                end_position = np.array(
                    [range_end_position[0, 0], range_end_position[1, 0], 0]
                )
                segment = [position, end_position]
            else:
                segment = [position, range_end_position[0:2, 0]]

            lines.append(segment)

        self.laser_LineCollection.set_segments(lines)

    def set_laser_color(self, laser_indices, laser_color: str = 'blue', alpha: float = 0.3):

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

        point_array = np.hstack(point_cloud)

        return point_array
    