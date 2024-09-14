from math import pi, cos, sin
import numpy as np
from shapely import MultiLineString, GeometryCollection, Point, is_valid, make_valid
from irsim.util.util import geometry_transform, transform_point_with_state
from irsim.global_param import env_param
from shapely import get_coordinates
from matplotlib.collections import LineCollection


class Lidar2D:
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




        alpha: for transparency

        """

        self.sensor_type = "lidar"

        self.range_min = range_min
        self.range_max = range_max

        self.angle_range = angle_range
        self.angle_min = -angle_range / 2
        self.angle_max = angle_range / 2
        self.angle_inc = angle_range / number  #

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

        self.time_inc = (angle_range / (2 * pi)) * scan_time / number  #
        self.range_data = range_max * np.ones(
            number,
        )
        self.angle_list = np.linspace(self.angle_min, self.angle_max, num=number)

        self._state = state
        self.init_geometry(self._state)

        self.color = kwargs.get("color", "r")

        self.obj_id = obj_id

        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

    def init_geometry(self, state):

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

        self._state = state

        self.lidar_origin = transform_point_with_state(self.offset, self._state)
        new_geometry = geometry_transform(self._init_geometry, self._state)

        # geo_list = [obj._geometry for obj in env_param.objects if self.obj_id != obj._id]
        # object_geometries = GeometryCollection(geo_list)
        # # new_diff_geometry = new_geometry.difference(object_geometries)
        # new_geometry = new_geometry.difference(env_param.objects[-1]._geometry)
        # map_geo = env_param.objects[-1]._geometry
        # new_geometry = new_geometry.difference(map_geo)
        # temp = env_param.objects[-1]._geometry.difference(new_geometry)

        intersect_index = []

        for ind, obj in enumerate(env_param.objects):
            if self.obj_id != obj._id:
                if new_geometry.intersects(obj._geometry):

                    # if not is_valid(obj._geometry):
                    #     make_valid(obj._geometry)

                    if not is_valid(obj._geometry):
                        # print('geometry of obj is not valid')
                        continue

                    new_geometry = new_geometry.difference(obj._geometry)
                    intersect_index.append(ind)

        if len(intersect_index) == 0:
            self._geometry = new_geometry
            self.calculate_range()
        else:
            # coord = get_coordinates(new_geometry)
            # distances = np.linalg.norm(coord[::2]- self.lidar_origin[0:2, 0], axis=1)
            # filtered_indices = np.where(distances < 0.001)[0]
            # filtered_points = [coord[2 * index: 2 * index + 2, :] for index in filtered_indices]
            # self._geometry = MultiLineString(filtered_points)
            # self.calculate_range_vel(intersect_index)
            # ranges = np.array([g.length for g in new_geometry.geoms])
            # filtered_indices = np.where(ranges < 0.001)[0]
            origin_point = Point(self.lidar_origin[0, 0], self.lidar_origin[1, 0])
            filtered_geoms = [
                g for g in new_geometry.geoms if g.intersects(origin_point)
            ]
            self._geometry = MultiLineString(filtered_geoms)
            self.calculate_range_vel(intersect_index)

    def calculate_range(self):

        # coord = get_coordinates(self._geometry)

        for index, l in enumerate(self._geometry.geoms):
            self.range_data[index] = l.length

        # for index, point in enumerate(coord[1::2]):
        #     point = np.c_[point]

        #     distance = np.round(np.linalg.norm(point - self.lidar_origin[0:2, 0]), 3)

        #     self.range_data[index] = distance

    def calculate_range_vel(self, intersect_index):

        for index, l in enumerate(self._geometry.geoms):
            self.range_data[index] = l.length

            if self.has_velocity:
                if l.length < self.range_max - 0.02:
                    for index_obj in intersect_index:
                        obj = env_param.objects[index_obj]
                        # p = Point(point[0, 0], point[1, 0])
                        if obj.geometry.distance(l) < 0.1:
                            self.velocity[:, index : index + 1] = obj.velocity_xy
                            break

        # coord = get_coordinates(self._geometry)

        # for index, point in enumerate(coord[1::2]):
        #     point = np.c_[point]

        #     distance = np.round(np.linalg.norm(point - self.lidar_origin[0:2, 0]), 3)

        #     self.range_data[index] = distance

        #     if self.has_velocity:
        #         if distance < self.range_max - 0.02:
        #             for index_obj in intersect_index:
        #                 obj = env_param.objects[index_obj]

        #                 p = Point(point[0, 0], point[1, 0])

        #                 if obj.geometry.distance(p) < 0.1:
        #                     self.velocity[:, index:index+1] = obj.velocity_xy
        #                     break

    def get_scan(self):
        """
        Get the 2D lidar scan data

        Return:
            scan_data: dict, refer to the ros topic scan: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html

                angle_min: float, start angle of the scan [rad]
                angle_max: float, end angle of the scan [rad]
                angle_increment: float, angular distance between measurements [rad]
                time_increment: float, time between measurements [s]
                scan_time: float, time between scans [s]
                range_min: float, minimum range value [m]
                range_max: float, maximum range value [m]
                ranges: list of float, range data [m]
                intensities: intensity data, None
                velocity: 2*number matrix, x,y velocity  data [m/s]
        """

        # reference: ros topic -- scan: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
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
        return self.scan_to_pointcloud()

    def get_offset(self):
        return np.squeeze(self.offset).tolist()

    def plot(self, ax, **kwargs):

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

        [patch.remove() for patch in self.plot_patch_list]
        [line.pop(0).remove() for line in self.plot_line_list]
        [text.remove() for text in self.plot_text_list]

        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

    def scan_to_pointcloud(self):
        """
        return poit cloud: (2, n)
        """

        point_cloud = []

        ranges = self.range_data
        angles = np.linspace(self.angle_min, self.angle_max, len(ranges))

        # trans, R = get_transform(self._state)

        for i in range(len(ranges)):
            scan_range = ranges[i]
            angle = angles[i]

            if scan_range < (self.range_max - 0.02):
                point = np.array([[scan_range * cos(angle)], [scan_range * sin(angle)]])
                point_cloud.append(point)

        if len(point_cloud) == 0:
            return None

        point_array = np.hstack(point_cloud)

        # point_coords = R @ point_array + trans

        return point_array
