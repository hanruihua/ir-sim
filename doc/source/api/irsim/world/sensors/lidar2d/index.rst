irsim.world.sensors.lidar2d
===========================

.. py:module:: irsim.world.sensors.lidar2d


Classes
-------

.. autoapisummary::

   irsim.world.sensors.lidar2d.Lidar2D


Module Contents
---------------

.. py:class:: Lidar2D(state: numpy.ndarray = None, obj_id: int = 0, range_min: float = 0, range_max: float = 10, angle_range: float = pi, number: int = 100, scan_time: float = 0.1, noise: bool = False, std: float = 0.2, angle_std: float = 0.02, offset: list[float] = [0, 0, 0], alpha: float = 0.3, has_velocity: bool = False, **kwargs)

   Simulates a 2D Lidar sensor for detecting obstacles in the environment.

   :param state: Initial state of the sensor.
   :type state: np.ndarray
   :param obj_id: ID of the associated object.
   :type obj_id: int
   :param range_min: Minimum detection range.
   :type range_min: float
   :param range_max: Maximum detection range.
   :type range_max: float
   :param angle_range: Total angle range of the sensor.
   :type angle_range: float
   :param number: Number of laser beams.
   :type number: int
   :param scan_time: Time taken for one complete scan.
   :type scan_time: float
   :param noise: Whether noise is added to measurements.
   :type noise: bool
   :param std: Standard deviation for range noise.
   :type std: float
   :param angle_std: Standard deviation for angle noise.
   :type angle_std: float
   :param offset: Offset of the sensor from the object's position.
   :type offset: list
   :param alpha: Transparency for plotting.
   :type alpha: float
   :param has_velocity: Whether the sensor measures velocity.
   :type has_velocity: bool
   :param \*\*kwargs: Additional arguments.
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

   Initialize the Lidar2D sensor.




   .. py:attribute:: sensor_type
      :value: 'lidar2d'



   .. py:attribute:: range_min
      :value: 0



   .. py:attribute:: range_max
      :value: 10



   .. py:attribute:: angle_range
      :value: 3.141592653589793



   .. py:attribute:: angle_min
      :value: -1.5707963267948966



   .. py:attribute:: angle_max
      :value: 1.5707963267948966



   .. py:attribute:: angle_inc
      :value: 0.031415926535897934



   .. py:attribute:: number
      :value: 100



   .. py:attribute:: scan_time
      :value: 0.1



   .. py:attribute:: noise
      :value: False



   .. py:attribute:: std
      :value: 0.2



   .. py:attribute:: angle_std
      :value: 0.02



   .. py:attribute:: offset


   .. py:attribute:: alpha
      :value: 0.3



   .. py:attribute:: has_velocity
      :value: False



   .. py:attribute:: velocity


   .. py:attribute:: time_inc
      :value: 0.0005



   .. py:attribute:: range_data


   .. py:attribute:: angle_list


   .. py:attribute:: color


   .. py:attribute:: obj_id
      :value: 0



   .. py:attribute:: plot_patch_list
      :value: []



   .. py:attribute:: plot_line_list
      :value: []



   .. py:attribute:: plot_text_list
      :value: []



   .. py:method:: init_geometry(state)

      Initialize the Lidar's scanning geometry.

      :param state: Current state of the sensor.
      :type state: np.ndarray



   .. py:method:: step(state)

      Update the Lidar's state and process intersections with environment objects.

      :param state: New state of the sensor.
      :type state: np.ndarray



   .. py:method:: laser_geometry_process(lidar_geometry)

      Find the intersected objects and return the intersected indices with the lidar geometry

      :param lidar_geometry: The geometry of the lidar.
      :type lidar_geometry: shapely.geometry.MultiLineString

      :returns: The indices of the intersected objects.
      :rtype: list



   .. py:method:: calculate_range()

      Calculate the range data from the current geometry.



   .. py:method:: calculate_range_vel(intersect_index)

      Calculate the range data and velocities from intersected geometries.

      :param intersect_index: List of intersected object indices.
      :type intersect_index: list



   .. py:method:: get_scan()

      Get the 2D lidar scan data. refer to the ros topic scan: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html

      :returns: Scan data including angles, ranges, and velocities.
      :rtype: dict



   .. py:method:: get_points()

      Convert scan data to a point cloud.

      :returns: Point cloud (2xN).
      :rtype: np.ndarray



   .. py:method:: get_offset()

      Get the sensor's offset.

      :returns: Offset as a list.
      :rtype: list



   .. py:method:: plot(ax, state: Optional[numpy.ndarray] = None, **kwargs)

      Plot the Lidar's detected lines on a given axis.



   .. py:property:: state
      :type: numpy.ndarray


      Get the current state of the lidar sensor.

      :returns: Current state of the sensor.
      :rtype: np.ndarray


   .. py:method:: step_plot()

      Public method to update the lidar visualization, calls _step_plot.



   .. py:method:: set_laser_color(laser_indices, laser_color: str = 'blue', alpha: float = 0.3)

      Set a specific color of the selected lasers.

      :param laser_indices: The indices of the lasers to set the color.
      :type laser_indices: list
      :param laser_color: The color to set the selected lasers. Default is 'blue'.
      :type laser_color: str
      :param alpha: The transparency of the lasers. Default is 0.3.
      :type alpha: float



   .. py:method:: plot_clear()

      Clear the plot elements from the axis.



   .. py:method:: scan_to_pointcloud()

      Convert the Lidar scan data to a point cloud.

      :returns: Point cloud (2xN).
      :rtype: np.ndarray



