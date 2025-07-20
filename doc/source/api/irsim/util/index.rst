irsim.util
==========

.. py:module:: irsim.util

.. autoapi-nested-parse::

   Utility functions for IR-SIM simulation.

   This package contains helper functions for:
   - Mathematical operations
   - Coordinate transformations
   - File operations
   - Geometry utilities



Submodules
----------

.. toctree::
   :maxdepth: 1

   /api/irsim/util/util/index


Functions
---------

.. autoapisummary::

   irsim.util.file_check
   irsim.util.find_file
   irsim.util.WrapToPi
   irsim.util.WrapToRegion
   irsim.util.relative_position
   irsim.util.transform_point_with_state
   irsim.util.geometry_transform
   irsim.util.vertices_transform
   irsim.util.omni_to_diff
   irsim.util.diff_to_omni
   irsim.util.time_it
   irsim.util.time_it2
   irsim.util.cross_product
   irsim.util.is_convex_and_ordered
   irsim.util.gen_inequal_from_vertex
   irsim.util.distance
   irsim.util.dist_hypot
   irsim.util.random_point_range
   irsim.util.is_2d_list


Package Contents
----------------

.. py:function:: file_check(file_name, root_path=None)

   Check whether a file exists and return its absolute path.

   :param file_name: Name of the file to check.
   :type file_name: str
   :param root_path: Root path to use if the file is not found.
   :type root_path: str, optional

   :returns: Absolute path of the file if found.
   :rtype: str

   :raises FileNotFoundError: If the file is not found.


.. py:function:: find_file(root_path, target_filename)

.. py:function:: WrapToPi(rad, positive=False)

   The function `WrapToPi` transforms an angle in radians to the range [-pi, pi].

   :param rad: Angle in radians.
               The `rad` parameter in the `WrapToPi` function represents an angle in radians that you want to
   :type rad: float
   :param transform to the range [-π:
   :param π]. The function ensures that the angle is within this range by wrapping:
   :param it around if it exceeds the bounds.:
   :param positive: Whether to return the positive value of the angle. Useful for angles difference.
   :type positive: bool

   :returns: The function `WrapToPi(rad)` returns the angle `rad` wrapped to the range [-pi, pi].


.. py:function:: WrapToRegion(rad, range)

   Transform an angle to a defined range, with length of 2*pi.

   :param rad: Angle in radians.
   :type rad: float
   :param range: List defining the range [min, max].
   :type range: list

   :returns: Wrapped angle.
   :rtype: float


.. py:function:: relative_position(position1, position2, topi=True)

   Calculate the relative position and angle between two points.

   :param position1: First position [x, y] (2x1).
   :type position1: np.array
   :param position2: Second position [x, y] (2x1).
   :type position2: np.array
   :param topi: Whether to wrap angle to [-pi, pi] (default True).
   :type topi: bool

   :returns: Distance and angle (radians).
   :rtype: tuple


.. py:function:: transform_point_with_state(point, state)

   Transform a point using a state.

   :param point: Point [x, y, theta] (3x1).
   :type point: np.array
   :param state: State [x, y, theta] (3x1).
   :type state: np.array

   :returns: Transformed point (2x1).
   :rtype: np.array


.. py:function:: geometry_transform(geometry, state)

   Transform geometry using a state.

   :param geometry: Shapely geometry to transform.
   :param state: [xoff, yoff, theta]
   :type state: np.array or sequence of 3 floats

   :returns: Transformed geometry.

   shapely expects [a, b, d, e, xoff, yoff] for:
   x' = a*x + b*y + xoff
   y' = d*x + e*y + yoff


.. py:function:: vertices_transform(vertices, state)

   Transform vertices using a state.

   :param vertices: Vertices of the object. (2xN)
   :type vertices: np.array
   :param state: State [x, y, theta] (3x1).
   :type state: np.array

   :returns: Transformed vertices.
   :rtype: np.array


.. py:function:: omni_to_diff(state_ori, vel_omni, w_max=1.5, guarantee_time=0.2, tolerance=0.1, mini_speed=0.02)

   Convert omnidirectional velocity to differential velocity.

   :param state_ori: Orientation angle.
   :type state_ori: float
   :param vel_omni: Omnidirectional velocity [vx, vy] (2x1).
   :type vel_omni: np.array
   :param w_max: Maximum angular velocity.
   :type w_max: float
   :param guarantee_time: Time to guarantee velocity.
   :type guarantee_time: float
   :param tolerance: Angular tolerance.
   :type tolerance: float
   :param mini_speed: Minimum speed threshold.
   :type mini_speed: float

   :returns: Differential velocity [linear, angular] (2x1).
   :rtype: np.array


.. py:function:: diff_to_omni(state_ori, vel_diff)

   Convert differential velocity to omnidirectional velocity.

   :param state_ori: Orientation angle.
   :type state_ori: float
   :param vel_diff: Differential velocity [linear, angular] (2x1).
   :type vel_diff: np.array

   :returns: Omnidirectional velocity [vx, vy] (2x1).
   :rtype: np.array


.. py:function:: time_it(name='Function')

   Decorator to measure function execution time.

   :param name: Function name for logging (default "Function").
   :type name: str
   :param print: Whether to print execution time (default True).
   :type print: bool

   :returns: Wrapped function with timing.
   :rtype: function


.. py:function:: time_it2(name='Function')

   Decorator to measure function execution time with instance attribute check.

   :param name: Function name for logging (default "Function").
   :type name: str

   :returns: Wrapped function with timing.
   :rtype: function


.. py:function:: cross_product(o, a, b)

   Compute the cross product of vectors OA and OB.

   :param o: Points representing vectors.
   :type o: array-like
   :param a: Points representing vectors.
   :type a: array-like
   :param b: Points representing vectors.
   :type b: array-like

   :returns: Cross product value.
   :rtype: float


.. py:function:: is_convex_and_ordered(points)

   Determine if the polygon is convex and return the order (CW or CCW).

   :param points: A 2xN NumPy array representing the vertices of the polygon.
   :type points: np.ndarray

   :returns:

             A tuple where the first element is True if the polygon is convex,
                           and the second element is 'CW' or 'CCW' based on the order.
                           If not convex, returns (False, None).
   :rtype: (bool, str)


.. py:function:: gen_inequal_from_vertex(vertex: numpy.ndarray)

   Generate inequality constraints for a convex polygon.

   :param vertex: Vertices of the polygon (2xN).
   :type vertex: np.array

   :returns: G matrix and h vector for the inequality Gx <= h.
   :rtype: tuple


.. py:function:: distance(point1, point2)

   Compute the distance between two points.

   :param point1: First point [x, y] (2x1).
   :type point1: np.array
   :param point2: Second point [x, y] (2x1).
   :type point2: np.array

   :returns: Distance between points.
   :rtype: float


.. py:function:: dist_hypot(x1, y1, x2, y2)

.. py:function:: random_point_range(range_low=[0, 0, -pi], range_high=[10, 10, pi])

   Generate a random point within a range.

   :param range_low: Lower bound of the range.
   :type range_low: list
   :param range_high: Upper bound of the range.
   :type range_high: list

   :returns: Random point within the range.
   :rtype: np.array


.. py:function:: is_2d_list(data: list) -> bool

   Returns True if 'data' is a non-empty list of lists (or tuples), indicating a 2D structure.
   Returns False if 'data' is a single list


