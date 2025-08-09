irsim.util.util
===============

.. py:module:: irsim.util.util


Functions
---------

.. autoapisummary::

   irsim.util.util.file_check
   irsim.util.util.find_file
   irsim.util.util.WrapToPi
   irsim.util.util.WrapTo2Pi
   irsim.util.util.WrapToRegion
   irsim.util.util.convert_list_length
   irsim.util.util.convert_list_length_dict
   irsim.util.util.is_list_of_dicts
   irsim.util.util.is_list_of_numbers
   irsim.util.util.is_list_of_lists
   irsim.util.util.is_list_not_list_of_lists
   irsim.util.util.relative_position
   irsim.util.util.get_transform
   irsim.util.util.transform_point_with_state
   irsim.util.util.get_affine_transform
   irsim.util.util.geometry_transform
   irsim.util.util.vertices_transform
   irsim.util.util.omni_to_diff
   irsim.util.util.diff_to_omni
   irsim.util.util.time_it
   irsim.util.util.time_it2
   irsim.util.util.cross_product
   irsim.util.util.is_convex_and_ordered
   irsim.util.util.gen_inequal_from_vertex
   irsim.util.util.distance
   irsim.util.util.dist_hypot
   irsim.util.util.random_point_range
   irsim.util.util.is_2d_list


Module Contents
---------------

.. py:function:: file_check(file_name: Optional[str], root_path: Optional[str] = None) -> Optional[str]

   Check whether a file exists and return its absolute path.

   :param file_name: Name of the file to check.
   :type file_name: str
   :param root_path: Root path to use if the file is not found.
   :type root_path: str, optional

   :returns: Absolute path of the file if found.
   :rtype: str

   :raises FileNotFoundError: If the file is not found.


.. py:function:: find_file(root_path: str, target_filename: str) -> str

.. py:function:: WrapToPi(rad: float, positive: bool = False) -> float

   The function `WrapToPi` transforms an angle in radians to the range [-pi, pi].

   :param rad: Angle in radians. The `rad` parameter in the `WrapToPi` function represents an angle in radians that you want to transform to the range `[-π, π]`. The function ensures that the angle is within this range by wrapping it around if it exceeds the bounds.
   :type rad: float
   :param positive: Whether to return the positive value of the angle. Useful for angles difference.
   :type positive: bool

   :returns: The function `WrapToPi(rad)` returns the angle `rad` wrapped to the range [-pi, pi].


.. py:function:: WrapTo2Pi(rad: float) -> float

   The function `WrapTo2Pi` transforms an angle in radians to the range [0, 2pi].

   :param rad: Angle in radians.
               The `rad` parameter in the `WrapTo2Pi` function represents an angle in radians that you want to transform to the range `[0, 2pi]`. The function ensures that the angle is within this range by wrapping it around if it exceeds the bounds.
   :type rad: float

   :returns: The function `WrapTo2Pi(rad)` returns the angle `rad` wrapped to the range [0, 2pi].


.. py:function:: WrapToRegion(rad: float, range: list[float]) -> float

   Transform an angle to a defined range, with length of 2*pi.

   :param rad: Angle in radians.
   :type rad: float
   :param range: List defining the range [min, max].
   :type range: list

   :returns: Wrapped angle.
   :rtype: float


.. py:function:: convert_list_length(input_data: list[Any], number: int = 0) -> list[Any]

   Convert input to a list with a specific length.

   :param input_data: Data to convert.
   :param number: Desired length.
   :type number: int

   :returns: Converted list.
   :rtype: list


.. py:function:: convert_list_length_dict(input_data: list[Any], number: int = 0) -> list[Any]

   Convert input to a list with a specific length for dictionaries.

   :param input_data: Data to convert.
   :param number: Desired length.
   :type number: int

   :returns: Converted list.
   :rtype: list


.. py:function:: is_list_of_dicts(lst: Any) -> bool

   Check if a list contains only dictionaries.

   :param lst: List to check.
   :type lst: list

   :returns: True if all elements are dictionaries, False otherwise.
   :rtype: bool


.. py:function:: is_list_of_numbers(lst: Any) -> bool

   Check if a list contains only numbers.

   :param lst: List to check.
   :type lst: list

   :returns: True if all elements are numbers, False otherwise.
   :rtype: bool


.. py:function:: is_list_of_lists(lst: Any) -> bool

   Check if a list contains lists.

   :param lst: List to check.
   :type lst: list

   :returns: True if any element is a list, False otherwise.
   :rtype: bool


.. py:function:: is_list_not_list_of_lists(lst: Any) -> bool

   Check if a list does not contain lists.

   :param lst: List to check.
   :type lst: list

   :returns: True if no elements are lists, False otherwise.
   :rtype: bool


.. py:function:: relative_position(position1: numpy.ndarray, position2: numpy.ndarray, topi: bool = True) -> tuple[float, float]

   Calculate the relative position and angle between two points.

   :param position1: First position [x, y] (2x1).
   :type position1: np.array
   :param position2: Second position [x, y] (2x1).
   :type position2: np.array
   :param topi: Whether to wrap angle to [-pi, pi] (default True).
   :type topi: bool

   :returns: Distance and angle (radians).
   :rtype: tuple


.. py:function:: get_transform(state: numpy.ndarray) -> tuple[numpy.ndarray, numpy.ndarray]

   Get rotation and translation matrices from state.

   :param state: State [x, y, theta] (3x1) or [x, y] (2x1).
   :type state: np.array

   :returns: Translation vector and rotation matrix.
   :rtype: tuple


.. py:function:: transform_point_with_state(point: numpy.ndarray, state: numpy.ndarray) -> numpy.ndarray

   Transform a point using a state.

   :param point: Point [x, y, theta] (3x1).
   :type point: np.array
   :param state: State [x, y, theta] (3x1).
   :type state: np.array

   :returns: Transformed point (2x1).
   :rtype: np.array


.. py:function:: get_affine_transform(state: numpy.ndarray) -> list[float]

   Get affine transform parameters from state.

   :param state: State [x, y, theta] (3x1).
   :type state: np.array

   :returns: Affine transform parameters.
   :rtype: list


.. py:function:: geometry_transform(geometry: Any, state: numpy.ndarray) -> Any

   Transform geometry using a state.

   :param geometry: Shapely geometry to transform.
   :param state: [xoff, yoff, theta]
   :type state: np.array or sequence of 3 floats

   :returns: Transformed geometry.

   shapely expects [a, b, d, e, xoff, yoff] for:
   x' = a*x + b*y + xoff
   y' = d*x + e*y + yoff


.. py:function:: vertices_transform(vertices: numpy.ndarray, state: numpy.ndarray) -> Optional[numpy.ndarray]

   Transform vertices using a state.

   :param vertices: Vertices of the object. (2xN)
   :type vertices: np.array
   :param state: State [x, y, theta] (3x1).
   :type state: np.array

   :returns: Transformed vertices.
   :rtype: np.array


.. py:function:: omni_to_diff(state_ori: float, vel_omni: numpy.ndarray, w_max: float = 1.5, guarantee_time: float = 0.2, tolerance: float = 0.1, mini_speed: float = 0.02) -> numpy.ndarray

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


.. py:function:: diff_to_omni(state_ori: float, vel_diff: numpy.ndarray) -> numpy.ndarray

   Convert differential velocity to omnidirectional velocity.

   :param state_ori: Orientation angle.
   :type state_ori: float
   :param vel_diff: Differential velocity [linear, angular] (2x1).
   :type vel_diff: np.array

   :returns: Omnidirectional velocity [vx, vy] (2x1).
   :rtype: np.array


.. py:function:: time_it(name: str = 'Function') -> Any

   Decorator to measure function execution time.

   :param name: Function name for logging (default "Function").
   :type name: str
   :param print: Whether to print execution time (default True).
   :type print: bool

   :returns: Wrapped function with timing.
   :rtype: function


.. py:function:: time_it2(name: str = 'Function') -> Any

   Decorator to measure function execution time with instance attribute check.

   :param name: Function name for logging (default "Function").
   :type name: str

   :returns: Wrapped function with timing.
   :rtype: function


.. py:function:: cross_product(o: list[float], a: list[float], b: list[float]) -> float

   Compute the cross product of vectors OA and OB.

   :param o: Points representing vectors.
   :type o: array-like
   :param a: Points representing vectors.
   :type a: array-like
   :param b: Points representing vectors.
   :type b: array-like

   :returns: Cross product value.
   :rtype: float


.. py:function:: is_convex_and_ordered(points: numpy.ndarray) -> tuple[bool, Optional[str]]

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


.. py:function:: distance(point1: Union[list[float], numpy.ndarray], point2: Union[list[float], numpy.ndarray]) -> float

   Compute the distance between two points.

   :param point1: First point [x, y] (2x1).
   :type point1: np.array
   :param point2: Second point [x, y] (2x1).
   :type point2: np.array

   :returns: Distance between points.
   :rtype: float


.. py:function:: dist_hypot(x1: float, y1: float, x2: float, y2: float) -> float

.. py:function:: random_point_range(range_low: Optional[list[float]] = None, range_high: Optional[list[float]] = None) -> list[float]

   Generate a random point within a range.

   :param range_low: Lower bound of the range.
   :type range_low: list
   :param range_high: Upper bound of the range.
   :type range_high: list

   :returns: Random point within the range.
   :rtype: np.array


.. py:function:: is_2d_list(data: Union[list, collections.deque]) -> bool

   Returns True if 'data' is a non-empty list of lists (or tuples), indicating a 2D structure.
   Returns False if 'data' is a single list


