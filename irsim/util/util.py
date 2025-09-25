import math
import os
import sys
import time
from collections import deque
from math import atan2, cos, pi, sin
from typing import Any, Optional, Union

import numpy as np
from shapely.affinity import affine_transform

from irsim.config import env_param


def file_check(
    file_name: Optional[str], root_path: Optional[str] = None
) -> Optional[str]:
    """
    Check whether a file exists and return its absolute path.

    Args:
        file_name (str): Name of the file to check.
        root_path (str, optional): Root path to use if the file is not found.

    Returns:
        str: Absolute path of the file if found.

    Raises:
        FileNotFoundError: If the file is not found.
    """
    if file_name is None:
        return None

    if os.path.exists(file_name):
        abs_file_name = file_name
    elif os.path.exists(sys.path[0] + "/" + file_name):
        abs_file_name = sys.path[0] + "/" + file_name
    elif os.path.exists(os.getcwd() + "/" + file_name):
        abs_file_name = os.getcwd() + "/" + file_name
    else:
        if root_path is None:
            # raise FileNotFoundError("File not found: " + file_name)
            logger = getattr(env_param, "logger", None)
            if logger is not None:
                logger.warning(f"{file_name} not found")
            return None
        # root_file_name = root_path + "/" + file_name
        root_file_name = find_file(root_path, file_name)
        if os.path.exists(root_file_name):
            abs_file_name = root_file_name
        else:
            # raise FileNotFoundError("File not found: " + root_file_name)
            logger = getattr(env_param, "logger", None)
            if logger is not None:
                logger.warning(f"{root_file_name} not found")
            return None

    return abs_file_name


def find_file(root_path: str, target_filename: str) -> str:
    for dirpath, _dirnames, filenames in os.walk(root_path):
        if target_filename in filenames:
            return os.path.join(dirpath, target_filename)
    return target_filename


def WrapToPi(rad: float, positive: bool = False) -> float:
    """The function `WrapToPi` transforms an angle in radians to the range [-pi, pi].

    Args:

        rad (float): Angle in radians. The `rad` parameter in the `WrapToPi` function represents an angle in radians that you want to transform to the range `[-π, π]`. The function ensures that the angle is within this range by wrapping it around if it exceeds the bounds.

        positive (bool): Whether to return the positive value of the angle. Useful for angles difference.

    Returns:
        The function `WrapToPi(rad)` returns the angle `rad` wrapped to the range [-pi, pi].

    """
    while rad > pi:
        rad = rad - 2 * pi
    while rad < -pi:
        rad = rad + 2 * pi

    return rad if not positive else abs(rad)


def WrapTo2Pi(rad: float) -> float:
    """The function `WrapTo2Pi` transforms an angle in radians to the range [0, 2pi].

    Args:

        rad (float): Angle in radians.
            The `rad` parameter in the `WrapTo2Pi` function represents an angle in radians that you want to transform to the range `[0, 2pi]`. The function ensures that the angle is within this range by wrapping it around if it exceeds the bounds.

    Returns:
        The function `WrapTo2Pi(rad)` returns the angle `rad` wrapped to the range [0, 2pi].

    """
    while rad > 2 * pi:
        rad = rad - 2 * pi
    while rad < 0:
        rad = rad + 2 * pi

    return rad


def WrapToRegion(rad: float, range: list[float]) -> float:
    """
    Transform an angle to a defined range, with length of 2*pi.

    Args:
        rad (float): Angle in radians.
        range (list): List defining the range [min, max].

    Returns:
        float: Wrapped angle.
    """
    assert len(range) >= 2
    assert range[1] - range[0] == 2 * pi
    while rad > range[1]:
        rad = rad - 2 * pi
    while rad < range[0]:
        rad = rad + 2 * pi
    return rad


def convert_list_length(input_data: list[Any], number: int = 0) -> list[Any]:
    """
    Convert input to a list with a specific length.

    Args:
        input_data: Data to convert.
        number (int): Desired length.

    Returns:
        list: Converted list.
    """
    if number == 0:
        return []
    if not isinstance(input_data, list) or is_list_of_numbers(input_data):
        return [input_data] * number
    if len(input_data) <= number:
        input_data.extend([input_data[-1]] * (number - len(input_data)))
    if len(input_data) > number:
        input_data = input_data[:number]
    return input_data


def convert_list_length_dict(input_data: list[Any], number: int = 0) -> list[Any]:
    """
    Convert input to a list with a specific length for dictionaries.

    Args:
        input_data: Data to convert.
        number (int): Desired length.

    Returns:
        list: Converted list.
    """
    if number == 0:
        return []
    if not isinstance(input_data, list) or is_list_of_dicts(input_data):
        return [input_data] * number
    if len(input_data) <= number:
        input_data.extend([input_data[-1]] * (number - len(input_data)))
    if len(input_data) > number:
        input_data = input_data[:number]
    return input_data


def is_list_of_dicts(lst: Any) -> bool:
    """
    Check if a list contains only dictionaries.

    Args:
        lst (list): List to check.

    Returns:
        bool: True if all elements are dictionaries, False otherwise.
    """
    return isinstance(lst, list) and all(isinstance(sub, dict) for sub in lst)


def is_list_of_numbers(lst: Any) -> bool:
    """
    Check if a list contains only numbers.

    Args:
        lst (list): List to check.

    Returns:
        bool: True if all elements are numbers, False otherwise.
    """
    return isinstance(lst, list) and all(isinstance(sub, (int, float)) for sub in lst)


def is_list_of_lists(lst: Any) -> bool:
    """
    Check if a list contains lists.

    Args:
        lst (list): List to check.

    Returns:
        bool: True if any element is a list, False otherwise.
    """
    return isinstance(lst, list) and any(isinstance(sub, list) for sub in lst)


def is_list_not_list_of_lists(lst: Any) -> bool:
    """
    Check if a list does not contain lists.

    Args:
        lst (list): List to check.

    Returns:
        bool: True if no elements are lists, False otherwise.
    """
    return isinstance(lst, list) and all(not isinstance(sub, list) for sub in lst)


def relative_position(
    position1: np.ndarray, position2: np.ndarray, topi: bool = True
) -> tuple[float, float]:
    """
    Calculate the relative position and angle between two points.

    Args:
        position1 (np.array): First position [x, y] (2x1).
        position2 (np.array): Second position [x, y] (2x1).
        topi (bool): Whether to wrap angle to [-pi, pi] (default True).

    Returns:
        tuple: Distance and angle (radians).
    """
    diff = position2[0:2] - position1[0:2]
    distance = dist_hypot(
        position1[0, 0], position1[1, 0], position2[0, 0], position2[1, 0]
    )
    radian = atan2(diff[1, 0], diff[0, 0])
    if topi:
        radian = WrapToPi(radian)
    return distance, radian


def get_transform(state: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Get rotation and translation matrices from state.

    Args:
        state (np.array): State [x, y, theta] (3x1) or [x, y] (2x1).

    Returns:
        tuple: Translation vector and rotation matrix.
    """
    if state.shape == (2, 1):
        rot = np.array([[1, 0], [0, 1]])
        trans = state[0:2]
    else:
        rot = np.array(
            [
                [cos(state[2, 0]), -sin(state[2, 0])],
                [sin(state[2, 0]), cos(state[2, 0])],
            ]
        )
        trans = state[0:2]
    return trans, rot


def transform_point_with_state(point: np.ndarray, state: np.ndarray) -> np.ndarray:
    """
    Transform a point using a state.

    Args:
        point (np.array): Point [x, y, theta] (3x1).
        state (np.array): State [x, y, theta] (3x1).

    Returns:
        np.array: Transformed point (2x1).
    """
    trans, rot = get_transform(state)
    new_position = rot @ point[0:2] + trans
    new_theta = WrapToPi(point[2, 0] + state[2, 0])
    return np.array([new_position[0, 0], new_position[1, 0], new_theta]).reshape((3, 1))


def get_affine_transform(state: np.ndarray) -> list[float]:
    """
    Get affine transform parameters from state.

    Args:
        state (np.array): State [x, y, theta] (3x1).

    Returns:
        list: Affine transform parameters.
    """
    return [
        cos(state[2, 0]),
        -sin(state[2, 0]),
        sin(state[2, 0]),
        cos(state[2, 0]),
        state[0, 0],
        state[1, 0],
    ]


def geometry_transform(geometry: Any, state: np.ndarray) -> Any:
    """
    Transform geometry using a state.

    Args:
        geometry: Shapely geometry to transform.
        state (np.array or sequence of 3 floats): [xoff, yoff, theta]

    Returns:
        Transformed geometry.


    shapely expects [a, b, d, e, xoff, yoff] for:
    x' = a*x + b*y + xoff
    y' = d*x + e*y + yoff
    """

    xoff, yoff = state[:2]
    theta = state[2] if len(state) >= 3 else 0

    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    a = cos_t
    b = -sin_t
    d = sin_t
    e = cos_t

    return affine_transform(geometry, [a, b, d, e, xoff, yoff])


def vertices_transform(vertices: np.ndarray, state: np.ndarray) -> Optional[np.ndarray]:
    """
    Transform vertices using a state.

    Args:
        vertices (np.array): Vertices of the object. (2xN)
        state (np.array): State [x, y, theta] (3x1).

    Returns:
        np.array: Transformed vertices.
    """

    if vertices is None or state is None:
        return None

    trans, rot = get_transform(state)
    return rot @ vertices + trans


def omni_to_diff(
    state_ori: float,
    vel_omni: np.ndarray,
    w_max: float = 1.5,
    guarantee_time: float = 0.2,
    tolerance: float = 0.1,
    mini_speed: float = 0.02,
) -> np.ndarray:
    """
    Convert omnidirectional velocity to differential velocity.

    Args:
        state_ori (float): Orientation angle.
        vel_omni (np.array): Omnidirectional velocity [vx, vy] (2x1).
        w_max (float): Maximum angular velocity.
        guarantee_time (float): Time to guarantee velocity.
        tolerance (float): Angular tolerance.
        mini_speed (float): Minimum speed threshold.

    Returns:
        np.array: Differential velocity [linear, angular] (2x1).
    """
    if isinstance(vel_omni, list):
        vel_omni = np.array(vel_omni).reshape((2, 1))

    speed = np.sqrt(vel_omni[0, 0] ** 2 + vel_omni[1, 0] ** 2)

    if speed <= mini_speed:
        return np.zeros((2, 1))

    vel_radians = atan2(vel_omni[1, 0], vel_omni[0, 0])
    robot_radians = state_ori
    diff_radians = robot_radians - vel_radians

    diff_radians = WrapToPi(diff_radians)

    if abs(diff_radians) < tolerance:
        w = 0
    else:
        w = -diff_radians / guarantee_time
        if w > w_max:
            w = w_max
        if w < -w_max:
            w = -w_max

    v = speed * cos(diff_radians)
    if v < 0:
        v = 0

    return np.array([[v], [w]])


def diff_to_omni(state_ori: float, vel_diff: np.ndarray) -> np.ndarray:
    """
    Convert differential velocity to omnidirectional velocity.

    Args:
        state_ori (float): Orientation angle.
        vel_diff (np.array): Differential velocity [linear, angular] (2x1).

    Returns:
        np.array: Omnidirectional velocity [vx, vy] (2x1).
    """
    if len(vel_diff.shape) == 0:
        return np.zeros((2, 1))

    vel_linear = vel_diff[0, 0]
    theta = state_ori
    vx = vel_linear * cos(theta)
    vy = vel_linear * sin(theta)

    return np.array([[vx], [vy]])


def cross_product(o: list[float], a: list[float], b: list[float]) -> float:
    """
    Compute the cross product of vectors OA and OB.

    Args:
        o, a, b (array-like): Points representing vectors.

    Returns:
        float: Cross product value.
    """
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def is_convex_and_ordered(points: np.ndarray) -> tuple[bool, Optional[str]]:
    """
    Determine if the polygon is convex and return the order (CW or CCW).

    Args:
        points (np.ndarray): A 2xN NumPy array representing the vertices of the polygon.

    Returns:
        (bool, str): A tuple where the first element is True if the polygon is convex,
                      and the second element is 'CW' or 'CCW' based on the order.
                      If not convex, returns (False, None).
    """
    n = points.shape[1]  # Number of points
    if n < 3:
        return False, None  # A polygon must have at least 3 points

    # Initialize the direction for the first cross product
    direction = 0

    for i in range(n):
        o = points[:, i]
        a = points[:, (i + 1) % n]
        b = points[:, (i + 2) % n]

        cross = cross_product(o, a, b)

        if cross != 0:  # Only consider non-collinear points
            if direction == 0:
                direction = 1 if cross > 0 else -1
            elif (cross > 0 and direction < 0) or (cross < 0 and direction > 0):
                return False, None  # Not convex

    return True, "CCW" if direction > 0 else "CW"


def gen_inequal_from_vertex(vertex: np.ndarray):
    """
    Generate inequality constraints for a convex polygon.

    Args:
        vertex (np.array): Vertices of the polygon (2xN).

    Returns:
        tuple: G matrix and h vector for the inequality Gx <= h.
    """
    convex_flag, order = is_convex_and_ordered(vertex)

    if not convex_flag:
        print("The polygon constructed by vertex is not convex.")
        return None, None

    if order == "CW":
        vertex = vertex[:, ::-1]

    num = vertex.shape[1]

    G = np.zeros((num, 2))
    h = np.zeros((num, 1))

    for i in range(num):
        if i + 1 < num:
            pre_point = vertex[:, i]
            next_point = vertex[:, i + 1]
        else:
            pre_point = vertex[:, i]
            next_point = vertex[:, 0]

        diff = next_point - pre_point

        a = diff[1]
        b = -diff[0]
        c = a * pre_point[0] + b * pre_point[1]

        G[i, 0] = a
        G[i, 1] = b
        h[i, 0] = c

    return G, h


def distance(
    point1: Union[list[float], np.ndarray], point2: Union[list[float], np.ndarray]
) -> float:
    """
    Compute the distance between two points.

    Args:
        point1 (np.array): First point [x, y] (2x1).
        point2 (np.array): Second point [x, y] (2x1).

    Returns:
        float: Distance between points.
    """
    return dist_hypot(point1[0, 0], point1[1, 0], point2[0, 0], point2[1, 0])


def dist_hypot(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def random_point_range(
    range_low: Optional[list[float]] = None, range_high: Optional[list[float]] = None
) -> list[float]:
    """
    Generate a random point within a range.

    Args:
        range_low (list): Lower bound of the range.
        range_high (list): Upper bound of the range.

    Returns:
        np.array: Random point within the range.
    """
    if range_low is None:
        range_low = [0, 0, -pi]
    if range_high is None:
        range_high = [10, 10, pi]

    if isinstance(range_low, list):
        range_low = np.c_[range_low]

    if isinstance(range_high, list):
        range_high = np.c_[range_high]

    return np.random.uniform(range_low, range_high)


def is_2d_list(data: Union[list, deque]) -> bool:
    """
    Returns True if 'data' is a non-empty list of lists (or tuples), indicating a 2D structure.
    Returns False if 'data' is a single list
    """

    if isinstance(data, np.ndarray):
        return False

    # assert isinstance(data, list)
    # Check if data is a list and is not empty.
    if data:
        first_element = data[0]
        if isinstance(first_element, (list, tuple)):
            return True

    return False


# decorator


def time_it(name: str = "Function") -> Any:
    """
    Decorator to measure function execution time.

    Args:
        name (str): Function name for logging (default "Function").
        print (bool): Whether to print execution time (default True).

    Returns:
        function: Wrapped function with timing.
    """

    def decorator(func):
        def wrapper(*args, **kwargs):
            wrapper.count += 1
            start = time.time()
            result = func(*args, **kwargs)
            end = time.time()
            wrapper.func_count += 1
            print(f"{name} execute time {(end - start):.6f} seconds")
            return result

        wrapper.count = 0
        wrapper.func_count = 0
        return wrapper

    return decorator


def normalize_actions(func):
    """
    Decorator to normalize (action, action_id) into an aligned actions list.

    The wrapped method must belong to a class that has a ``objects`` attribute.
    It will receive an extra keyword argument ``aligned_actions`` (length == len(self.objects)).
    """

    def wrapper(self, action=None, action_id=0, *args, **kwargs):
        num_objects = len(getattr(self, "objects", []))
        actions = [None] * num_objects

        if action is not None:
            if isinstance(action, list):
                if isinstance(action_id, list):
                    for a, ai in zip(action, action_id):
                        actions[int(ai)] = a
                else:
                    start = int(action_id)
                    actions[start : start + len(action)] = action[:]
            elif isinstance(action, np.ndarray):
                if isinstance(action_id, list):
                    for ai in action_id:
                        actions[int(ai)] = action
                else:
                    actions[int(action_id)] = action

        # Call the original function with normalized actions and a neutral action_id
        return func(self, actions, 0, *args, **kwargs)

    return wrapper


def time_it2(name: str = "Function") -> Any:
    """
    Decorator to measure function execution time with instance attribute check.

    Args:
        name (str): Function name for logging (default "Function").

    Returns:
        function: Wrapped function with timing.
    """

    def decorator(func):
        def wrapper(self, *args, **kwargs):
            wrapper.count += 1
            start = time.time()
            result = func(self, *args, **kwargs)
            end = time.time()
            wrapper.func_count += 1
            if self.time_print:
                print(f"{name} execute time {(end - start):.6f} seconds")
            return result

        wrapper.count = 0
        wrapper.func_count = 0
        return wrapper

    return decorator


def to_numpy(
    data: Any,
    default: Optional[np.ndarray] = None,
    expected_shape: Optional[tuple[int, ...]] = None,
) -> Optional[np.ndarray]:
    """
    Convert input to numpy array and optionally reshape.

    - If data is None: return default (reshaped if expected_shape provided).
    - If data is a list: convert to ndarray. 1D lists become column vectors.
    - If data is a 1D ndarray: convert to column vector.
    - If expected_shape is provided: reshape the resulting array to it.
    """

    if data is None:
        if default is None:
            return None
        return default if expected_shape is None else default.reshape(expected_shape)

    arr = np.array(data) if isinstance(data, list) else data

    if isinstance(arr, np.ndarray) and arr.ndim == 1:
        arr = arr[:, np.newaxis]

    if expected_shape is not None:
        arr = np.asarray(arr).reshape(expected_shape)

    return arr


def traj_to_xy_list(
    traj: Any, three_d: bool = False
) -> tuple[list[float], list[float]]:
    """
    Convert trajectory to a list of [x, y].

    Args:
        traj (list or np.ndarray): list of points or array of points [x, y, theta].

    Returns:
        tuple: A tuple of lists containing x and y coordinates of the trajectory
            x_list (list): List of x coordinates.
            y_list (list): List of y coordinates.
    """
    if isinstance(traj, list):
        x_list = [p[0, 0] for p in traj]
        y_list = [p[1, 0] for p in traj]
        if three_d:
            z_list = [p[2, 0] for p in traj]
    elif isinstance(traj, np.ndarray):
        if traj.shape[1] > 1:
            x_list = [p[0] for p in traj.T]
            y_list = [p[1] for p in traj.T]
            if three_d:
                z_list = [p[2] for p in traj.T]
        else:
            x_list = [traj[0]]
            y_list = [traj[1]]
            if three_d:
                z_list = [traj[2]]
    else:
        raise ValueError(f"Invalid trajectory type: {type(traj)}")

    if three_d:
        return x_list, y_list, z_list
    return x_list, y_list


def points_to_xy_list(
    points: Any, three_d: bool = False
) -> tuple[list[float], list[float]]:
    """
    Convert points to a list of [x, y].

    Args:
        points (list or np.ndarray): list of points or array of points [x, y].
        three_d (bool): Whether the points are 3D.
    Returns:
        tuple: A tuple of lists containing x and y coordinates of the points
            x_list (list): List of x coordinates.
            y_list (list): List of y coordinates.
    """

    if points is None:
        return [], []

    if isinstance(points, list):
        x_list = [point[0] for point in points]
        y_list = [point[1] for point in points]
        if three_d:
            z_list = [point[2] for point in points]
    elif isinstance(points, np.ndarray):
        if points.shape[1] > 1:
            x_list = [point[0] for point in points.T]
            y_list = [point[1] for point in points.T]
            if three_d:
                z_list = [point[2] for point in points.T]
        else:
            x_list = [points[0]]
            y_list = [points[1]]
            if three_d:
                z_list = [points[2]]
    else:
        raise ValueError(f"Invalid points type: {type(points)}")

    if three_d:
        return x_list, y_list, z_list
    return x_list, y_list
