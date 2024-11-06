import os
import sys
from math import pi, atan2, sin, cos, sqrt
import numpy as np
from shapely import ops
import time


def file_check(file_name, root_path=None):
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
            raise FileNotFoundError("File not found: " + file_name)
        else:
            root_file_name = root_path + "/" + file_name
            if os.path.exists(root_file_name):
                abs_file_name = root_file_name
            else:
                raise FileNotFoundError("File not found: " + root_file_name)

    return abs_file_name


def repeat_mk_dirs(path, max_num=100):
    """
    Create a directory, appending numbers if it already exists.

    Args:
        path (str): Path of the directory to create.
        max_num (int): Maximum number of attempts.

    Returns:
        str: Path of the created directory.
    """
    if not os.path.exists(path):
        os.makedirs(path)
        return path
    else:
        if len(os.listdir(path)) == 0:  # empty dir
            return path
        else:
            i = 1
            while i < max_num:
                new_path = path + "_" + str(i)
                i = i + 1
                if not os.path.exists(new_path):
                    break
            os.makedirs(new_path)
            return new_path


def WrapToPi(rad):
    """
    Transform an angle to the range [-pi, pi].

    Args:
        rad (float): Angle in radians.

    Returns:
        float: Wrapped angle.
    """
    while rad > pi:
        rad = rad - 2 * pi
    while rad < -pi:
        rad = rad + 2 * pi
    return rad


def WrapToRegion(rad, range):
    """
    Transform an angle to a defined range, with length of 2*pi.

    Args:
        rad (float): Angle in radians.
        range (list): List defining the range [min, max].

    Returns:
        float: Wrapped angle.
    """
    assert len(range) >= 2 and range[1] - range[0] == 2 * pi
    while rad > range[1]:
        rad = rad - 2 * pi
    while rad < range[0]:
        rad = rad + 2 * pi
    return rad


def extend_list(input_list, number):
    """
    Extend a list to a specific length.

    Args:
        input_list (list): List to extend.
        number (int): Desired length.

    Returns:
        list: Extended list.
    """
    if not isinstance(input_list, list):
        return [input_list] * number
    if number == 0:
        return []
    if len(input_list) == 0:
        return None
    if len(input_list) <= number:
        input_list.extend([input_list[-1]] * (number - len(input_list)))
    if len(input_list) > number:
        input_list = input_list[:number]
    return input_list


def convert_list_length(input_data, number=0):
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


def convert_list_length_dict(input_data, number=0):
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


def is_list_of_dicts(lst):
    """
    Check if a list contains only dictionaries.

    Args:
        lst (list): List to check.

    Returns:
        bool: True if all elements are dictionaries, False otherwise.
    """
    return isinstance(lst, list) and all(isinstance(sub, dict) for sub in lst)


def is_list_of_numbers(lst):
    """
    Check if a list contains only numbers.

    Args:
        lst (list): List to check.

    Returns:
        bool: True if all elements are numbers, False otherwise.
    """
    return isinstance(lst, list) and all(isinstance(sub, (int, float)) for sub in lst)


def is_list_of_lists(lst):
    """
    Check if a list contains lists.

    Args:
        lst (list): List to check.

    Returns:
        bool: True if any element is a list, False otherwise.
    """
    return isinstance(lst, list) and any(isinstance(sub, list) for sub in lst)


def is_list_not_list_of_lists(lst):
    """
    Check if a list does not contain lists.

    Args:
        lst (list): List to check.

    Returns:
        bool: True if no elements are lists, False otherwise.
    """
    return isinstance(lst, list) and all(not isinstance(sub, list) for sub in lst)


def relative_position(position1, position2, topi=True):
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
    distance = np.linalg.norm(diff)
    radian = atan2(diff[1, 0], diff[0, 0])
    if topi:
        radian = WrapToPi(radian)
    return distance, radian


def get_transform(state):
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


def transform_point_with_state(point, state):
    """
    Transform a point using a state.

    Args:
        point (np.array): Point [x, y] (2x1).
        state (np.array): State [x, y, theta] (3x1).

    Returns:
        np.array: Transformed point (2x1).
    """
    trans, rot = get_transform(state)
    new_point = rot @ point[0:2] + trans
    return new_point


def get_affine_transform(state):
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


def geometry_transform(geometry, state):
    """
    Transform geometry using a state.

    Args:
        geometry: Geometry to transform.
        state (np.array): State [x, y, theta] (3x1).

    Returns:
        Transformed geometry.
    """

    def transfor_with_state(x, y):
        trans, rot = get_transform(state)
        points = np.array([x, y])
        new_points = rot @ points + trans
        return (new_points[0, :], new_points[1, :])

    new_geometry = ops.transform(transfor_with_state, geometry)
    return new_geometry


def omni_to_diff(
    state_ori, vel_omni, w_max=1.5, guarantee_time=0.2, tolerance=0.1, mini_speed=0.02
):
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


def diff_to_omni(state_ori, vel_diff):
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


def time_it(name="Function", print=True):
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


def time_it2(name="Function"):
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


def cal_init_vertex(length, width, wheelbase):
    """
    Calculate initial vertices of a rectangle representing a robot.

    Args:
        length (float): Length of the rectangle.
        width (float): Width of the rectangle.
        wheelbase (float): Wheelbase of the robot.

    Returns:
        np.array: Vertices of the rectangle (2x4).
    """
    start_x = -(length - wheelbase) / 2
    start_y = -width / 2

    point0 = np.array([[start_x], [start_y]])  # left bottom point
    point1 = np.array([[start_x + length], [start_y]])
    point2 = np.array([[start_x + length], [start_y + width]])
    point3 = np.array([[start_x], [start_y + width]])

    return np.hstack((point0, point1, point2, point3))


def cal_init_vertex_diff(length, width):
    """
    Calculate initial vertices of a rectangle for differential drive robot.

    Args:
        length (float): Length of the rectangle.
        width (float): Width of the rectangle.

    Returns:
        np.array: Vertices of the rectangle (2x4).
    """
    start_x = -length / 2
    start_y = -width / 2

    point0 = np.array([[start_x], [start_y]])  # left bottom point
    point1 = np.array([[start_x + length], [start_y]])
    point2 = np.array([[start_x + length], [start_y + width]])
    point3 = np.array([[start_x], [start_y + width]])

    return np.hstack((point0, point1, point2, point3))


def cross_product(o, a, b):
    """
    Compute the cross product of vectors OA and OB.

    Args:
        o, a, b (array-like): Points representing vectors.

    Returns:
        float: Cross product value.
    """
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def is_convex_and_ordered(points):
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


def gen_inequal_from_vertex(vertex):
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


def distance(point1, point2):
    """
    Compute the distance between two points.

    Args:
        point1 (np.array): First point [x, y] (2x1).
        point2 (np.array): Second point [x, y] (2x1).

    Returns:
        float: Distance between points.
    """
    return sqrt((point1[0, 0] - point2[0, 0]) ** 2 + (point1[1, 0] - point2[1, 0]) ** 2)



