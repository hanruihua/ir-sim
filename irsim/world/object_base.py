import shapely
import logging
import itertools
import math
import numpy as np
import matplotlib as mpl
import matplotlib.transforms as mtransforms
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Wedge
from matplotlib import image
from mpl_toolkits.mplot3d import Axes3D
from dataclasses import dataclass
from math import inf, pi, atan2, cos, sin
from typing import Optional, Union
from collections import deque
from irsim.lib import Behavior, KinematicsFactory, GeometryFactory
from irsim.global_param import world_param, env_param
from irsim.world import SensorFactory
from irsim.env.env_plot import linewidth_from_data_units
from irsim.global_param.path_param import path_manager

from irsim.util.util import (
    WrapToRegion,
    relative_position,
    diff_to_omni,
    random_point_range,
    WrapToPi,
    is_2d_list,
)


@dataclass
class ObjectInfo:
    id: int
    shape: str
    kinematics: str
    role: str
    color: str
    static: bool
    goal: np.ndarray
    vel_min: np.ndarray
    vel_max: np.ndarray
    acce: np.ndarray
    angle_range: np.ndarray
    goal_threshold: float
    wheelbase: float
    G: np.ndarray
    h: np.ndarray
    cone_type: str
    convex_flag: bool

    def add_property(self, key, value):
        setattr(self, key, value)


@dataclass
class ObstacleInfo:
    center: np.ndarray
    vertex: np.ndarray
    velocity: np.ndarray
    radius: float
    G: np.ndarray
    h: np.ndarray
    cone_type: str
    convex_flag: bool

    def add_property(self, key, value):
        setattr(self, key, value)


class ObjectBase:
    """
    Base class representing a generic object in the robot simulator.

    This class encapsulates common attributes and behaviors for all objects,
    including robots and obstacles, managing their state, velocity, goals,
    and kinematics.

    Args:
        shape (dict): Parameters defining the shape of the object for geometry creation.
            The dictionary should contain keys and values required by the GeometryFactory to create
            the object's geometry, such as 'type' (e.g., 'circle', 'rectangle') and associated parameters.
            Defaults to None.
        kinematics (dict): Parameters defining the kinematics of the object.
            Includes kinematic model and any necessary parameters. If None, no kinematics model is applied.
            Defaults to None.
        state (list of float): Initial state vector [x, y, theta, ...].
            The state can have more dimensions depending on `state_dim`. Excess dimensions are truncated,
            and missing dimensions are filled with zeros. Defaults to [0, 0, 0].
        velocity (list of float): Initial velocity vector [vx, vy] or according to the kinematics model.
            Defaults to [0, 0].
        goal (list of float or list of list of float): Goal state vector [x, y, theta, ...] or [[x, y, theta], [x, y, theta], ...] for multiple goals
            Used by behaviors to determine the desired movement. Defaults to [10, 10, 0].
        role (str): Role of the object in the simulation, e.g., "robot" or "obstacle".
            Defaults to "obstacle".
        color (str): Color of the object when plotted.
            Defaults to "k" (black).
        static (bool): Indicates if the object is static (does not move).
            Defaults to False.
        vel_min (list of float): Minimum velocity limits for each control dimension.
            Used to constrain the object's velocity. Defaults to [-1, -1].
        vel_max (list of float): Maximum velocity limits for each control dimension.
            Used to constrain the object's velocity. Defaults to [1, 1].
        acce (list of float): Acceleration limits, specifying the maximum change in velocity per time step.
            Defaults to [inf, inf].
        angle_range (list of float): Allowed range of orientation angles [min, max] in radians.
            The object's orientation will be wrapped within this range. Defaults to [-pi, pi].
        behavior (dict or str): Behavioral mode or configuration of the object.
            Can be a behavior name (str) or a dictionary with behavior parameters. If None, default behavior is applied.
            Defaults to {'name': 'dash'}, moving to the goal directly.
        goal_threshold (float): Threshold distance to determine if the object has reached its goal.
            When the object is within this distance to the goal, it's considered to have arrived. Defaults to 0.1.
        sensors (list of dict): List of sensor configurations attached to the object.
            Each sensor configuration is a dictionary specifying sensor type and parameters. Defaults to None.
        arrive_mode (str): Mode for arrival detection, either "position" or "state".
            Determines how arrival at the goal is evaluated. Defaults to "position".
        description (str): Description or label for the object.
            Can be used for identification or attaching images in plotting. Defaults to None.
        group (int): Group identifier for organizational purposes, allowing objects to be grouped.
            Defaults to 0.
        state_dim (int): Dimension of the state vector.
            If None, it is inferred from the class attribute `state_shape`. Defaults to None.
        vel_dim (int): Dimension of the velocity vector.
            If None, it is inferred from the class attribute `vel_shape`. Defaults to None.
        unobstructed (bool): Indicates if the object should be considered to have an unobstructed path,
            ignoring obstacles in certain scenarios. Defaults to False.
        fov (float): Field of view angles in radians for the object's sensors. Defaults to None. If set lidar, the default value is angle range of lidar.
        fov_radius (float): Field of view radius for the object's sensors. Defaults to None. If set lidar, the default value is range_max of lidar.
        **kwargs: Additional keyword arguments for extended functionality.
            plot (dict): Plotting options for the object.
                May include 'show_goal', 'show_text', 'show_arrow', 'show_uncertainty', 'show_trajectory',
                'trail_freq', etc.

    Raises:
        ValueError: If dimension parameters do not match the provided shapes or if input parameters are invalid.

    Attributes:
        state_dim (int): Dimension of the state vector.
        state_shape (tuple): Shape of the state array.
        vel_dim (int): Dimension of the velocity vector.
        vel_shape (tuple): Shape of the velocity array.
        state (np.ndarray): Current state of the object.
        _init_state (np.ndarray): Initial state of the object.
        _velocity (np.ndarray): Current velocity of the object.
        _init_velocity (np.ndarray): Initial velocity of the object.
        _goal (np.ndarray): Goal state of the object.
        _init_goal (np.ndarray): Initial goal state of the object.
        _geometry (any): Geometry representation of the object.
        group (int): Group identifier for the object.
        stop_flag (bool): Flag indicating if the object should stop.
        arrive_flag (bool): Flag indicating if the object has arrived at the goal.
        collision_flag (bool): Flag indicating a collision has occurred.
        unobstructed (bool): Indicates if the object has an unobstructed path.
        static (bool): Indicates if the object is static.
        vel_min (np.ndarray): Minimum velocity limits.
        vel_max (np.ndarray): Maximum velocity limits.
        color (str): Color of the object.
        role (str): Role of the object (e.g., "robot", "obstacle").
        info (ObjectInfo): Information container for the object.
        wheelbase (float): Distance between the front and rear wheels. Specified for ackermann robots.
        fov (float): Field of view angles in radians.
        fov_radius (float): Field of view radius.

    """

    id_iter = itertools.count()
    vel_shape = (2, 1)
    state_shape = (3, 1)

    def __init__(
        self,
        shape: Optional[dict] = None,
        kinematics: Optional[dict] = None,
        state: list = [0, 0, 0],
        velocity: list = [0, 0],
        goal: list = [10, 10, 0],
        role: str = "obstacle",
        color: str = "k",
        static: bool = False,
        vel_min: list = [-1, -1],
        vel_max: list = [1, 1],
        acce: list = [inf, inf],
        angle_range: list = [-pi, pi],
        behavior: Optional[dict] = None,
        goal_threshold: float = 0.1,
        sensors: Optional[dict] = None,
        arrive_mode: str = "position",
        description: Optional[str] = None,
        group: int = 0,
        state_dim: Optional[int] = None,
        vel_dim: Optional[int] = None,
        unobstructed: bool = False,
        fov: Optional[float] = None,
        fov_radius: Optional[float] = None,
        **kwargs,
    ) -> None:
        """
        Initialize an ObjectBase instance.

        This method sets up a new ObjectBase object with the specified parameters, initializing its
        geometry, kinematics, behaviors, sensors, and other properties relevant to simulation.
        """

        self._id = next(ObjectBase.id_iter)

        # handlers
        self.gf = (
            GeometryFactory.create_geometry(**shape) if shape is not None else None
        )
        self.kf = (
            KinematicsFactory.create_kinematics(
                wheelbase=self.wheelbase, role=role, **kinematics
            )
            if kinematics is not None
            else None
        )

        if self.gf is not None:
            self.G, self.h, self.cone_type, self.convex_flag = self.gf.get_init_Gh()
        else:
            self.G, self.h, self.cone_type, self.convex_flag = None, None, None, None

        self.state_dim = state_dim if state_dim is not None else self.state_shape[0]
        self.state_shape = (
            (self.state_dim, 1) if state_dim is not None else self.state_shape
        )
        self.vel_dim = vel_dim if vel_dim is not None else self.vel_shape[0]
        self.vel_shape = (self.vel_dim, 1) if vel_dim is not None else self.vel_shape

        state = self.input_state_check(state, self.state_dim)
        self._state = np.c_[state]
        self._init_state = np.c_[state]

        self._velocity = np.c_[velocity]
        self._init_velocity = np.c_[velocity]

        self._goal = deque(goal) if is_2d_list(goal) else deque([goal])
        self._init_goal = self._goal.copy()

        self._geometry = self.gf.step(self.state)
        self.group = group

        # flag
        self.stop_flag = False
        self.arrive_flag = False
        self.collision_flag = False
        self.unobstructed = unobstructed

        # information
        self.static = static if self.kf is not None else True
        self.vel_min = np.c_[vel_min]
        self.vel_max = np.c_[vel_max]
        self.color = color
        self.role = role

        self.info = ObjectInfo(
            self._id,
            self.shape,
            self.kinematics,
            role,
            color,
            static,
            np.c_[goal],
            np.c_[vel_min],
            np.c_[vel_max],
            np.c_[acce],
            np.c_[angle_range],
            goal_threshold,
            self.wheelbase,
            self.G,
            self.h,
            self.cone_type,
            self.convex_flag,
        )

        self.obstacle_info = None

        self.trajectory = []

        self.description = description

        # arrive judgement
        self.goal_threshold = goal_threshold
        self.arrive_mode = arrive_mode

        # sensor
        sf = SensorFactory()
        self.lidar = None
        if sensors is not None:
            self.sensors = [
                sf.create_sensor(self._state[0:3], self._id, **sensor_kwargs)
                for sensor_kwargs in sensors
            ]

            self.lidar = [
                sensor for sensor in self.sensors if sensor.sensor_type == "lidar2d"
            ][0]
        else:
            self.sensors = []

        if fov is None:
            self.fov = self.lidar.angle_range if self.lidar is not None else None
            self.fov_radius = self.lidar.range_max if self.lidar is not None else None
        else:
            self.fov = fov
            self.fov_radius = fov_radius

        # behavior
        self.obj_behavior = Behavior(self.info, behavior)
        self.rl = self.beh_config.get("range_low", [0, 0, -pi])
        self.rh = self.beh_config.get("range_high", [10, 10, pi])
        self.wander = self.beh_config.get("wander", False)

        if self.wander:
            self._goal = deque([random_point_range(self.rl, self.rh).flatten().tolist()])

        # plot
        self.plot_kwargs = kwargs.get("plot", dict())
        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []
        self.collision_obj = []

    def __eq__(self, o: "ObjectBase") -> bool:

        if isinstance(o, ObjectBase):
            return self._id == o._id
        else:
            return False

    def __hash__(self) -> int:
        return self._id

    def __str__(self) -> str:
        return f"ObjectBase: {self._id}"

    @classmethod
    def reset_id_iter(cls, start: int = 0, step: int = 1):
        """reset the id iterator"""
        cls.id_iter = itertools.count(start, step)

    def step(self, velocity: Optional[np.ndarray] = None, **kwargs: any):
        """
        Perform a simulation step, updating the object's state.

        Args:
            velocity (np.ndarray, optional): Desired velocity for the step.
            **kwargs: Additional parameters.

        Returns:
            np.ndarray: The new state of the object.
        """

        if self.static or self.stop_flag:
            self._velocity = np.zeros(self.vel_shape)
            return self.state
        else:
            self.pre_process()
            behavior_vel = self.gen_behavior_vel(velocity)
            new_state = self.kf.step(self.state, behavior_vel, world_param.step_time)
            next_state = self.mid_process(new_state)

            self._state = next_state
            self._velocity = behavior_vel
            self._geometry = self.gf.step(self.state)
            self.sensor_step()
            self.post_process()
            self.check_status()
            self.trajectory.append(self.state.copy())
            return next_state

    def sensor_step(self):
        """
        Update all sensors for the current state.
        """
        [sensor.step(self.state[0:3]) for sensor in self.sensors]

    def check_status(self):
        """
        Check the current status, including arrival and collision.
        """
        self.check_arrive_status()
        self.check_collision_status()

        if world_param.collision_mode == "stop":
            self.stop_flag = any([not obj.unobstructed for obj in self.collision_obj])

        elif world_param.collision_mode == "reactive":
            pass

        elif world_param.collision_mode == "unobstructed":
            pass

        elif world_param.collision_mode == "unobstructed_obstacles":
            if self.role == "robot":
                self.stop_flag = any(
                    [not obj.unobstructed for obj in self.collision_obj]
                )
            elif self.role == "obstacle":
                self.stop_flag = False
        else:
            if world_param.count % 50 == 0 and self.role == "robot":
                env_param.logger.warning(f"collision mode {world_param.collision_mode} is not defined within [stop, reactive, unobstructed, unobstructed_obstacles], the unobstructed mode is used")
        

    def check_arrive_status(self):
        """
        Check if the object has arrived at the goal.
        """
        if self.arrive_mode == "state":
            diff = np.linalg.norm(self.state[:3] - self.goal[:3])
        elif self.arrive_mode == "position":
            diff = np.linalg.norm(self.state[:2] - self.goal[:2])

        if diff < self.goal_threshold:
            if len(self._goal) == 1:
                self.arrive_flag = True
            else:
                self._goal.popleft()
                self.arrive_flag = False    
        else:
            self.arrive_flag = False

    def check_collision_status(self):
        """
        Check if the object is in collision with others.
        """
        collision_flags = []

        self.collision_obj = []

        for obj in self.external_objects:
            if not obj.unobstructed:
                if self.check_collision(obj):
                    collision_flags.append(True)
                    self.collision_obj.append(obj)
                    
                    if self.role == "robot":
                        if not self.collision_flag:
                            env_param.logger.warning(
                                f"{self.name} collided with {obj.name} at state {np.round(self.state[:3, 0], 2).tolist()}"
                            )
                else:
                    collision_flags.append(False)

        self.collision_flag = any(collision_flags)

    def check_collision(self, obj):
        """
        Check collision with another object.

        Args:
            obj (ObjectBase): Another object.

        Returns:
            bool: True if collision occurs, False otherwise.
        """
        return shapely.intersects(self.geometry, obj._geometry)

    def gen_behavior_vel(self, velocity: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Generate behavior-influenced velocity for the object.

        This method adjusts the desired velocity based on the object's behavior configurations.
        If no desired velocity is provided (`velocity` is None), the method may generate a default
        velocity or issue warnings based on the object's role and behavior settings.

        Args:
            velocity (Optional[np.ndarray]): Desired velocity vector. If None, the method determines
                the velocity based on behavior configurations. Defaults to None.

        Returns:
            np.ndarray: Velocity vector adjusted based on behavior configurations and constraints.

        Raises:
            Warning: If `velocity` is None and no behavior configuration is set for a robot.
        """
        min_vel, max_vel = self.get_vel_range()

        if velocity is None:
            if self.beh_config is None:
                if self.role == "robot":
                    env_param.logger.warning(
                        "behavior and input velocity is not defined, robot will stay static"
                    )

                return np.zeros_like(self._velocity)

            else:
                if self.wander and self.arrive_flag:
                    self._goal = deque([random_point_range(self.rl, self.rh).flatten().tolist()])
                    self.arrive_flag = False

                behavior_vel = self.obj_behavior.gen_vel(
                    self.ego_object, self.external_objects
                )

        else:
            if isinstance(velocity, list):
                velocity = np.c_[velocity]
            if velocity.ndim == 1:
                velocity = velocity[:, np.newaxis]

            assert velocity.shape == self.vel_shape

            behavior_vel = velocity

        # clip the behavior_vel by maximum and minimum limits
        if (behavior_vel < (min_vel - 0.01)).any():
            logging.warning(
                "input velocity {} is smaller than min_vel {}, velocity is clipped".format(
                    behavior_vel.flatten(), min_vel.flatten()
                )
            )
        elif (behavior_vel > (max_vel + 0.01)).any():
            logging.warning(
                "input velocity {} is larger than max_vel {}, velocity is clipped".format(
                    behavior_vel.flatten(), max_vel.flatten()
                )
            )

        behavior_vel_clip = np.clip(behavior_vel, min_vel, max_vel)

        return behavior_vel_clip

    def pre_process(self):
        pass

    def post_process(self):
        pass

    def mid_process(self, state: np.ndarray):
        """
        Process state in the middle of a step. Make sure the state is within the desired dimension.

        Args:
            state (np.ndarray): State vector.

        Returns:
            np.ndarray: Processed state.
        """
        if state.shape[0] > 2:
            state[2, 0] = WrapToRegion(state[2, 0], self.info.angle_range)

        if state.shape[0] < self.state_dim:
            pad_value = (
                self.state[self.state_dim - 1, 0]
                if self.state.shape[0] >= self.state_dim
                else 0
            )

            pad_rows = self.state_dim - state.shape[0]

            padding = pad_value * np.ones((pad_rows, state.shape[1]))
            state = np.concatenate((state, padding), axis=0)

        elif state.shape[0] > self.state_dim:
            state = state[: self.state_dim]

        return state

    def get_lidar_scan(self):
        return self.lidar.get_scan()

    def get_lidar_points(self):
        return self.lidar.get_points()

    def get_lidar_offset(self):
        return self.lidar.get_offset()

    def get_inequality_Gh(self):
        return self.gf.G, self.gf.h

    def get_fov_detected_objects(self):
        """
        Detect the env objects that in the field of view.


        Returns:
            list: The objects that in the field of view of the object.
        """

        return [obj for obj in self.external_objects if self.fov_detect_object(obj)]

    def fov_detect_object(self, detected_object: "ObjectBase"):
        """
        Detect whether the input object is in the field of view.

        Args:
            object: The object that to be detected.

        Returns:
            bool: Whether the object is in the field of view.
        """

        rx, ry = self.state[:2, 0]
        px, py = detected_object.state[:2, 0]
        radius_do = detected_object.radius

        dx = px - rx
        dy = py - ry

        rad_to_do = WrapToPi(math.atan2(dy, dx))
        object_orientation = self.state[2, 0]

        rad_diff = WrapToPi(rad_to_do - object_orientation, True)
        distance_do = math.hypot(dx, dy)

        if distance_do == 0:
            rad_offset = pi / 2
        else:
            rad_offset = WrapToPi(math.asin(min(radius_do / distance_do, 1)))

        fov_diff = abs(rad_diff - rad_offset)

        if fov_diff <= (self.fov / 2) and (distance_do - radius_do) <= self.fov_radius:
            return True
        else:
            return False

    def set_state(self, state: Union[list, np.ndarray]= [0, 0, 0], init: bool = False):
        """
        Set the state of the object.

        Args:
            state: The state of the object [x, y, theta].
            init (bool): Whether to set the initial state (default False).
        """
        if isinstance(state, list):
            if len(state) > self.state_dim:
                temp_state = np.c_[state[: self.state_dim]]
            elif len(state) < self.state_dim:
                temp_state = np.c_[state + [0] * (self.state_dim - len(state))]
            else:
                temp_state = np.c_[state]

        elif isinstance(state, np.ndarray):
            if state.shape[0] > self.state_dim:
                temp_state = state[: self.state_dim]
            elif state.shape[0] < self.state_dim:
                temp_state = np.r_[
                    state, np.zeros((self.state_dim - state.shape[0], state.shape[1]))
                ]
            else:
                temp_state = state

        assert self.state.shape == temp_state.shape

        if init:
            self._init_state = temp_state.copy()

        self._state = temp_state.copy()
        self._geometry = self.gf.step(self.state)

    def set_velocity(self, velocity: Union[list, np.ndarray] = [0, 0], init: bool = False):
        """
        Set the velocity of the object.

        Args:
            velocity: The velocity of the object. Depending on the kinematics model.
            init (bool): Whether to set the initial velocity (default False).
        """

        if isinstance(velocity, list):
            if len(velocity) > self.vel_dim:
                temp_velocity = np.c_[velocity[: self.vel_dim]]
            elif len(velocity) < self.vel_dim:
                temp_velocity = np.c_[velocity + [0] * (self.vel_dim - len(velocity))]
            else:
                temp_velocity = np.c_[velocity]

        elif isinstance(velocity, np.ndarray):
            if velocity.shape[0] > self.vel_dim:
                temp_velocity = velocity[: self.vel_dim]
            elif velocity.shape[0] < self.vel_dim:
                temp_velocity = np.r_[
                    velocity,
                    np.zeros((self.vel_dim - velocity.shape[0], velocity.shape[1])),
                ]
            else:
                temp_velocity = velocity

        assert self._velocity.shape == temp_velocity.shape

        if init:
            self._init_velocity = temp_velocity.copy()

        self._velocity = temp_velocity.copy()

    def set_init_geometry(self, geometry: shapely.geometry.base.BaseGeometry):
        """
        Set the initial geometry of the object.

        Args:
            geometry: The shapely geometry of the object.
        """
        self._init_geometry = geometry

    def set_random_goal(
        self,
        obstacle_list,
        init: bool = False,
        free: bool = True,
        goal_check_radius: float = 0.2,
        range_limits: list = None,
        max_attempts: int = 100,
        ):
        """
        Set random goal(s) in the environment. If free set to True, the goal will be placed only in the free from
        obstacles part of the environment.

        Args:
            obstacle_list: List of objects in the environment
            init (bool): Whether to set the initial goal (default False).
            free (bool): Whether to check that goal is placed in a position free of obstacles.
            goal_check_radius (float): Radius in which to check if the goal is free of obstacles.
            range_limits (list): List of lower and upper bound range limits in which to set the random goal position.
            max_attempts (int): Max number of attempts to place the goal in a position free of obstacles.
        """
        if range_limits is None:
            range_limits = [self.rl, self.rh]

        deque_goals = deque()
        for _ in range(len(self._goal)):
            if free:
                covered_goal = True
                counter = 0
                while covered_goal and counter < max_attempts:
                    goal = random_point_range(range_limits[0], range_limits[1]).flatten().tolist()
                    shape = {"name": "circle", "radius": goal_check_radius}
                    gf = GeometryFactory.create_geometry(**shape)
                    geometry = gf.step(np.c_[goal])
                    covered_goal = any(
                        [
                            shapely.intersects(geometry, obj._geometry)
                            for obj in obstacle_list
                        ]
                    )
                    counter += 1
                if counter == max_attempts:
                    env_param.logger.warning(
                        f"Could not place the goal in a position free of obstacles in {max_attempts} tries"
                    )
            else :
                goal = random_point_range(range_limits[0], range_limits[1]).flatten().tolist()
            deque_goals.append(goal)

        self.set_goal(deque_goals, init=init)

    def set_goal(self, goal: Union[list, np.ndarray] = [10, 10, 0], init: bool = False):
        """
        Set the goal of the object.

        Args:
            goal: The goal of the object [x, y, theta] or [[x, y, theta], [x, y, theta], ...]
            init (bool): Whether to set the initial goal (default False).
        """

        if is_2d_list(goal):
            self._goal = deque(goal)

            if init:
                self._init_goal = self._goal.copy()
            
            return 

        if isinstance(goal, list):
            if len(goal) > self.state_dim:
                temp_goal = np.c_[goal[: self.state_dim]]
            elif len(goal) < self.state_dim:
                temp_goal = np.c_[goal + [0] * (self.state_dim - len(goal))]
            else:
                temp_goal = np.c_[goal]

        elif isinstance(goal, np.ndarray):
            if goal.shape[0] > self.state_dim:
                temp_goal = goal[: self.state_dim]
            elif goal.shape[0] < self.state_dim:
                temp_goal = np.r_[
                    goal, np.zeros((self.state_dim - goal.shape[0], goal.shape[1]))
                ]
            else:
                temp_goal = goal

        assert self.goal.shape == temp_goal.shape

        goal_deque = deque([temp_goal.copy().flatten().tolist()])

        if init:
            self._init_goal = goal_deque

        self._goal = goal_deque

    def geometry_state_transition(self):
        pass

    def input_state_check(self, state: np.ndarray, dim: int = 3):
        """
        Check and adjust the state to match the desired dimension.

        Args:
            state (list): State of the object.
            dim (int): Desired dimension.

        Returns:
            list: Adjusted state.
        """
        if len(state) > dim:
            return state[:dim]
        elif len(state) < dim:
            return state + [0] * (dim - len(state))
        else:
            return state

    def plot(self, ax, **kwargs):
        """
        Plot the object on a given axis.

        Args:
            ax: Matplotlib axis.
            kwargs:
                - show_goal (bool): Whether show the goal position.
                - show_text (bool): Whether show text information.
                - show_arrow (bool): Whether show the velocity arrow.
                - show_uncertainty (bool): Whether show the uncertainty. To be completed.
                - show_trajectory (bool): Whether show the trajectory.
                - show_trail (bool): Whether show the trail.
                - show_sensor (bool): Whether show the sensor.
                - show_fov (bool): Whether show the field of view.
                - trail_freq (int): Frequency of trail display.
                - goal_color (str): Color of the goal marker.
                - traj_color (str): Color of the trajectory.
                - traj_style (str): Style of the trajectory.
                - traj_width (float): Width of the trajectory.
                - traj_alpha (float): Transparency of the trajectory.
                - trail_edgecolor (str): Edge color of the trail.
                - trail_linewidth (float): Width of the trail.
                - trail_alpha (float): Transparency of the trail.
                - trail_fill (bool): Whether fill the trail.
                - trail_color (str): Color of the trail.
                - obj_linestyle (str): Style of the object edge line.

        """
        self.state_re = self.state
        self.goal_re = self.goal
        self.plot_patch_list = []

        self.plot_kwargs.update(kwargs)

        show_goal = self.plot_kwargs.get("show_goal", False)
        show_text = self.plot_kwargs.get("show_text", False)
        show_arrow = self.plot_kwargs.get("show_arrow", False)
        show_uncertainty = self.plot_kwargs.get("show_uncertainty", False)
        show_trajectory = self.plot_kwargs.get("show_trajectory", False)
        show_trail = self.plot_kwargs.get("show_trail", False)
        show_sensor = self.plot_kwargs.get("show_sensor", True)
        show_fov = self.plot_kwargs.get("show_fov", False)
        trail_freq = self.plot_kwargs.get("trail_freq", 2)
        goal_color = self.plot_kwargs.get("goal_color", self.color)


        self.plot_object(ax, **self.plot_kwargs)

        if show_goal:
            self.plot_goal(ax, goal_color)

        if show_text:
            self.plot_text(ax, **self.plot_kwargs)

        if show_arrow:
            self.plot_arrow(ax, **self.plot_kwargs)

        if show_uncertainty:
            self.plot_uncertainty(ax, **self.plot_kwargs)

        if show_trajectory:
            self.plot_trajectory(ax, **self.plot_kwargs)

        if show_trail and world_param.count % trail_freq == 0:
            self.plot_trail(ax, **self.plot_kwargs)

        if show_sensor:
            [sensor.plot(ax, **self.plot_kwargs) for sensor in self.sensors]

        if show_fov:
            self.plot_fov(ax, **self.plot_kwargs)

    def plot_object(self, ax, **kwargs):
        """
        Plot the object itself.

        Args:
            ax: Matplotlib axis.
            **kwargs: Additional plotting options.
        """
        
        obj_linestyle = kwargs.get("obj_linestyle", "-")

        if self.description is None or isinstance(ax, Axes3D):
            x = self.state_re[0, 0]
            y = self.state_re[1, 0]

            if self.shape == "circle":
                object_patch = mpl.patches.Circle(
                    xy=(x, y), radius=self.radius, color=self.color, linestyle=obj_linestyle
                )
                object_patch.set_zorder(3)

                if isinstance(ax, Axes3D):
                    art3d.patch_2d_to_3d(object_patch, z=self.z, zdir="z")

                ax.add_patch(object_patch)

            elif self.shape == "polygon" or self.shape == "rectangle":
                object_patch = mpl.patches.Polygon(xy=self.vertices.T, color=self.color, linestyle=obj_linestyle)
                object_patch.set_zorder(3)

                if isinstance(ax, Axes3D):
                    art3d.patch_2d_to_3d(object_patch, z=self.z)

                ax.add_patch(object_patch)

            elif self.shape == "linestring":

                if isinstance(ax, Axes3D):
                    object_patch = art3d.Line3D(
                        self.vertices[0, :],
                        self.vertices[1, :],
                        zs=self.z * np.ones((3,)),
                        color=self.color,
                    )
                else:
                    object_patch = mpl.lines.Line2D(
                        self.vertices[0, :], self.vertices[1, :], color=self.color, linestyle=obj_linestyle
                    )
                object_patch.set_zorder(3)
                ax.add_line(object_patch)

            elif self.shape == "map":
                return

            self.plot_patch_list.append(object_patch)

        else:
            self.plot_object_image(ax, self.description, **kwargs)

    def plot_object_image(self, ax, description: str, **kwargs):

        # x = self.vertices[0, 0]
        # y = self.vertices[1, 0]

        start_x = self.vertices[0, 0]
        start_y = self.vertices[1, 0]
        r_phi = self.state[2, 0]
        r_phi_ang = 180 * r_phi / pi

        robot_image_path = path_manager.root_path + "/world/description/" + description
        robot_img_read = image.imread(robot_image_path)

        robot_img = ax.imshow(
            robot_img_read,
            extent=[start_x, start_x + self.length, start_y, start_y + self.width],
        )
        trans_data = (
            mtransforms.Affine2D().rotate_deg_around(start_x, start_y, r_phi_ang)
            + ax.transData
        )
        robot_img.set_transform(trans_data)

        self.plot_patch_list.append(robot_img)

    def plot_trajectory(self, ax, keep_length: int = 0, **kwargs):
        """
        Plot the trajectory of the object.

        Args:
            ax: Matplotlib axis.
            keep_length (int): Number of steps to keep in the plot.
            **kwargs: Additional plotting options.
        """
        traj_color = kwargs.get("traj_color", self.color)
        traj_style = kwargs.get("traj_style", "-")
        traj_width = kwargs.get("traj_width", self.width)
        traj_alpha = kwargs.get("traj_alpha", 0.5)

        x_list = [t[0, 0] for t in self.trajectory[-keep_length:]]
        y_list = [t[1, 0] for t in self.trajectory[-keep_length:]]

        linewidth = linewidth_from_data_units(traj_width, ax, "y")

        if isinstance(ax, Axes3D):
            linewidth = traj_width * 10

        solid_capstyle = "round" if self.shape == "circle" else "butt"

        self.plot_line_list.append(
            ax.plot(
                x_list,
                y_list,
                color=traj_color,
                linestyle=traj_style,
                linewidth=linewidth,
                solid_joinstyle="round",
                solid_capstyle=solid_capstyle,
                alpha=traj_alpha,
            )
        )

    def plot_goal(self, ax, goal_color: str = "r"):
        """
        Plot the goal position of the object.

        Args:
            ax: Matplotlib axis.
            goal_color (str): Color of the goal marker.
        """
        goal_x = self.goal_re[0, 0]
        goal_y = self.goal_re[1, 0]

        goal_circle = mpl.patches.Circle(
            xy=(goal_x, goal_y), radius=self.radius, color=goal_color, alpha=0.5
        )

        if isinstance(ax, Axes3D):
            art3d.patch_2d_to_3d(goal_circle, z=self.z)

        goal_circle.set_zorder(1)
        ax.add_patch(goal_circle)

        self.plot_patch_list.append(goal_circle)

    def plot_text(self, ax, **kwargs):
        """
        Plot the text of the object.

        Args:
            ax: Matplotlib axis.
            **kwargs: Additional plotting options.
                text_color (str): Color of the text, default is 'k'.
                text_size (int): Font size of the text, default is 10.
                text_position (list): Position of the text in xy, default is [-radius-0.1, radius+0.1].
        """

        text_color = kwargs.get("text_color", "k")
        text_size = kwargs.get("text_size", 10)
        text_position = kwargs.get("text_position", [-self.radius-0.1, self.radius+0.1])

        x, y = self.state[0, 0], self.state[1, 0]
        
        text = ax.text(x + text_position[0], y + text_position[1], self.abbr, fontsize = text_size, color = text_color)

        self.plot_text_list.append(text)


    def plot_arrow(
        self, ax, arrow_length: float = 0.4, arrow_width: float = 0.6, arrow_color: str = "gold", **kwargs
    ):
        """
        Plot an arrow indicating the velocity orientation of the object.

        Args:
            ax: Matplotlib axis.
            arrow_length (float): Length of the arrow.
            arrow_width (float): Width of the arrow.
            **kwargs: Additional plotting options.
        """
        x = self.state_re[0][0]
        y = self.state_re[1][0]

        theta = (
            atan2(self.velocity_xy[1, 0], self.velocity_xy[0, 0])
            if self.kinematics == "omni"
            else self.state_re[2][0]
        )

        arrow = mpl.patches.Arrow(
            x,
            y,
            arrow_length * cos(theta),
            arrow_length * sin(theta),
            width=arrow_width,
            color=arrow_color,
        )

        if isinstance(ax, Axes3D):
            art3d.patch_2d_to_3d(arrow, z=self.z)

        arrow.set_zorder(3)
        ax.add_patch(arrow)

        self.plot_patch_list.append(arrow)

    def plot_trail(self, ax, **kwargs):
        """
        Plot the trail of the object.

        Args:
            ax: Matplotlib axis.
            **kwargs: Additional plotting options.
        """
        trail_type = kwargs.get("trail_type", self.shape)
        trail_edgecolor = kwargs.get("trail_edgecolor", self.color)
        trail_linewidth = kwargs.get("trail_linewidth", 0.8)
        trail_alpha = kwargs.get("trail_alpha", 0.7)
        trail_fill = kwargs.get("trail_fill", False)
        trail_color = kwargs.get("trail_color", self.color)

        r_phi_ang = 180 * self.state[2, 0] / pi

        if trail_type == "rectangle" or trail_type == "polygon":
            start_x = self.vertices[0, 0]
            start_y = self.vertices[1, 0]

            car_rect = mpl.patches.Rectangle(
                xy=(start_x, start_y),
                width=self.length,
                height=self.width,
                angle=r_phi_ang,
                edgecolor=trail_edgecolor,
                fill=False,
                alpha=trail_alpha,
                linewidth=trail_linewidth,
                facecolor=trail_color,
            )

            if isinstance(ax, Axes3D):
                art3d.patch_2d_to_3d(car_rect, z=self.z)

            ax.add_patch(car_rect)

        elif trail_type == "circle":
            car_circle = mpl.patches.Circle(
                xy=self.centroid,
                radius=self.radius,
                edgecolor=trail_edgecolor,
                fill=trail_fill,
                alpha=trail_alpha,
                facecolor=trail_color,
            )

            if isinstance(ax, Axes3D):
                art3d.patch_2d_to_3d(car_circle, z=self.z)

            ax.add_patch(car_circle)

    def plot_fov(self, ax, **kwargs):
        """
        Plot the field of view of the object.

        Args:
            ax: Matplotlib axis.
            **kwargs: Additional plotting options.
                fov_color (str): Color of the field of view. Default is 'lightblue'.
                fov_edge_color (str): Edge color of the field of view. Default is 'blue'.
        """

        fov_color = kwargs.get("fov_color", "lightblue")
        fov_edge_color = kwargs.get("fov_edge_color", "blue")

        direction = self.state[2, 0] if self.state_dim >= 3 else 0
        position = self.state[:2, 0]

        start_angle = direction - self.fov / 2
        end_angle = direction + self.fov / 2

        start_degree = 180 * start_angle / pi
        end_degree = 180 * end_angle / pi

        fov_wedge = Wedge(
            position,
            self.fov_radius,
            start_degree,
            end_degree,
            facecolor=fov_color,
            alpha=0.5,
            edgecolor=fov_edge_color,
        )

        ax.add_patch(fov_wedge)

        self.plot_patch_list.append(fov_wedge)

    def plot_uncertainty(self, ax, **kwargs):
        """
        To be completed.
        """
        pass

    def plot_clear(self):
        """
        Clear all plotted elements from the axis.
        """
        [patch.remove() for patch in self.plot_patch_list]
        [line.pop(0).remove() for line in self.plot_line_list]
        [text.remove() for text in self.plot_text_list]

        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

        [sensor.plot_clear() for sensor in self.sensors]

    def done(self):
        """
        Check if the object has completed its task.

        Returns:
            bool: True if the task is done, False otherwise.
        """
        if self.stop_flag or self.arrive_flag:
            return True
        else:
            return False

    def reset(self):
        """
        Reset the object to its initial state.
        """
        self._state = self._init_state.copy()
        self._goal = self._init_goal.copy()
        self._velocity = self._init_velocity.copy()

        self.collision_flag = False
        self.arrive_flag = False
        self.stop_flag = False
        self.trajectory = []

    def remove(self):
        """
        Remove the object from the environment.
        """
        del self


    def get_vel_range(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Get the velocity range considering acceleration limits.

        Returns:
            tuple: Minimum and maximum velocities.
        """
        min_vel = np.maximum(
            self.vel_min, self._velocity - self.info.acce * world_param.step_time
        )
        max_vel = np.minimum(
            self.vel_max, self._velocity + self.info.acce * world_param.step_time
        )

        return min_vel, max_vel

    def get_info(self) -> ObjectInfo:
        """
        Get object information.

        Returns:
            ObjectInfo: Information about the object.
        """
        return self.info

    def get_obstacle_info(self) -> ObstacleInfo:
        """
        Get information about the object as an obstacle.

        Returns:
            ObstacleInfo: Obstacle-related information, including state, vertices, velocity, and radius.
        """
        return ObstacleInfo(
            self.state[:2, :],
            self.vertices,
            self._velocity,
            self.radius,
            self.G,
            self.h,
            self.cone_type,
            self.convex_flag,
        )

    def get_init_Gh(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Get the generalized inequality matrices G and h for the convex object's.

        Returns:
            G matrix and h vector.
        """
        return self.gf.get_init_Gh()

    def get_Gh(self) -> tuple[np.ndarray, np.ndarray]:

        """
        Get the generalized inequality matrices G and h for the convex object's.
        """
        return self.gf.get_Gh(center=self.position, radius=self.radius, vertices=self.vertices)

    @property
    def name(self) -> str:

        '''
        Get the name of the object.

        Returns:
            str: The name of the object.
        '''

        return self.info.role + "_" + str(self.id)

    @property
    def abbr(self) -> str:

        '''
        Get the abbreviation of the object.

        Returns:
            str: The abbreviation of the object.
        '''

        return self.info.role[0] + str(self.id)

    @property
    def shape(self) -> str:

        '''
        Get the shape name of the object.

        Returns:
            str: The shape name of the object.
        '''

        return self.gf.name

    @property
    def z(self) -> float:

        '''
        Get the z coordinate of the object. For 3D object, the z coordinate is the height of the object, for 2D object, the z coordinate is 0.

        Returns:
            float: The z coordinate of the object.
        '''

        return self.state[2, 0] if self.state_dim >= 6 else 0

    @property
    def kinematics(self) -> str:

        '''
        Get the kinematics name of the object.

        Returns:
            str: The kinematics name of the object.
        '''

        return self.kf.name if self.kf is not None else None

    @property
    def geometry(self) -> shapely.geometry.base.BaseGeometry:

        '''
        Get the geometry Instance of the object.

        Returns:
            shapely.geometry.base.BaseGeometry: The geometry of the object.
        '''

        return self._geometry

    @property
    def centroid(self) -> np.ndarray:

        '''
        Get the centroid of the object.

        Returns:
            np.ndarray: The centroid of the object.
        '''

        return self._geometry.centroid.coords._coords.T

    @property
    def id(self) -> int:

        '''
        Get the id of the object.

        Returns:
            int: The id of the object.
        '''

        return self._id

    @property
    def state(self) -> np.ndarray:

        '''
        Get the state of the object.

        Returns:
            np.ndarray: The state of the object.
        '''

        return self._state

    @property
    def velocity(self) -> np.ndarray:

        '''
        Get the velocity of the object.

        Returns:
            np.ndarray: The velocity of the object.
        '''

        return self._velocity

    @property
    def goal(self) -> np.ndarray:

        '''
        Get the goal of the object.

        Returns:
            np.ndarray: The goal of the object.
        '''

        return np.c_[self._goal[0]]

    @property
    def position(self) -> np.ndarray:

        '''
        Get the position of the object.

        Returns:
            np.ndarray: The position of the object .
        '''

        return self._state[:2]

    @property
    def radius(self) -> float:

        '''
        Get the radius of the object.

        Returns:
            float: The radius of the object.
        '''

        return self.gf.radius

    @property
    def length(self) -> float:

        '''
        Get the length of the object.

        Returns:
            float: The length of the object.
        '''

        return self.gf.length

    @property
    def width(self) -> float:

        '''
        Get the width of the object.

        Returns:
            float: The width of the object.
        '''

        return self.gf.width

    @property
    def wheelbase(self) -> float:

        '''
        Get the wheelbase of the object.

        Returns:
            float: The wheelbase of the object.
        '''

        return self.gf.wheelbase

    @property
    def radius_extend(self) -> float:

        '''
        Get the radius of the object with a buffer.

        Returns:
            float: The radius of the object with a buffer.
        '''

        return self.radius + 0.1

    @property
    def arrive(self) -> bool:

        '''
        Get the arrive flag of the object.

        Returns:
            bool: The arrive flag of the object.
        '''

        return self.arrive_flag

    @property
    def collision(self) -> bool:

        '''
        Get the collision flag of the object.

        Returns:
            bool: The collision flag of the object.
        '''

        return self.collision_flag

    @property
    def ineq_Ab(self):

        '''
        Get the inequality matrix A and b of the object.

        Returns:
            tuple: The inequality matrix A and b of the object.
        '''

        return self.get_inequality_Ab()

    @property
    def vertices(self) -> np.ndarray:

        '''
        Get the vertices of the object.

        Returns:
            np.ndarray: The vertices of the object.
        '''

        return self.gf.vertices

    @property
    def external_objects(self):

        '''
        The environment objects that are not the self object.

        Returns:
            list: The environment objects that are not the self object.
        '''

        return [obj for obj in env_param.objects if self.id != obj.id]

    @property
    def ego_object(self):

        '''
        Get the ego object.

        Returns:
            ObjectBase: The ego object.
        '''

        return self

    @property
    def desired_omni_vel(self, goal_threshold=0.1):
        """
        Calculate the desired omnidirectional velocity.

        Args:
            goal_threshold (float): Threshold for goal proximity.

        Returns:
            np.ndarray: Desired velocity [vx, vy].
        """
        dis, radian = relative_position(self.state, self.goal)

        if dis > goal_threshold:
            vx = self.vel_max[0, 0] * cos(radian)
            vy = self.vel_max[1, 0] * sin(radian)
        else:
            vx = 0
            vy = 0

        return np.array([[vx], [vy]])

    @property
    def rvo_neighbors(self):
        """
        Get the list of RVO neighbors.
        Returns:
            list: List of RVO neighbor states [x, y, vx, vy, radius].
        """
        return [
            obj.rvo_neighbor_state for obj in self.external_objects if not obj.unobstructed
        ]

    @property
    def rvo_neighbor_state(self):
        """
        Get the RVO state for this object.

        Returns:
            list: State [x, y, vx, vy, radius].
        """
        return [
            self.state[0, 0],
            self.state[1, 0],
            self.velocity_xy[0, 0],
            self.velocity_xy[1, 0],
            self.radius_extend,
        ]

    @property
    def rvo_state(self):
        """
        Get the full RVO state including desired velocity.

        Returns:
            list: State [x, y, vx, vy, radius, vx_des, vy_des, theta].
        """
        vx_des, vy_des = self.desired_omni_vel[:, 0]
        return [
            self.state[0, 0],
            self.state[1, 0],
            self.velocity_xy[0, 0],
            self.velocity_xy[1, 0],
            self.radius_extend,
            vx_des,
            vy_des,
            self.state[2, 0],
        ]

    @property
    def velocity_xy(self):
        """
        Get the velocity in x and y directions.

        Returns:
            (2*1) np.ndarray: Velocity [vx, vy].
        """
        if self.kinematics == "omni":
            return self._velocity
        elif self.kinematics == "diff" or self.kinematics == "acker":
            return diff_to_omni(self.state[2, 0], self._velocity)
        else:
            return np.zeros((2, 1))

    @property
    def beh_config(self):
        
        '''
        Get the behavior configuration of the object.

        Returns:
            dict: The behavior configuration of the object.
        '''

        return self.obj_behavior.behavior_dict
