import itertools
import math
from collections import deque
from dataclasses import dataclass
from math import cos, pi, sin
from typing import Any, ClassVar

import numpy as np
import shapely
from shapely.geometry.base import BaseGeometry

from irsim.lib import Behavior, GeometryFactory, KinematicsFactory
from irsim.util.util import (
    WrapTo2Pi,
    WrapToPi,
    WrapToRegion,
    check_unknown_kwargs,
    is_2d_list,
    random_point_range,
    relative_position,
    to_numpy,
    vertices_transform,
)
from irsim.world.object_plot import ObjectPlot
from irsim.world.sensors.sensor_factory import SensorFactory


@dataclass
class ObjectInfo:
    """Snapshot of an object's public state used by behaviors and planners.

    Behavior functions receive this structure instead of the full object when
    only immutable configuration-style fields are needed. Additional fields can
    be attached with :meth:`add_property` for custom behaviors.
    """

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
    name: str

    def add_property(self, key, value):
        """Attach an additional field to this info snapshot."""
        setattr(self, key, value)


@dataclass
class ObstacleInfo:
    """Geometry and motion snapshot exposed for collision-aware behaviors."""

    center: np.ndarray
    vertex: np.ndarray
    velocity: np.ndarray
    radius: float
    G: np.ndarray
    h: np.ndarray
    cone_type: str
    convex_flag: bool

    def add_property(self, key, value):
        """Attach an additional field to this obstacle snapshot."""
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
            the object's geometry, including ``name`` (for example, ``circle`` or
            ``rectangle``) and associated parameters. If omitted, a circle with radius
            ``1`` is created; an explicit ``{"name": "circle"}`` uses radius ``0.2``.
        kinematics (dict): Parameters defining the kinematics of the object.
            Includes kinematic model and any necessary parameters. If None, no kinematics model is applied.
            Defaults to None.
        state (list of float): Initial state vector [x, y, theta, ...].
            The state can have more dimensions depending on `state_dim`. Excess dimensions are truncated,
            and missing dimensions are filled with zeros. Defaults to [0, 0, 0].
        velocity (list of float): Initial velocity vector [vx, vy] or according to the kinematics model.
            Defaults to [0, 0].
        goal (list of float or list of list of float): Goal state vector [x, y, theta, ...] or [[x, y, theta], [x, y, theta], ...] for multiple goals
            Used by behaviors to determine the desired movement. Defaults to None.
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
            Can be a behavior name (str) or a dictionary with behavior parameters.
            If None and no group behavior is configured, the object remains static unless an external velocity is supplied.
        group_behavior (dict): Shared behavior defaults for objects in the same group.
            When an object's own behavior configuration is empty or missing, the
            group behavior will be used as a fallback and exposed via `beh_config`.
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

            - plot (dict): Plotting options for the object.
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

    _VALID_PARAMS: ClassVar[set[str]] = {
        "shape",
        "kinematics",
        "state",
        "velocity",
        "goal",
        "role",
        "color",
        "static",
        "vel_min",
        "vel_max",
        "acce",
        "angle_range",
        "behavior",
        "group_behavior",
        "goal_threshold",
        "sensors",
        "arrive_mode",
        "description",
        "group",
        "group_name",
        "state_dim",
        "vel_dim",
        "unobstructed",
        "fov",
        "fov_radius",
        "name",
        "plot",
        # consumed by ObjectFactory before reaching __init__
        "number",
        "distribution",
    }

    def __init__(
        self,
        shape: dict | None = None,
        kinematics: dict | None = None,
        state: list | None = None,
        velocity: list | None = None,
        goal: list | None = None,
        role: str = "obstacle",
        color: str = "k",
        static: bool = False,
        vel_min: list | None = None,
        vel_max: list | None = None,
        acce: list | None = None,
        angle_range: list | None = None,
        behavior: dict | None = None,
        group_behavior: dict | None = None,
        goal_threshold: float = 0.1,
        sensors: dict | None = None,
        arrive_mode: str = "position",
        description: str | None = None,
        group: int = 0,
        group_name: str | None = None,
        state_dim: int | None = None,
        vel_dim: int | None = None,
        unobstructed: bool = False,
        fov: float | None = None,
        fov_radius: float | None = None,
        name: str | None = None,
        **kwargs,
    ) -> None:
        """
        Initialize an ObjectBase instance.

        This method sets up a new ObjectBase object with the specified parameters, initializing its
        geometry, kinematics, behaviors, sensors, and other properties relevant to simulation.

        The initialization process includes:
        - Setting up geometry handlers and collision detection
        - Configuring kinematics models for movement
        - Initializing state vectors and goal management
        - Setting up behaviors and sensor systems
        - Configuring visualization and plotting options

        Note:
            All parameters are documented in the class docstring above. Refer to the
            :py:class:`ObjectBase` class documentation for detailed parameter descriptions.

        Raises:
            ValueError: If dimension parameters do not match the provided shapes or
                if input parameters are invalid.
        """

        # --- 1. Identity ---
        self._env = None
        self._id = next(ObjectBase.id_iter)
        self._name = name
        self.role = role
        self.group = group
        self._group_name = group_name
        self.description = description
        self.color = color

        # --- 2. Geometry & kinematics handlers ---
        if shape is None:
            self.logger.warning(
                f"No shape provided for object {self._id}, using default circle"
            )
            shape = {"name": "circle", "radius": 1, "center": [0, 0]}

        self.gf = GeometryFactory.create_geometry(**shape)
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

        # --- 3. Dimensions (derived from handlers) ---
        action_dim = self.kf.action_dim if self.kf else 2
        self.state_dim = state_dim if state_dim is not None else self.state_shape[0]
        self.state_shape = (
            (self.state_dim, 1) if state_dim is not None else self.state_shape
        )
        self.vel_dim = vel_dim if vel_dim is not None else action_dim
        self.vel_shape = (self.vel_dim, 1)

        # --- 4. Resolve defaults from kf ---
        if angle_range is None:
            angle_range = [-pi, pi]

        if self.kf is not None:
            acce = self.kf.acce if acce is None else acce
            vel_max = self.kf.vel_max if vel_max is None else vel_max
            vel_min = self.kf.vel_min if vel_min is None else vel_min
        else:
            acce = acce or [float("inf"), float("inf")]
            vel_max = vel_max or [1, 1]
            vel_min = vel_min or [-1, -1]

        # --- 5. State & velocity ---
        if state is None:
            state = [0, 0, 0]
        if velocity is None:
            velocity = [0] * action_dim

        state = self.input_state_check(state, self.state_dim)
        self._state = np.c_[state]
        self._init_state = np.c_[state]

        self._velocity = np.c_[velocity]
        self._init_velocity = np.c_[velocity]

        self.vel_min = np.c_[vel_min]
        self.vel_max = np.c_[vel_max]
        self.static = static if self.kf is not None else True

        # --- 6. Goal ---
        self._goal = (
            deque(goal)
            if goal is not None and is_2d_list(goal)
            else deque([goal])
            if goal is not None
            else None
        )
        self._goal_vertices = (
            vertices_transform(self.original_vertices, self.goal)
            if self.goal is not None
            else None
        )
        self._init_goal = self._goal.copy() if self._goal is not None else None
        self._init_goal_vertices = (
            self._goal_vertices.copy() if self._goal_vertices is not None else None
        )
        self.goal_threshold = goal_threshold
        self.arrive_mode = arrive_mode

        # --- 7. Geometry instance ---
        self._geometry = self.gf.step(self.state) if self.gf is not None else None
        self._geometry_valid = (
            shapely.is_valid(self._geometry) if self._geometry is not None else False
        )

        # --- 8. ObjectInfo ---
        self.info = ObjectInfo(
            self._id,
            self.shape,
            self.kinematics,
            role,
            color,
            static,
            np.c_[goal],
            self.vel_min,
            self.vel_max,
            np.c_[acce],
            np.c_[angle_range],
            goal_threshold,
            self.wheelbase,
            self.G,
            self.h,
            self.cone_type,
            self.convex_flag,
            self.name,
        )
        self.obstacle_info = None
        self.trajectory = []

        # --- 9. Sensors ---
        sf = SensorFactory()
        self.lidar = None
        if sensors is not None:
            self.sensors = [
                sf.create_sensor(self._state[0:3], self._id, **sensor_kwargs)
                for sensor_kwargs in sensors
            ]
            for sensor in self.sensors:
                sensor.parent = self

            self.lidar = next(
                (
                    sensor
                    for sensor in self.sensors
                    if sensor.sensor_type in {"lidar2d", "fmcw_lidar2d"}
                ),
                None,
            )
        else:
            self.sensors = []

        if fov is None:
            self.fov = (
                WrapTo2Pi(self.lidar.angle_range) if self.lidar is not None else None
            )
            self.fov_radius = self.lidar.range_max if self.lidar is not None else None
        else:
            self.fov = WrapTo2Pi(fov)
            self.fov_radius = fov_radius

        # --- 10. Behavior ---
        self.obj_behavior = Behavior(self.info, behavior)
        self.group_behavior_dict = group_behavior if group_behavior is not None else {}

        self.rl = self.beh_config.get("range_low", [0, 0, -pi])
        self.rh = self.beh_config.get("range_high", [10, 10, pi])
        self.wander = self.beh_config.get("wander", False)
        self.loop = self.beh_config.get("loop", False)

        if self.wander and self.loop:
            self.logger.warning(
                f"Object {self.id}: Both 'wander' and 'loop' are enabled. "
                "'wander' takes priority, 'loop' will be disabled."
            )
            self.loop = False

        if self.wander:
            self._goal = deque([random_point_range(self.rl, self.rh)])

        # --- 11. Flags ---
        self.stop_flag = False
        self.arrive_flag = False
        self.collision_flag = False
        self.unobstructed = unobstructed

        # --- 12. Plot state ---
        self.plot_kwargs = kwargs.get("plot", {})
        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []
        self._custom_text: str | None = None
        self._custom_goal_text: str | None = None
        self.collision_obj = []
        self.plot_trail_list = []
        self._object_plot = ObjectPlot(self)

        # --- 13. Validate kwargs ---
        check_unknown_kwargs(kwargs, self._VALID_PARAMS, context=f" in '{role}' config")

    def __eq__(self, o: "ObjectBase") -> bool:
        if isinstance(o, ObjectBase):
            return self._id == o._id
        return False

    def __hash__(self) -> int:
        return self._id

    def __str__(self) -> str:
        return f"ObjectBase: {self._id}"

    @classmethod
    def reset_id_iter(cls, start: int = 0, step: int = 1):
        """reset the id iterator"""
        cls.id_iter = itertools.count(start, step)

    def step(
        self,
        velocity: np.ndarray | None = None,
        sensor_step: bool = True,
        **kwargs: Any,
    ):
        """
        Perform a single simulation step, updating the object's state and sensors.

        This method advances the object by one time step, integrating the given velocity
        or behavior-generated velocity to update the object's position, orientation, and
        other state variables. It also updates sensors and checks for collisions.

        Args:
            velocity (np.ndarray, optional): Desired velocity for this step.
                If None, the object will use its behavior system to generate velocity.
                The shape and meaning depend on the kinematics model:

                - Differential: [linear_velocity, angular_velocity]
                - Omnidirectional: [velocity_x, velocity_y]
                - Ackermann: [linear_velocity, steering_angle]

            **kwargs: Additional parameters passed to behavior generation and processing.

        Returns:
            np.ndarray: The updated state vector of the object after the step.
            Returns the current state unchanged if the object is static or stopped.

        Note:
            - Static objects (static=True) will not move and return their current state
            - Objects with stop_flag=True will halt and return their current state
            - The method automatically handles sensor updates and trajectory recording
        """

        if self.static or self.stop_flag:
            self._velocity = np.zeros(self.vel_shape)
            # velocity just got zeroed; invalidate per-tick caches so
            # other agents don't perceive a stopped object as still moving.
            self._invalidate_reactive_cache()
            return self.state
        self.pre_process()
        behavior_vel = self.gen_behavior_vel(velocity)
        new_state = self.kf.step(self.state, behavior_vel, self._world_param.step_time)
        next_state = self.mid_process(new_state)

        self._state = next_state
        self._velocity = behavior_vel
        self._geometry = self.gf.step(self.state)
        self._geometry_valid = shapely.is_valid(self._geometry)
        # state/velocity/geometry changed; invalidate per-tick caches
        # used by reactive behaviors (SFM/RVO read these once per
        # neighbour). Non-static linestring objects have their vertices
        # re-transformed by ``gf.step(state)`` above, so the segment
        # cache must drop too; static objects skip this whole method
        # (early-return at top), so their segment cache remains valid
        # for the lifetime of the env.
        self._invalidate_reactive_cache()

        if sensor_step:
            self.sensor_step()

        self.post_process()
        self.trajectory.append(self.state.copy())
        return next_state

    def sensor_step(self):
        """
        Update all sensors for the current state.
        """
        [sensor.step(self.state[0:3]) for sensor in self.sensors]

    def check_status(self):
        """
        Check the current status of the object, including arrival and collision detection.

        This method evaluates collision detection and sets stop flags based on the collision mode.
        It also handles different collision modes like 'stop', 'reactive', 'unobstructed',
        and 'unobstructed_obstacles'.
        """
        self.check_arrive_status()
        self.check_collision_status()

        if self._world_param.collision_mode == "stop":
            self.stop_flag = any(not obj.unobstructed for obj in self.collision_obj)

        elif self._world_param.collision_mode == "reactive":
            "currently same as unobstructed: to be further implemented"
            pass

        elif self._world_param.collision_mode == "unobstructed":
            pass

        elif self._world_param.collision_mode == "unobstructed_obstacles":
            if self.role == "robot":
                self.stop_flag = any(not obj.unobstructed for obj in self.collision_obj)
            elif self.role == "obstacle":
                self.stop_flag = False
        else:
            if self._world_param.count % 50 == 0 and self.role == "robot":
                self.logger.warning(
                    f"collision mode {self._world_param.collision_mode} is not defined within [stop, reactive, unobstructed, unobstructed_obstacles], the unobstructed mode is used"
                )

    def check_arrive_status(self):
        """
        Check if the object has arrived at its goal position.

        The arrival detection depends on the arrive_mode setting:
        - "state": Compares full state (x, y, theta)
        - "position": Compares only position (x, y)

        Updates the arrive_flag and handles multiple goals by removing completed ones.
        """

        if self.check_arrive(self.goal):
            if len(self._goal) == 1:
                self.arrive_flag = True
            else:
                self._goal.popleft()
                self.arrive_flag = False
        else:
            self.arrive_flag = False

    def check_arrive(self, goal, threshold=None):
        """
        Check if the object has arrived at a given goal.

        Args:
            goal (np.ndarray): Goal state to check arrival against.
            threshold (float, optional): Distance threshold for arrival.
                Defaults to self.goal_threshold if not provided.

        Returns:
            bool: True if the object is within the threshold, False otherwise.
        """
        if goal is None:
            return False

        if threshold is None:
            threshold = self.goal_threshold

        if self.arrive_mode == "state":
            diff = np.linalg.norm(self.state[:3] - goal[:3])
        elif self.arrive_mode == "position":
            diff = np.linalg.norm(self.state[:2] - goal[:2])
        else:
            raise ValueError(
                f"Unsupported arrive_mode '{self.arrive_mode}'. "
                "Supported modes are 'state' and 'position'."
            )

        return diff < threshold

    def check_collision_status(self):
        """
        Check if the object is in collision with other objects in the environment.

        This method queries possible collision objects from the geometry tree and
        checks for intersections. It logs collision warnings for robots and updates
        the collision_flag and collision_obj list.
        """
        collision_flags = []
        self.collision_obj = []

        for obj in self.possible_collision_objects:
            if self.check_collision(obj):
                collision_flags.append(True)
                self.collision_obj.append(obj)

                if self.role == "robot" and not self.collision_flag:
                    self.logger.warning(
                        f"{self.name} collided with {obj.name} at state {np.round(self.state[:3, 0], 2).tolist()}"
                    )
            else:
                collision_flags.append(False)

        self.collision_flag = any(collision_flags)

    def check_collision(self, obj):
        """
        Check collision with another object.

        Args:
            obj (ObjectBase): Another object to check collision with.

        Returns:
            bool: True if collision occurs, False otherwise.
        """

        if obj.shape == "map":
            return obj.is_collision(self.geometry)

        return shapely.intersects(self.geometry, obj._geometry)

    def gen_behavior_vel(self, velocity: np.ndarray | None = None) -> np.ndarray:
        """
        Generate behavior-influenced velocity for the object.

        This method adjusts the desired velocity based on the object's behavior configurations.
        If no desired velocity is provided (`velocity` is None), the method may generate a default
        velocity or issue warnings based on the object's role and behavior settings.

        Note:
            Wander goal renewal (sampling a new random goal upon arrival) is handled in pre_process().

        Args:
            velocity (Optional[np.ndarray]): Desired velocity vector. If None, the method determines
                the velocity based on behavior configurations. Defaults to None.

        Returns:
            np.ndarray: Velocity vector adjusted based on behavior configurations and constraints.

        Logging:
            Emits a warning if `velocity` is None and no behavior configuration is set for a robot.
        """
        min_vel, max_vel = self.get_vel_range()

        if velocity is None:
            if self.beh_config is None:
                if self.role == "robot":
                    self.logger.warning(
                        "behavior and input velocity is not defined, robot will stay static"
                    )

                return np.zeros_like(self._velocity)

            behavior_vel = self.obj_behavior.gen_vel(
                self.ego_object, self.external_objects
            )

        else:
            velocity = to_numpy(velocity, expected_shape=self.vel_shape)

            behavior_vel = velocity

        # clip the behavior_vel by maximum and minimum limits
        if (behavior_vel < (min_vel - 0.01)).any():
            self.logger.warning(
                f"Input velocity {np.round(behavior_vel.flatten(), 2)} below min {np.round(min_vel.flatten(), 2)}. Clipped due to acceleration limit."
            )

        elif (behavior_vel > (max_vel + 0.01)).any():
            self.logger.warning(
                f"Input velocity {np.round(behavior_vel.flatten(), 2)} exceeds max {np.round(max_vel.flatten(), 2)}. Clipped due to acceleration limit."
            )

        return np.clip(behavior_vel, min_vel, max_vel)

    def pre_process(self):
        """
        Perform pre-processing before stepping the object.

        This method is called before velocity generation and state updates.
        Can be overridden by subclasses to implement custom pre-processing logic.

        Default behavior:
            - If `wander` is enabled and the object has just arrived (`arrive_flag`),
              sample a new random goal within [`rl`, `rh`] and clear `arrive_flag`.
            - If `loop` is enabled and the object has just arrived (`arrive_flag`),
              reset goals to initial waypoints and clear `arrive_flag`.
        """

        if self.wander and self.arrive_flag:
            self._goal = deque([random_point_range(self.rl, self.rh)])
            self.arrive_flag = False

        if self.loop and self.arrive_flag and self._init_goal:
            if len(self._init_goal) > 1:
                self._goal = self._init_goal.copy()
            else:
                # Single goal: cycle between start position and goal
                start_pos = self._init_state[0:2, 0].tolist()
                goal_pos = self._init_goal[0][0:2]
                self._goal = deque([goal_pos, start_pos])
            self.arrive_flag = False

    def post_process(self):
        """
        Perform post-processing after stepping the object.

        This method is called after state updates and sensor updates.
        Can be overridden by subclasses to implement custom post-processing logic.
        """
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
            state[2, 0] = WrapToRegion(
                float(state[2, 0]), self.info.angle_range.flatten().tolist()
            )

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
        """
        Get the lidar scan of the object.

        Returns:
            dict: Lidar scan data containing range and angle information.
        """
        return self.lidar.get_scan()

    def get_lidar_points(self):
        """
        Get the lidar scan points of the object.

        Returns:
            np.ndarray: Array of lidar scan points.
        """
        return self.lidar.get_points()

    def get_lidar_offset(self):
        """
        Get the lidar offset relative to the object.

        Returns:
            list: Lidar offset [x, y, theta] relative to the object center.
        """
        return self.lidar.get_offset()

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

        return bool(
            fov_diff <= self.fov / 2 and distance_do - radius_do <= self.fov_radius
        )

    def set_state(self, state: list | np.ndarray | None = None, init: bool = False):
        """
        Set the current state of the object.

        This method updates the object's position, orientation, and other state variables.
        It also updates the object's geometry representation to match the new state.

        Args:
            state (Union[list, np.ndarray]): The new state vector for the object.
                The format depends on the object's state dimension:

                - 2D objects: [x, y, theta] where theta is orientation in radians
                - 3D objects: [x, y, z, roll, pitch, yaw] or similar based on configuration

                Must match the object's state_dim dimension.

            init (bool): Whether to also set this as the initial state for reset purposes.
                If True, the object will return to this state when reset() is called.
                Default is False.

        Raises:
            AssertionError: If the state dimension doesn't match the expected state_dim.

        Example:
            >>> # Set robot position and orientation
            >>> robot.set_state([5.0, 3.0, 1.57])  # x=5, y=3, facing pi/2 radians
            >>>
            >>> # Set as initial state for resets
            >>> robot.set_state([0, 0, 0], init=True)
        """
        if state is None:
            state = [0, 0, 0]

        temp_state = to_numpy(state, expected_shape=self.state_shape)

        if init:
            self._init_state = temp_state.copy()

        self._state = temp_state.copy()
        self._geometry = self.gf.step(self.state)
        self._geometry_valid = shapely.is_valid(self._geometry)
        self._invalidate_reactive_cache()

    def set_velocity(
        self, velocity: list | np.ndarray | None = None, init: bool = False
    ) -> None:
        """
        Set the velocity of the object.

        Args:
            velocity: The velocity of the object. Depending on the kinematics model.
            init (bool): Whether to set the initial velocity (default False).
        """

        temp_velocity = to_numpy(
            velocity, default=np.zeros(self.vel_shape), expected_shape=self.vel_shape
        )

        if init:
            self._init_velocity = temp_velocity.copy()

        self._velocity = temp_velocity.copy()
        self._invalidate_reactive_cache()

    def set_original_geometry(self, geometry: BaseGeometry):
        """
        Set the original geometry of the object.

        Args:
            geometry (BaseGeometry): Shapely geometry to use as the new original geometry.
                Subsequent geometry updates will be transformed from this base.
        """
        self.gf._original_geometry = geometry

    def set_random_goal(
        self,
        obstacle_list,
        init: bool = False,
        free: bool = True,
        goal_check_radius: float = 0.2,
        range_limits: list | None = None,
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
                    goal = random_point_range(range_limits[0], range_limits[1])
                    shape = {"name": "circle", "radius": goal_check_radius}
                    gf = GeometryFactory.create_geometry(**shape)
                    geometry = gf.step(np.c_[goal])
                    covered_goal = any(
                        shapely.intersects(geometry, obj._geometry)
                        for obj in obstacle_list
                    )
                    counter += 1
                if counter == max_attempts:
                    self.logger.warning(
                        f"Could not place the goal in a position free of obstacles in {max_attempts} tries"
                    )
            else:
                goal = random_point_range(range_limits[0], range_limits[1])
            deque_goals.append(goal)

        self.set_goal(deque_goals, init=init)

    def set_goal(self, goal: list | np.ndarray | None = None, init: bool = False):
        """
        Set the goal(s) for the object to navigate towards.

        This method configures the target location(s) that the object's behavior system
        will attempt to reach. Multiple goals can be provided for sequential navigation.

        Args:
            goal (Union[list, np.ndarray]): The goal specification. Can be:

                - Single goal: [x, y, theta] for one target location
                - Multiple goals: [[x1, y1, theta1], [x2, y2, theta2], ...] for sequential targets
                - None: Clear all goals

                The theta component specifies the desired final orientation in radians.

            init (bool): Whether to also set this as the initial goal for reset purposes.
                If True, these goals will be restored when reset() is called.
                Default is False.

        Example:
            >>> # Set single goal
            >>> robot.set_goal([10.0, 5.0, 0.0])  # Move to (10,5) facing East
            >>>
            >>> # Set multiple sequential goals
            >>> waypoints = [[5, 0, 0], [10, 5, 1.57], [0, 10, 3.14]]
            >>> robot.set_goal(waypoints)
            >>>
            >>> # Clear goals
            >>> robot.set_goal(None)
        """

        if goal is None:
            self._goal = None

            if init:
                self._init_goal = None

            return

        if isinstance(goal, list) and is_2d_list(goal):
            self._goal = deque(goal)

            if init:
                self._init_goal = self._goal.copy()

            return

        if isinstance(goal, deque):
            self._goal = goal

            if init:
                self._init_goal = self._goal.copy()

            return

        if isinstance(goal, list):
            assert len(goal) >= 2, (
                f"The goal dimension is not correct. Your input goal length is {len(goal)} and the goal dimension should be at least 2"
            )
            temp_goal = np.c_[goal]

        elif isinstance(goal, np.ndarray):
            assert goal.shape[0] >= 2, (
                f"The goal dimension is not correct. Your input goal dimension is {goal.shape[0]} and the goal dimension should be at least 2"
            )
            temp_goal = goal

        goal_deque = deque([temp_goal.copy().flatten().tolist()])

        if init:
            self._init_goal = goal_deque

        self._goal = goal_deque

    def append_goal(self, goal: list | np.ndarray):
        """
        Append a goal to the goal list.
        """

        if isinstance(goal, list):
            self._goal.append(goal)

        elif isinstance(goal, np.ndarray):
            self._goal.append(goal.flatten().tolist())

    def set_laser_color(
        self, laser_indices, laser_color: str = "cyan", alpha: float = 0.3
    ):
        """
        Set the color of the lasers.

        Args:
            laser_indices (list): The indices of the lasers to set the color.
            laser_color (str): The color to set the lasers. Default is 'cyan'.
            alpha (float): The transparency of the lasers. Default is 0.3.
        """

        if self.lidar is not None:
            self.lidar.set_laser_color(laser_indices, laser_color, alpha)
        else:
            self.logger.warning("No lidar sensor found for this object.")

    def input_state_check(self, state: list, dim: int = 3):
        """
        Check and adjust the state to match the desired dimension.

        Args:
            state (list): State of the object.
            dim (int): Desired dimension. Defaults to 3.

        Returns:
            list: Adjusted state.
        """

        if len(state) > dim:
            if self.role == "robot":
                self.logger.warning(
                    f"The state dimension {len(state)} of {self.abbr} is larger than the desired dimension {dim}, the state dimension is truncated"
                )
            return state[:dim]
        if len(state) < dim:
            if self.role == "robot":
                self.logger.warning(
                    f"The state dimension {len(state)} of {self.abbr} is smaller than the desired dimension {dim}, zero padding is added"
                )
            return state + [0] * (dim - len(state))
        return state

    def plot(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        **kwargs: Any,
    ) -> None:
        """Plot this object through its dedicated renderer."""
        self._object_plot.plot(ax, state, vertices, **kwargs)

    def _init_plot(self, ax: Any, **kwargs: Any) -> list[str]:
        """Initialize this object's Matplotlib artists."""
        return self._object_plot.init(ax, **kwargs)

    def _plot(
        self,
        ax: Any,
        state: np.ndarray,
        vertices: np.ndarray,
        initial: bool = False,
        **kwargs: Any,
    ) -> list[str]:
        """Create this object's configured Matplotlib artists."""
        return self._object_plot._plot(ax, state, vertices, initial=initial, **kwargs)

    def _step_plot(self, **kwargs: Any) -> None:
        """Update this object's Matplotlib artists."""
        self._object_plot.step(**kwargs)

    def plot_object(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        **kwargs: Any,
    ) -> None:
        """Draw this object's geometry or description image."""
        self._object_plot.plot_object(ax, state, vertices, **kwargs)

    def plot_object_image(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        description: str | None = None,
        **kwargs: Any,
    ) -> None:
        """Draw this object's configured description image."""
        self._object_plot.plot_object_image(ax, state, vertices, description, **kwargs)

    def plot_trajectory(
        self,
        ax: Any,
        trajectory: list | None = None,
        keep_traj_length: int = 0,
        **kwargs: Any,
    ) -> None:
        """Draw this object's trajectory."""
        self._object_plot.plot_trajectory(ax, trajectory, keep_traj_length, **kwargs)

    def plot_goal(
        self,
        ax: Any,
        goal_state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        goal_color: str | None = None,
        goal_zorder: int | None = 1,
        goal_alpha: float | None = 0.5,
        **kwargs: Any,
    ) -> None:
        """Draw this object's goal."""
        self._object_plot.plot_goal(
            ax,
            goal_state,
            vertices,
            goal_color,
            goal_zorder,
            goal_alpha,
            **kwargs,
        )

    def plot_text(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        **kwargs: Any,
    ) -> None:
        """Draw this object's text labels."""
        self._object_plot.plot_text(ax, state, **kwargs)

    def plot_arrow(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        velocity: np.ndarray | None = None,
        arrow_theta: float | None = 0.0,
        arrow_length: float = 0.4,
        arrow_width: float = 0.6,
        arrow_color: str | None = None,
        arrow_zorder: int = 3,
        **kwargs: Any,
    ) -> None:
        """Draw this object's velocity arrow."""
        self._object_plot.plot_arrow(
            ax,
            state,
            velocity,
            arrow_theta,
            arrow_length,
            arrow_width,
            arrow_color,
            arrow_zorder,
            **kwargs,
        )

    def plot_trail(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        keep_trail_length: int = 0,
        **kwargs: Any,
    ) -> None:
        """Draw one historical outline for this object."""
        self._object_plot.plot_trail(ax, state, vertices, keep_trail_length, **kwargs)

    def plot_fov(self, ax: Any, **kwargs: Any) -> None:
        """Draw this object's field of view."""
        self._object_plot.plot_fov(ax, **kwargs)

    def plot_uncertainty(self, ax: Any, **kwargs: Any) -> None:
        """Draw this object's uncertainty visualization."""
        self._object_plot.plot_uncertainty(ax, **kwargs)

    def plot_clear(self, all: bool = False) -> None:
        """Clear this object's artists."""
        self._object_plot.clear(all=all)

    def done(self):
        """
        Check if the object has completed its task.

        Returns:
            bool: True if the task is done, False otherwise.
        """
        return bool(self.stop_flag or self.arrive_flag)

    def reset(self):
        """
        Reset the object to its initial state.
        """
        self._state = self._init_state.copy()
        self._goal = self._init_goal.copy() if self._init_goal is not None else None
        self._velocity = self._init_velocity.copy()

        self.collision_flag = False
        self.arrive_flag = False
        self.stop_flag = False
        self.trajectory = []
        self._invalidate_reactive_cache()

    def refresh(self):
        """
        Refresh state-derived attributes (geometry and sensors) without
        advancing the simulation. Used after ``reset`` so geometry/sensor
        readings reflect the current state without running a kinematic
        step (which would clobber ``_velocity`` and add noise drift).
        """
        self._geometry = self.gf.step(self.state)
        self._geometry_valid = shapely.is_valid(self._geometry)
        self.sensor_step()
        self._invalidate_reactive_cache()

    def _invalidate_reactive_cache(self) -> None:
        """Drop per-tick caches read by reactive behaviors (SFM/RVO).

        Must be called from every path that mutates ``_state``,
        ``_velocity``, or the transformed geometry: ``step()``,
        ``set_state``, ``set_velocity``, ``reset``, ``refresh``;
        otherwise the next reader gets stale cached data.
        """
        self._velocity_xy_cache = None
        self._rvo_neighbor_state_cache = None
        self._rvo_line_segments_cache = None

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
            self.vel_min, self.velocity - self.info.acce * self._world_param.step_time
        )
        max_vel = np.minimum(
            self.vel_max, self.velocity + self.info.acce * self._world_param.step_time
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
            self.velocity,
            self.radius,
            self.G,
            self.h,
            self.cone_type,
            self.convex_flag,
        )

    def get_init_Gh(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Get the initial generalized inequality matrices G and h for the convex object.

        Returns:
            tuple[np.ndarray, np.ndarray]: Tuple containing initial G matrix and h vector.
        """
        return self.gf.get_init_Gh()

    def get_Gh(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Get the generalized inequality matrices G and h for the convex object.

        Returns:
            tuple[np.ndarray, np.ndarray]: Tuple containing G matrix and h vector.
        """
        return self.gf.get_Gh(
            center=self.centroid, radius=self.radius, vertices=self.vertices
        )

    def get_desired_omni_vel(self, goal_threshold=0.1, normalized=False) -> np.ndarray:
        """
        Get the desired omnidirectional velocity of the object.

        Args:
            goal_threshold (float): Threshold for goal proximity.
            normalized (bool): Whether to normalize the velocity.
        """

        if self.goal is None:
            return np.zeros((2, 1))

        dis, radian = relative_position(self.state, self.goal)

        if dis > goal_threshold:
            vx = self.vel_max[0, 0] * cos(radian)
            vy = self.vel_max[1, 0] * sin(radian)
        else:
            vx = 0
            vy = 0

        if normalized:
            length = np.linalg.norm([vx, vy])

            if length > 1:
                vx = vx / length
                vy = vy / length

        return np.array([[vx], [vy]])

    @property
    def name(self) -> str:
        """
        Get the name of the object.

        Returns:
            str: The name of the object.
        """

        return self._name if self._name is not None else self.role + "_" + str(self.id)

    @property
    def group_name(self) -> str:
        """
        Get the group name of the object.

        Returns:
            str: The group name of the object.
        """

        return (
            self._group_name
            if self._group_name is not None
            else self.role + "_" + str(self.group)
        )

    @property
    def abbr(self) -> str:
        """
        Get the abbreviation of the object.

        Returns:
            str: The abbreviation of the object.
        """

        return self.role[0] + str(self.id)

    @property
    def goal_abbr(self) -> str:
        """
        Get the goal abbreviation of the object.

        Returns:
            str: The goal abbreviation of the object.
        """

        return "G" + "-" + self.role[0] + str(self.id)

    def _get_text(self) -> str:
        """Return custom text if set, otherwise the default abbreviation."""
        return self._custom_text if self._custom_text is not None else self.abbr

    def set_text(self, text: str | None) -> None:
        """
        Set custom display text for this object.

        The text will be shown on the next render when ``show_text`` is enabled.
        Pass ``None`` to reset back to the default abbreviation.

        Args:
            text: The text string to display, or ``None`` to reset.
        """
        self._custom_text = text
        if hasattr(self, "_text"):
            self._text.set_text(self._get_text())

    def _get_goal_text(self) -> str:
        """Return custom goal text if set, otherwise the default goal abbreviation."""
        return (
            self._custom_goal_text
            if self._custom_goal_text is not None
            else self.goal_abbr
        )

    def set_goal_text(self, text: str | None) -> None:
        """
        Set custom display text for this object's goal.

        The text will be shown on the next render when ``show_goal_text`` is enabled.
        Pass ``None`` to reset back to the default goal abbreviation.

        Args:
            text: The text string to display, or ``None`` to reset.
        """
        self._custom_goal_text = text
        if hasattr(self, "_goal_text"):
            self._goal_text.set_text(self._get_goal_text())

    @property
    def shape(self) -> str:
        """
        Get the shape name of the object.

        Returns:
            str: The shape name of the object.
        """

        return self.gf.name

    @property
    def z(self) -> float:
        """
        Get the z coordinate of the object. For 3D object, the z coordinate is the height of the object, for 2D object, the z coordinate is 0.

        Returns:
            float: The z coordinate of the object.
        """

        return self.state[2, 0] if self.state_dim >= 6 else 0

    @property
    def kinematics(self) -> str | None:
        """
        Get the kinematics name of the object.

        Returns:
            str: The kinematics name of the object.
        """

        return self.kf.name if self.kf is not None else None

    @property
    def geometry(self) -> BaseGeometry:
        """
        Get the geometry Instance of the object.

        Returns:
            shapely.geometry.base.BaseGeometry: The geometry of the object.
        """

        return self._geometry

    @property
    def centroid(self) -> np.ndarray:
        """
        Get the centroid of the object.

        Returns:
            np.ndarray: The centroid of the object.
        """

        return self._geometry.centroid.coords._coords.T

    @property
    def id(self) -> int:
        """
        Get the id of the object.

        Returns:
            int: The id of the object.
        """

        return self._id

    @property
    def state(self) -> np.ndarray:
        """
        Get the state of the object.

        Returns:
            np.ndarray: The state of the object.
        """

        return self._state

    @property
    def init_state(self) -> np.ndarray:
        """
        Get the initial state of the object.

        Returns:
            np.ndarray: The initial state of the object.
        """

        return self._init_state

    @property
    def velocity(self) -> np.ndarray:
        """
        Get the velocity of the object.

        Returns:
            np.ndarray: The velocity of the object.
        """

        return self._velocity

    @property
    def goal(self) -> np.ndarray | None:
        """
        Get the goal of the object.

        Returns:
            np.ndarray: The goal of the object.
        """

        if self._goal is None:
            return None
        return np.c_[self._goal[0]]

    @property
    def goal_vertices(self) -> np.ndarray:
        """
        Get the goal vertices of the object.

        Returns:
            np.ndarray: The goal vertices of the object.
        """

        return self._goal_vertices

    @property
    def position(self) -> np.ndarray:
        """
        Get the position of the object.

        Returns:
            np.ndarray: The position of the object .
        """

        return self._state[:2]

    @property
    def radius(self) -> float:
        """
        Get the radius of the object.

        Returns:
            float: The radius of the object.
        """

        return self.gf.radius

    @property
    def length(self) -> float:
        """
        Get the length of the object.

        Returns:
            float: The length of the object.
        """

        return self.gf.length

    @property
    def width(self) -> float:
        """
        Get the width of the object.

        Returns:
            float: The width of the object.
        """

        return self.gf.width

    @property
    def wheelbase(self) -> float:
        """
        Get the wheelbase of the object.

        Returns:
            float: The wheelbase of the object.
        """

        return self.gf.wheelbase

    @property
    def radius_extend(self) -> float:
        """
        Get the radius of the object with a buffer.

        Returns:
            float: The radius of the object with a buffer.
        """

        return self.radius + 0.1

    @property
    def arrive(self) -> bool:
        """
        Get the arrive flag of the object.

        Returns:
            bool: The arrive flag of the object.
        """

        return self.arrive_flag

    @property
    def collision(self) -> bool:
        """
        Get the collision flag of the object.

        Returns:
            bool: The collision flag of the object.
        """

        return self.collision_flag

    @property
    def vertices(self) -> np.ndarray:
        """
        Get the vertices of the object.

        Returns:
            np.ndarray: The vertices of the object.
        """

        return self.gf.vertices

    @property
    def original_vertices(self) -> np.ndarray:
        """
        Get the original vertices of the object.

        Returns:
            np.ndarray: The original vertices of the object before any transformations.
        """
        return self.gf.original_vertices

    @property
    def original_geometry(self) -> BaseGeometry:
        """
        Get the original geometry of the object.

        Returns:
            shapely.geometry.base.BaseGeometry: The original geometry of the object.
        """
        return self.gf._original_geometry

    @property
    def original_centroid(self) -> np.ndarray:
        """
        Get the center of the object.

        Returns:
            np.ndarray: The center of the object.
        """
        return self.gf.original_centroid

    @property
    def original_state(self) -> np.ndarray:
        """
        Get the original state of the object from the original centroid.

        Returns:
            np.ndarray (3,1): The original state of the object.
        """
        return np.vstack((self.original_centroid, 0))

    @property
    def external_objects(self):
        """
        Get the environment objects that are not the self object.

        Returns:
            list: The environment objects that are not the self object.
        """
        return [obj for obj in self._env_param.objects if self.id != obj.id]

    @property
    def ego_object(self):
        """
        Get the ego object (this object itself).

        Returns:
            ObjectBase: The ego object (this object).
        """
        return self

    @property
    def possible_collision_objects(self):
        """
        Get the possible collision objects of the object from the geometry tree.

        Returns:
            list: The possible collision objects that could collide with this object.
        """
        tree = self._env_param.GeometryTree
        possible = []

        if tree is None:
            return possible

        candidates_index = tree.query(self.geometry)

        for index in candidates_index:
            obj = self._env_param.objects[index]

            if obj.unobstructed or obj.id == self.id:
                continue
            possible.append(obj)

        return possible

    @property
    def desired_omni_vel(self, goal_threshold=0.1):
        """
        Calculate the desired omnidirectional velocity.

        Args:
            goal_threshold (float): Threshold for goal proximity.

        Returns:
            np.ndarray: Desired velocity [vx, vy].
        """

        if self.goal is None:
            return np.zeros((2, 1))

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
            obj.rvo_neighbor_state
            for obj in self.external_objects
            if not obj.unobstructed
        ]

    @property
    def rvo_neighbor_state(self):
        """
        Get the RVO state for this object.

        Returns:
            list: State [x, y, vx, vy, radius], with (x, y) at the geometry
            centroid so the disc covers the collision geometry.
        """
        cached = getattr(self, "_rvo_neighbor_state_cache", None)
        if cached is not None:
            return cached
        vxy = self.velocity_xy
        centroid = self.centroid
        out = [
            centroid[0, 0],
            centroid[1, 0],
            vxy[0, 0],
            vxy[1, 0],
            self.radius_extend,
        ]
        self._rvo_neighbor_state_cache = out
        return out

    @property
    def rvo_line_segments(self) -> list[list[float]]:
        """
        Get line segments for RVO line obstacle avoidance.

        Returns:
            list: List of line segments [[x1, y1, x2, y2], ...] for linestring objects,
                  empty list for other shapes.
        """
        # Per-tick cache: invalidated in ``step()`` so a moving linestring
        # rebuilds its segments from the freshly transformed vertices on
        # the next read. Static objects never enter ``step()``, so their
        # cached segments persist for the lifetime of the env.
        cached = getattr(self, "_rvo_line_segments_cache", None)
        if cached is not None:
            return cached
        if self.shape != "linestring":
            self._rvo_line_segments_cache = []
            return self._rvo_line_segments_cache
        verts = self.vertices  # 2xN array
        segments = [
            [verts[0, i], verts[1, i], verts[0, i + 1], verts[1, i + 1]]
            for i in range(verts.shape[1] - 1)
        ]
        self._rvo_line_segments_cache = segments
        return segments

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
        cached = getattr(self, "_velocity_xy_cache", None)
        if cached is not None:
            return cached
        if self.kf is not None:
            out = self.kf.velocity_to_xy(self.state, self.velocity)
        else:
            out = np.zeros((2, 1))
        self._velocity_xy_cache = out
        return out

    @property
    def max_speed(self):
        """
        Get the maximum speed of the object.

        Returns:
            float: The maximum speed of the object.
        """
        if self.kf is not None:
            return self.kf.compute_max_speed(self.vel_max)
        return 0

    @property
    def beh_config(self) -> dict[str, Any]:
        """
        Get the behavior configuration for this object with group fallback.

        Returns:
            dict: The per-object behavior configuration if defined and non-empty;
                otherwise, the group's shared behavior configuration.
        """
        return self.obj_behavior.behavior_dict or self.group_behavior_dict

    @property
    def _world_param(self):
        """
        Access world_param via env instance if available, otherwise fallback to global.

        Returns:
            WorldParam: The world param instance.
        """
        if self._env is not None:
            return self._env._world_param
        from irsim.config import world_param

        return world_param

    @property
    def _env_param(self):
        """
        Access env_param via env instance if available, otherwise fallback to global.

        Returns:
            EnvParam: The env param instance.
        """
        if self._env is not None:
            return self._env._env_param
        from irsim.config import env_param

        return env_param

    @property
    def world_param(self):
        """
        Get the world parameters.

        Returns:
            WorldParam: World parameters including time, control_mode,
                collision_mode, step_time, and count.
        """
        return self._world_param

    @property
    def env_param(self):
        """
        Get the environment parameters.

        Returns:
            EnvParam: Environment parameters including logger and objects.
        """
        return self._env_param

    @property
    def logger(self):
        """
        Get the logger of the env_param.

        Returns:
            Logger: The logger associated in the env_param.
        """

        return self._env_param.logger

    @property
    def heading(self):
        """
        Get the heading of the object.

        Returns:
            float: The heading of the object.
        """
        if self.kf is not None:
            return self.kf.compute_heading(self.state, self.velocity)
        if self.state.shape[0] > 2:
            return self.state[2, 0]
        return 0.0

    @property
    def orientation(self):
        """
        Get the orientation of the object.

        Returns:
            float: The orientation angle of the object in radians.
        """
        return self.state[2, 0]
