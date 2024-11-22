import shapely
import logging
import itertools
import numpy as np
import matplotlib as mpl

from dataclasses import dataclass
from shapely.ops import transform
from irsim.lib.behavior import Behavior
from math import inf, pi, atan2, cos, sin, sqrt
from irsim.global_param import world_param, env_param
from irsim.util.util import (
    WrapToRegion,
    get_transform,
    relative_position,
    WrapToPi,
    gen_inequal_from_vertex,
    diff_to_omni,
    random_point_range
)
from irsim.lib.generation import random_generate_polygon
from irsim.world.sensors.sensor_factory import SensorFactory
from shapely import Point, Polygon, LineString, minimum_bounding_radius, MultiPoint
from irsim.lib import kinematics_factory
from irsim.env.env_plot import linewidth_from_data_units

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
    G: np.ndarray
    h: np.ndarray
    cone_type: str
    wheelbase: float

    def add_property(self, key, value):
        setattr(self, key, value)


@dataclass
class ObstacleInfo:
    center: np.ndarray
    vertex: np.ndarray
    velocity: np.ndarray
    cone_type: str
    radius: float

    def add_property(self, key, value):
        setattr(self, key, value)


class ObjectBase:
    """
    Represents a base class for objects in a simulation environment.

    This class provides a template for defining the properties and behaviors of various objects such as robots,
    obstacles, or landmarks within a simulation world. Each instance of this class or its derivatives represents
    a distinct object with specific attributes and kinematics.

    Attributes:
        id_iter (iterator): A class-level iterator to generate unique IDs for each object.
        vel_shape (tuple): The shape of the velocity vector, default is (2, 1).
    """

    id_iter = itertools.count()
    vel_shape = (2, 1)
    state_shape = (3, 1)

    def __init__(
        self,
        shape: str = "circle",
        shape_tuple=None,
        state=[0, 0, 0],
        velocity=[0, 0],
        goal=[10, 10, 0],
        kinematics: str = "omni",
        role: str = "obstacle",
        color="k",
        static=False,
        vel_min=[-1, -1],
        vel_max=[1, 1],
        acce=[inf, inf],
        angle_range=[-pi, pi],
        behavior=None,
        goal_threshold=0.1,
        sensors=None,
        kinematics_dict=dict(),
        arrive_mode="position",
        description=None,
        group=0,
        reso=0.1,
        state_dim=None,
        vel_dim=None,
        unobstructed=False,
        **kwargs,
    ) -> None:
        """
        Initialize an ObjectBase instance.

        Args:
            shape (str): The shape of the object, e.g., circle, polygon, etc.
            shape_tuple: Tuple to initialize the geometry.
            state (list or np.ndarray): The state of the object [x, y, theta].
            velocity (list or np.ndarray): The velocity of the object [vx, vy].
            goal (list or np.ndarray): The goal state of the object [x, y, theta].
            kinematics (str): The moving kinematics, e.g., omni, diff, etc.
            role (str): The role of the object, e.g., robot, obstacle.
            color (str): The color of the object.
            static (bool): Whether the object is static.
            vel_min (list or np.ndarray): Minimum velocity limits.
            vel_max (list or np.ndarray): Maximum velocity limits.
            acce (list or np.ndarray): Acceleration limits.
            angle_range (list or np.ndarray): Range of angles.
            behavior (dict): Behavior parameters.
            goal_threshold (float): Threshold for reaching the goal.
            sensors (list): List of sensors.
            kinematics_dict (dict): Additional kinematics parameters.
            arrive_mode (str): Mode of arrival, position or state.
            description (str): Description of the object.
            group (int): Group identifier.
            reso (float): Resolution.
            state_dim (int): Dimension of the state.
            vel_dim (int): Dimension of the velocity.
            unobstructed (bool): Whether the object is unobstructed.
        """
        self._id = next(ObjectBase.id_iter)
        self._shape = shape
        self._init_geometry = self.construct_geometry(shape, shape_tuple, reso)

        if state_dim is None:
            self.state_dim = self.state_shape[0]
        else:
            self.state_dim = state_dim
            self.state_shape = (state_dim, 1)

        if vel_dim is None:
            self.vel_dim = self.vel_shape[0]
        else:
            self.vel_dim = vel_dim
            self.vel_shape = (vel_dim, 1)

        state = self.input_state_check(state, self.state_dim)
        self._state = np.c_[state]
        self._init_state = np.c_[state]

        self._velocity = np.c_[velocity]
        self._init_velocity = np.c_[velocity]

        self._goal = np.c_[goal]
        self._init_goal = np.c_[goal]

        self._geometry = self.geometry_transform(self._init_geometry, self._state)
        self.group = group

        self.kinematics = kinematics
        self.kinematics_dict = kinematics_dict

        # flag
        self.stop_flag = False
        self.arrive_flag = False
        self.collision_flag = False
        self.unobstructed = unobstructed

        # information
        self.static = static
        self.vel_min = np.c_[vel_min]
        self.vel_max = np.c_[vel_max]
        self.color = color
        self.role = role

        self.length = kwargs.get("length", self.radius * 2)
        self.width = kwargs.get("width", self.radius * 2)
        self.wheelbase = kwargs.get("wheelbase", None)

        self.info = ObjectInfo(
            self._id,
            shape,
            kinematics,
            role,
            color,
            static,
            np.c_[goal],
            np.c_[vel_min],
            np.c_[vel_max],
            np.c_[acce],
            np.c_[angle_range],
            goal_threshold,
            self.G,
            self.h,
            self.cone_type,
            self.wheelbase,
        )
        self.obstacle_info = None

        self.trajectory = []

        self.description = description

        # arrive judgement
        self.goal_threshold = goal_threshold
        self.arrive_mode = arrive_mode

        # sensor
        sf = SensorFactory()
        if sensors is not None:
            self.sensors = [
                sf.create_sensor(self._state[0:3], self._id, **sensor_kwargs)
                for sensor_kwargs in sensors
            ]

            self.lidar = [
                sensor for sensor in self.sensors if sensor.sensor_type == "lidar"
            ][0]
        else:
            self.sensors = []

        # behavior
        self.obj_behavior = Behavior(self.info, behavior)
        self.gl = self.beh_config.get("range_low", [0, 0, -pi])
        self.gh = self.beh_config.get("range_high", [10, 10, pi])
        self.wander = self.beh_config.get('wander', False)

        if self.wander: self._goal = random_point_range(self.gl, self.gh)
            
        # plot
        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

        self.plot_kwargs = kwargs.get("plot", dict())

        self.collision_obj = []

    def __repr__(self) -> str:
        pass

    def __eq__(self, o: object) -> bool:
        return self._id == o._id

    @classmethod
    def create_with_shape(cls, kinematics_name, shape_dict, **kwargs):
        """
        Create an object with a specific shape.

        Args:
            kinematics_name (str): Kinematics type, e.g., diff, omni.
            shape_dict (dict): Dictionary defining the shape.
            **kwargs: Additional parameters.

        Returns:
            ObjectBase: An instance of ObjectBase with the specified shape.
        """
        shape_name = shape_dict.get("name", "circle")

        if shape_name == "circle":
            radius = shape_dict.get("radius", 0.2)
            wheelbase = shape_dict.get("wheelbase", radius)

            return cls(
                shape="circle",
                shape_tuple=(0, 0, radius),
                wheelbase=wheelbase,
                **kwargs,
            )

        elif shape_name == "rectangle":

            if kinematics_name == "diff" or kinematics_name == "omni":
                length = shape_dict.get("length", 0.2)
                width = shape_dict.get("width", 0.1)

                return cls(
                    shape="polygon",
                    shape_tuple=[
                        (-length / 2, -width / 2),
                        (length / 2, -width / 2),
                        (length / 2, width / 2),
                        (-length / 2, width / 2),
                    ],
                    length=length,
                    width=width,
                    **kwargs,
                )

            elif kinematics_name == "acker":

                length = shape_dict.get("length", 4.6)
                width = shape_dict.get("width", 1.6)
                wheelbase = shape_dict.get("wheelbase", 3)

                start_x = -(length - wheelbase) / 2
                start_y = -width / 2

                vertices = [
                    (start_x, start_y),
                    (start_x + length, start_y),
                    (start_x + length, start_y + width),
                    (start_x, start_y + width),
                ]

                return cls(
                    shape="polygon",
                    shape_tuple=vertices,
                    wheelbase=wheelbase,
                    length=length,
                    width=width,
                    **kwargs,
                )

            else:
                length = shape_dict.get("length", 0.2)
                width = shape_dict.get("width", 0.1)

                return cls(
                    shape="polygon",
                    shape_tuple=[
                        (-length / 2, -width / 2),
                        (length / 2, -width / 2),
                        (length / 2, width / 2),
                        (-length / 2, width / 2),
                    ],
                    **kwargs,
                )

        elif shape_name == "polygon":

            if shape_dict.get("random_shape", False):
                vertices = random_generate_polygon(**shape_dict)
            else:
                vertices = shape_dict.get("vertices", None)

            if vertices is None:
                raise ValueError("vertices are not set")

            return cls(shape="polygon", shape_tuple=vertices, **kwargs)

        elif shape_name == "linestring":

            vertices = shape_dict.get("vertices", None)

            if vertices is None:
                raise ValueError("vertices should not be None")

            return cls(shape="linestring", shape_tuple=vertices, **kwargs)

        elif shape_name == "points":
            pass

        else:
            raise NotImplementedError(f"shape {shape_name} not implemented")

    def step(self, velocity=None, **kwargs):
        """
        Perform a simulation step, updating the object's state.

        Args:
            velocity (np.ndarray, optional): Desired velocity for the step.
            **kwargs: Additional parameters.

        Returns:
            np.ndarray: The new state of the object.
        """
        if self.static or self.stop_flag:
            self._velocity = np.zeros_like(velocity)
            return self._state
        else:
            self.pre_process()
            behavior_vel = self.gen_behavior_vel(velocity)
            new_state = self._kinematics_step(behavior_vel, **self.kinematics_dict)
            next_state = self.mid_process(new_state)
            self._state = next_state
            self._velocity = behavior_vel
            self._geometry = self.geometry_transform(self._init_geometry, self._state)
            self.sensor_step()
            self.post_process()
            self.check_status()
            self.trajectory.append(self._state.copy())
            return next_state

    def sensor_step(self):
        """
        Update all sensors for the current state.
        """
        [sensor.step(self._state[0:3]) for sensor in self.sensors]

    def _kinematics_step(
        self, velocity, noise=False, alpha=[0.03, 0, 0, 0.03], mode="steer", **kwargs
    ):
        """
        Calculate the next state using the kinematics model.

        Args:
            velocity (np.ndarray): Velocity vector [vx, vy] (2x1).
            noise (bool): Whether to add noise (default False).
            alpha (list): Noise parameters.
            mode (str): Mode for kinematics, e.g., "steer".

        Returns:
            np.ndarray: Next state [x, y, theta] (3x1).
        """
        assert (
            velocity.shape == self.vel_shape and self._state.shape == self.state_shape
        )

        if self.kinematics == "omni" or self.kinematics == "diff":
            next_state = kinematics_factory[self.kinematics](
                self._state, velocity, world_param.step_time, noise, alpha
            )

        elif self.kinematics == "acker":
            next_state = kinematics_factory[self.kinematics](
                self._state,
                velocity,
                world_param.step_time,
                noise,
                alpha,
                mode=mode,
                wheelbase=self.wheelbase,
            )

        elif self.kinematics == "custom":
            raise NotImplementedError("custom kinematics is not implemented")

        else:
            raise ValueError(
                "kinematics should be one of the following: omni, diff, acker"
            )

        if next_state.shape[0] < self.state_dim:
            next_state = np.r_[
                next_state,
                np.zeros((self.state_dim - next_state.shape[0], next_state.shape[1])),
            ]
        elif next_state.shape[0] > self.state_dim:
            next_state = next_state[: self.state_dim]

        return next_state

    def geometry_transform(self, geometry, state):
        """
        Transform geometry to the new state.

        Args:
            geometry: The geometry to transform.
            state (np.ndarray): State vector [x, y, theta].

        Returns:
            Transformed geometry.
        """

        def transform_with_state(x, y):
            trans, rot = get_transform(state)
            points = np.array([x, y])
            new_points = rot @ points + trans
            return (new_points[0, :], new_points[1, :])

        new_geometry = transform(transform_with_state, geometry)
        return new_geometry

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

    def check_arrive_status(self):
        """
        Check if the object has arrived at the goal.
        """
        if self.arrive_mode == "state":
            diff = np.linalg.norm(self._state[:3] - self._goal[:3])
        elif self.arrive_mode == "position":
            diff = np.linalg.norm(self._state[:2] - self._goal[:2])

        if diff < self.goal_threshold:
            self.arrive_flag = True
        else:
            self.arrive_flag = False

    def check_collision_status(self):
        """
        Check if the object is in collision with others.
        """
        collision_flags = [
            self.check_collision(obj) for obj in env_param.objects if self.id != obj.id
        ]

        self.collision_obj = []

        for obj in env_param.objects:
            if self.id != obj.id:
                if self.check_collision(obj):
                    self.collision_obj.append(obj)
                    if self.role == "robot":
                        if not self.collision_flag:
                            env_param.logger.warning(
                                self.role
                                + "{} is collided with {} at state {}".format(
                                    self.id,
                                    obj.id,
                                    list(np.round(self._state[:2, 0], 2)),
                                )
                            )

        self.collision_flag = any(collision_flags)

    def check_collision(self, obj):
        """
        Check collision with another object.

        Args:
            obj (ObjectBase): Another object.

        Returns:
            bool: True if collision occurs, False otherwise.
        """
        return shapely.intersects(self._geometry, obj._geometry)

    def gen_behavior_vel(self, velocity):
        """
        Generate velocity based on behavior.

        Args:
            velocity (np.ndarray): Desired velocity.

        Returns:
            np.ndarray: Behavior-based velocity.
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
                    self._goal = random_point_range(self.gl, self.gh)
                    self.arrive_flag = False

                behavior_vel = self.obj_behavior.gen_vel(env_param.objects)

                # if self.beh_config["name"] == "rvo":

                #     behavior_vel = self.obj_behavior.gen_vel(
                #         self._state,
                #         self._goal,
                #         min_vel,
                #         max_vel,
                #         rvo_neighbor=self.rvo_neighbors,
                #         rvo_state=self.rvo_state,
                #     )

                # else:
                #     behavior_vel = self.obj_behavior.gen_vel(
                #         self._state, self._goal, min_vel, max_vel
                #     )

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

    def vel_check(self, velocity):
        """
        Check if velocity is within limits.

        Args:
            velocity (np.ndarray): Desired velocity.

        Returns:
            np.ndarray: Clipped velocity.
        """
        if isinstance(velocity, list):
            velocity = np.c_[velocity]
        if velocity.ndim == 1:
            velocity = velocity[:, np.newaxis]

        assert velocity.shape == self.vel_shape

        min_vel = np.maximum(
            self.vel_min, self._velocity - self.acce * world_param.step_time
        )
        max_vel = np.minimum(
            self.vel_max, self._velocity + self.acce * world_param.step_time
        )

        if (velocity < min_vel).any():
            logging.warning(
                "velocity is smaller than min_vel, velocity is clipped to min_vel"
            )
        elif (velocity > max_vel).any():
            logging.warning(
                "velocity is larger than max_vel, velocity is clipped to max_vel"
            )

        velocity_clip = np.clip(velocity, min_vel, max_vel)

        return velocity_clip

    def pre_process(self):
        pass

    def post_process(self):
        pass

    def mid_process(self, state):
        """
        Process state in the middle of a step.

        Args:
            state (np.ndarray): State vector.

        Returns:
            np.ndarray: Processed state.
        """
        if state.shape[0] > 2:
            state[2, 0] = WrapToRegion(state[2, 0], self.info.angle_range)

        return state

    def get_lidar_scan(self):
        return self.lidar.get_scan()

    def get_lidar_points(self):
        return self.lidar.get_points()

    def get_lidar_offset(self):
        return self.lidar.get_offset()

    def set_state(self, state=[0, 0, 0], init=False):
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

        assert self._state.shape == temp_state.shape

        if init:
            self._init_state = temp_state.copy()

        self._state = temp_state.copy()
        self._geometry = self.geometry_transform(self._init_geometry, self._state)

    def set_init_geometry(self, geometry):
        """
        Set the initial geometry of the object.

        Args:
            geometry: The shapely geometry of the object.
        """
        self._init_geometry = geometry

    def construct_geometry(self, shape, shape_tuple, reso=0.1):
        """
        Construct the geometry of the object.

        Args:
            shape (str): The shape of the object.
            shape_tuple: Tuple to initialize the geometry.
            reso (float): The resolution of the object.

        Returns:
            Geometry of the object.
        """
        if shape == "circle":
            geometry = Point([shape_tuple[0], shape_tuple[1]]).buffer(shape_tuple[2])

        elif shape == "polygon" or shape == "rectangle":
            geometry = Polygon(shape_tuple)

        elif shape == "linestring":
            geometry = LineString(shape_tuple)

        elif shape == "points":
            geometry = MultiPoint(shape_tuple.T).buffer(reso / 2)

        else:
            raise ValueError(
                "shape should be one of the following: circle, polygon, linestring, points"
            )

        if shape == "polygon" or shape == "rectangle" or shape == "circle":
            self.G, self.h, self.cone_type = self.generate_Gh(shape, shape_tuple)
        else:
            self.G, self.h, self.cone_type = None, None, "Rpositive"

        return geometry

    def generate_Gh(self, shape, shape_tuple):
        """
        Generate G and h for convex object.

        Args:
            shape (str): The shape of the object.
            shape_tuple: Tuple to initialize the geometry.

        Returns:
            tuple: G matrix, h vector, and cone type.
        """
        if shape == "circle":
            radius = shape_tuple[2]
            G = np.array([[1, 0], [0, 1], [0, 0]])
            h = np.array([[0], [0], [-radius]])
            cone_type = "norm2"

        else:
            init_vertex = np.array(shape_tuple).T
            G, h = gen_inequal_from_vertex(init_vertex)
            cone_type = "Rpositive"

        return G, h, cone_type

    def geometry_state_transition(self):
        pass

    def input_state_check(self, state, dim=3):
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
            **kwargs: Additional plotting options.
        """
        self.state_re = self._state
        self.goal_re = self._goal

        self.plot_kwargs.update(kwargs)

        show_goal = self.plot_kwargs.get("show_goal", False)
        show_text = self.plot_kwargs.get("show_text", False)
        show_arrow = self.plot_kwargs.get("show_arrow", False)
        show_uncertainty = self.plot_kwargs.get("show_uncertainty", False)
        show_trajectory = self.plot_kwargs.get("show_trajectory", False)
        show_trail = self.plot_kwargs.get("show_trail", False)
        show_sensor = self.plot_kwargs.get("show_sensor", True)
        trail_freq = self.plot_kwargs.get("trail_freq", 2)
        goal_color = self.plot_kwargs.get("goal_color", self.color)

        self.plot_object(ax, **kwargs)

        if show_goal:
            self.plot_goal(ax, goal_color)

        if show_text:
            self.plot_text(ax)

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

    def plot_object(self, ax, **kwargs):
        """
        Plot the object itself.

        Args:
            ax: Matplotlib axis.
            **kwargs: Additional plotting options.
        """
        if self.description is None:
            x = self.state_re[0, 0]
            y = self.state_re[1, 0]

            if self.shape == "circle":
                object_patch = mpl.patches.Circle(
                    xy=(x, y), radius=self.radius, color=self.color
                )
                object_patch.set_zorder(3)
                ax.add_patch(object_patch)

            elif self.shape == "polygon":
                object_patch = mpl.patches.Polygon(xy=self.vertices.T, color=self.color)
                object_patch.set_zorder(3)
                ax.add_patch(object_patch)

            elif self.shape == "linestring":
                object_patch = mpl.lines.Line2D(
                    self.vertices[0, :], self.vertices[1, :], color=self.color
                )
                object_patch.set_zorder(3)
                ax.add_line(object_patch)

            elif self.shape == "points":
                return

            self.plot_patch_list.append(object_patch)

        else:
            self.plot_object_image(ax, self.description, **kwargs)

    def plot_object_image(self):
        pass

    def plot_trajectory(self, ax, keep_length=0, **kwargs):
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
        solid_capstyle = "round" if self._shape == "circle" else "butt"

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

    def plot_goal(self, ax, goal_color="r"):
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
        goal_circle.set_zorder(1)

        ax.add_patch(goal_circle)

        self.plot_patch_list.append(goal_circle)

    def plot_text(self, ax, **kwargs):
        pass

    def plot_arrow(self, ax, arrow_length=0.4, arrow_width=0.6, **kwargs):
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
        theta = atan2(self.velocity_xy[1, 0], self.velocity_xy[0, 0])
        arrow_color = kwargs.get("arrow_color", "gold")

        arrow = mpl.patches.Arrow(
            x,
            y,
            arrow_length * cos(theta),
            arrow_length * sin(theta),
            width=arrow_width,
            color=arrow_color,
        )
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
        trail_edgecolor = kwargs.get("edgecolor", self.color)
        trail_linewidth = kwargs.get("linewidth", 0.8)
        trail_alpha = kwargs.get("trail_alpha", 0.7)
        trail_fill = kwargs.get("trail_fill", False)
        trail_color = kwargs.get("trail_color", self.color)

        r_phi_ang = 180 * self._state[2, 0] / pi

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
            ax.add_patch(car_circle)

    def plot_uncertainty(self, ax, **kwargs):
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

    def get_vel_range(self):
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

    def get_info(self):
        """
        Get object information.

        Returns:
            ObjectInfo: Information about the object.
        """
        return self.info

    def get_obstacle_info(self):
        """
        Get information about the object as an obstacle.

        Returns:
            ObstacleInfo: Obstacle-related information.
        """
        return ObstacleInfo(
            self._state[:2, :],
            self.vertices[:, :-1],
            self._velocity,
            self.info.cone_type,
            self.radius,
        )

    def get_Gh(self):
        """
        Get the G and h matrices for the object's geometry.

        Returns:
            tuple: G matrix and h vector.
        """
        return self.info.G, self.info.h

    @property
    def name(self):
        return self.info.role + "_" + str(self.id)

    @property
    def shape(self):
        return self._shape

    @property
    def geometry(self):
        return self._geometry

    @property
    def centroid(self):
        return self._geometry.centroid.coords._coords.T

    @property
    def id(self):
        return self._id

    @property
    def state(self):
        return self._state

    @property
    def goal(self):
        return self._goal

    @property
    def position(self):
        return self._state[:2]

    @property
    def radius(self):
        return minimum_bounding_radius(self._geometry)

    @property
    def radius_extend(self):
        return self.radius + 0.1

    @property
    def arrive(self):
        return self.arrive_flag

    @property
    def collision(self):
        return self.collision_flag

    @property
    def ineq_Ab(self):
        return self.get_inequality_Ab()

    @property
    def vertices(self):
        if self.shape == "linestring":
            x = self._geometry.xy[0]
            y = self._geometry.xy[1]
            return np.c_[x, y].T
        return self._geometry.exterior.coords._coords.T

    @property
    def desired_diff_vel(self, angle_tolerance=0.1, goal_threshold=0.1):
        """
        Calculate the desired differential velocity.

        Args:
            angle_tolerance (float): Tolerance for angle deviation.
            goal_threshold (float): Threshold for goal proximity.

        Returns:
            np.ndarray: Desired velocity [linear, angular].
        """
        distance, radian = relative_position(self._state, self._goal)

        if distance < goal_threshold:
            return np.zeros((2, 1))

        diff_radian = WrapToPi(radian - self._state[2, 0])

        linear = self.vel_max[0, 0] * np.cos(diff_radian)

        if abs(diff_radian) < angle_tolerance:
            angular = 0
        else:
            angular = self.vel_max[1, 0] * np.sign(diff_radian)

        return np.array([[linear], [angular]])

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
            obj.rvo_neighbor_state for obj in env_param.objects if self.id != obj.id
        ]

    @property
    def rvo_neighbor_state(self):
        """
        Get the RVO state for this object.

        Returns:
            list: State [x, y, vx, vy, radius].
        """
        return [
            self._state[0, 0],
            self._state[1, 0],
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
            self._state[0, 0],
            self._state[1, 0],
            self.velocity_xy[0, 0],
            self.velocity_xy[1, 0],
            self.radius_extend,
            vx_des,
            vy_des,
            self._state[2, 0],
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
            return diff_to_omni(self._state[2, 0], self._velocity)
        else:
            raise ValueError("kinematics not implemented")

    @property
    def beh_config(self):
        # behavior config dictory
        return self.obj_behavior.behavior_dict