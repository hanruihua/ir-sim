import shapely
import logging
import itertools
import numpy as np
import matplotlib as mpl
from dataclasses import dataclass
from irsim.lib import Behavior
from math import inf, pi, atan2, cos, sin, sqrt
from irsim.global_param import world_param, env_param
from irsim.lib import KinematicsFactory, GeometryFactory
from irsim.world import SensorFactory
from irsim.env.env_plot import linewidth_from_data_units
from irsim.global_param.path_param import path_manager
import matplotlib.transforms as mtransforms
from matplotlib import image
from typing import Optional

from irsim.util.util import (
    WrapToRegion,
    relative_position,
    gen_inequal_from_vertex,
    diff_to_omni,
    random_point_range,
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
    Base class representing a generic object in the robot simulator.

    This class encapsulates common attributes and behaviors for all objects,
    including robots and obstacles, managing their state, velocity, goals, 
    and kinematics.

    Attributes:
        state_dim (int): Dimension of the state vector.
        state_shape (tuple): Shape of the state array.
        vel_dim (int): Dimension of the velocity vector.
        vel_shape (tuple): Shape of the velocity array.
        _state (np.ndarray): Current state of the object.
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
    """

    id_iter = itertools.count()
    vel_shape = (2, 1)
    state_shape = (3, 1)

    def __init__(
        self,
        shape=None,
        kinematics=None,
        state=[0, 0, 0],
        velocity=[0, 0],
        goal=[10, 10, 0],
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
        arrive_mode="position",
        description=None,
        group=0,
        state_dim=None,
        vel_dim=None,
        unobstructed=False,
        **kwargs,
    ) -> None:
        
        """
        Initialize an ObjectBase instance.

        Args:
            shape (dict): Parameters defining the shape of the object for geometry creation.
            kinematics (dict, optional): Parameters defining the kinematics of the object.
            state (list of float, optional): Initial state vector [x, y, theta]. Defaults to [0, 0, 0].
            velocity (list of float, optional): Initial velocity vector [vx, vy]. Defaults to [0, 0].
            goal (list of float, optional): Goal state vector [x, y, theta]. Defaults to [10, 10, 0].
            role (str, optional): Role of the object, e.g., "robot" or "obstacle". Defaults to "obstacle".
            color (str, optional): Color of the object. Defaults to "k" (black).
            static (bool, optional): Indicates if the object is static. Defaults to False.
            vel_min (list of float, optional): Minimum velocity limits. Defaults to [-1, -1].
            vel_max (list of float, optional): Maximum velocity limits. Defaults to [1, 1].
            acce (list of float, optional): Acceleration limits. Defaults to [inf, inf].
            angle_range (list of float, optional): Allowed range of angles [min, max]. Defaults to [-pi, pi].
            behavior (str, optional): Behavioral mode of the object. Defaults to None.
            goal_threshold (float, optional): Threshold to determine if the goal is reached. Defaults to 0.1.
            sensors (list, optional): List of sensors attached to the object. Defaults to None.
            arrive_mode (str, optional): Mode for arrival detection, e.g., "position". Defaults to "position".
            description (str, optional): Description of the object. Defaults to None.
            group (int, optional): Group identifier for organizational purposes. Defaults to 0.
            state_dim (int, optional): Dimension of the state vector. If None, inferred from `state_shape`. Defaults to None.
            vel_dim (int, optional): Dimension of the velocity vector. If None, inferred from `vel_shape`. Defaults to None.
            unobstructed (bool, optional): Indicates if the object has an unobstructed path. Defaults to False.
            **kwargs: Additional keyword arguments for extended functionality.
        
        Raises:
            ValueError: If dimension parameters do not match the provided shapes.
        """

        self._id = next(ObjectBase.id_iter)

        # handlers
        self.gf = (
            GeometryFactory.create_geometry(**shape) if shape is not None else None
        )
        self.kf = (
            KinematicsFactory.create_kinematics(wheelbase=self.wheelbase, **kinematics)
            if kinematics is not None
            else None
        )

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

        self._goal = np.c_[goal]
        self._init_goal = np.c_[goal]

        self._geometry = self.gf.step(self.state)
        self.group = group

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
        self.rl = self.beh_config.get("range_low", [0, 0, -pi])
        self.rh = self.beh_config.get("range_high", [10, 10, pi])
        self.wander = self.beh_config.get("wander", False)

        if self.wander:
            self._goal = random_point_range(self.rl, self.rh)

        # plot
        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

        self.plot_kwargs = kwargs.get("plot", dict())

        self.collision_obj = []

    def __eq__(self, o: object) -> bool:
        return self._id == o._id

    def __hash__(self) -> int:
        return self._id

    def __str__(self) -> str:
        return f"ObjectBase: {self._id}"

    @classmethod
    def reset_id_iter(cls, start=0, step=1):
        """reset the id iterator"""
        cls.id_iter = itertools.count(start, step)

    def step(self, velocity=None, **kwargs):
        """
        Perform a simulation step, updating the object's state.

        Args:
            velocity (np.ndarray, optional): Desired velocity for the step.
            **kwargs: Additional parameters.

        Returns:
            np.ndarray: The new state of the object.
        """

        if self.static or self.stop_flag or self.kf is None:
            self._velocity = np.zeros_like(velocity)
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
        [sensor.step(self._state[0:3]) for sensor in self.sensors]

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
                    self._goal = random_point_range(self.rl, self.rh)
                    self.arrive_flag = False

                behavior_vel = self.obj_behavior.gen_vel(env_param.objects)

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

        if state.shape[0] < self.state_dim:
            state = np.r_[
                state,
                np.zeros((self.state_dim - state.shape[0], state.shape[1])),
            ]
        elif state.shape[0] > self.state_dim:
            state = state[: self.state_dim]

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
        self._geometry = self.gf.step(self.state)

    def set_init_geometry(self, geometry):
        """
        Set the initial geometry of the object.

        Args:
            geometry: The shapely geometry of the object.
        """
        self._init_geometry = geometry

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

            elif self.shape == "polygon" or self.shape == "rectangle":
                object_patch = mpl.patches.Polygon(xy=self.vertices.T, color=self.color)
                object_patch.set_zorder(3)
                ax.add_patch(object_patch)

            elif self.shape == "linestring":
                object_patch = mpl.lines.Line2D(
                    self.vertices[0, :], self.vertices[1, :], color=self.color
                )
                object_patch.set_zorder(3)
                ax.add_line(object_patch)

            elif self.shape == "map":
                return

            self.plot_patch_list.append(object_patch)

        else:
            self.plot_object_image(ax, self.description, **kwargs)

    def plot_object_image(self, ax, description, **kwargs):

        # x = self.vertices[0, 0]
        # y = self.vertices[1, 0]

        start_x = self.vertices[0, 0]
        start_y = self.vertices[1, 0]
        r_phi = self._state[2, 0]
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
        return self.gf.name

    @property
    def kinematics(self):
        return self.kf.name if self.kf is not None else None

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
        return self.gf.radius

    @property
    def length(self):
        return self.gf.length

    @property
    def width(self):
        return self.gf.width

    @property
    def wheelbase(self):
        return self.gf.wheelbase

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
        return self.gf.vertices

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
