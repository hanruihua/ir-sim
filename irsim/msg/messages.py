"""ROS-style, dependency-free messages for simulation snapshots.

The classes in this module are plain dataclasses. They intentionally do not
depend on ROS, which keeps them usable in regular Python, learning, and batch
simulation workflows while providing familiar topic names and message shapes.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields, is_dataclass
from math import cos, sin, tan
from typing import Any, ClassVar

import numpy as np


def _copy_array(value: Any) -> np.ndarray | None:
    """Return an independent numpy copy, preserving ``None``."""
    if value is None:
        return None
    return np.array(value, copy=True)


def _float32_array(value: Any) -> np.ndarray:
    """Return an independent ROS ``float32[]``-compatible array."""
    if value is None:
        return np.array([], dtype=np.float32)
    return np.array(value, dtype=np.float32, copy=True)


def _to_builtin(value: Any) -> Any:
    """Recursively convert message data to JSON-compatible Python values."""
    if is_dataclass(value) and not isinstance(value, type):
        return {
            item.name: _to_builtin(getattr(value, item.name)) for item in fields(value)
        }
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: _to_builtin(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_builtin(item) for item in value]
    return value


class Message:
    """Base class that provides JSON-compatible message serialization."""

    __slots__ = ()

    def to_dict(self) -> dict[str, Any]:
        """Return the complete message as JSON-compatible built-in values."""
        return _to_builtin(self)


@dataclass(slots=True)
class Header(Message):
    """Metadata shared by all IR-SIM messages."""

    ros_type: ClassVar[str] = "std_msgs/Header"

    seq: int = 0
    stamp: float = 0.0
    frame_id: str = "world"


@dataclass(slots=True)
class Point(Message):
    """ROS ``geometry_msgs/Point``-shaped Cartesian position."""

    ros_type: ClassVar[str] = "geometry_msgs/Point"

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass(slots=True)
class Vector3(Message):
    """ROS ``geometry_msgs/Vector3``-shaped Cartesian vector."""

    ros_type: ClassVar[str] = "geometry_msgs/Vector3"

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass(slots=True)
class Quaternion(Message):
    """ROS ``geometry_msgs/Quaternion``-shaped orientation."""

    ros_type: ClassVar[str] = "geometry_msgs/Quaternion"

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    @classmethod
    def from_yaw(cls, yaw: float) -> Quaternion:
        """Create a planar quaternion from a yaw angle in radians."""
        half_yaw = float(yaw) / 2.0
        return cls(z=sin(half_yaw), w=cos(half_yaw))


@dataclass(slots=True)
class Pose(Message):
    """ROS ``geometry_msgs/Pose``-shaped position and orientation."""

    ros_type: ClassVar[str] = "geometry_msgs/Pose"

    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass(slots=True)
class PoseWithCovariance(Message):
    """ROS ``geometry_msgs/PoseWithCovariance``-shaped pose."""

    ros_type: ClassVar[str] = "geometry_msgs/PoseWithCovariance"

    pose: Pose = field(default_factory=Pose)
    covariance: np.ndarray = field(default_factory=lambda: np.zeros(36))


@dataclass(slots=True)
class Twist(Message):
    """ROS ``geometry_msgs/Twist``-shaped linear and angular velocity."""

    ros_type: ClassVar[str] = "geometry_msgs/Twist"

    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)


@dataclass(slots=True)
class TwistWithCovariance(Message):
    """ROS ``geometry_msgs/TwistWithCovariance``-shaped velocity."""

    ros_type: ClassVar[str] = "geometry_msgs/TwistWithCovariance"

    twist: Twist = field(default_factory=Twist)
    covariance: np.ndarray = field(default_factory=lambda: np.zeros(36))


@dataclass(slots=True)
class Odometry(Message):
    """ROS ``nav_msgs/Odometry``-shaped object pose and velocity."""

    ros_type: ClassVar[str] = "nav_msgs/Odometry"

    header: Header = field(default_factory=Header)
    child_frame_id: str = ""
    pose: PoseWithCovariance = field(default_factory=PoseWithCovariance)
    twist: TwistWithCovariance = field(default_factory=TwistWithCovariance)

    @classmethod
    def from_object(
        cls,
        obj: Any,
        *,
        stamp: float = 0.0,
        seq: int = 0,
        frame_id: str = "world",
    ) -> Odometry:
        """Capture an object's planar pose and body-frame velocity."""
        state = np.asarray(obj.state).reshape(-1)
        velocity = np.asarray(obj.velocity).reshape(-1)
        yaw = float(state[2]) if state.size > 2 else 0.0

        linear = Vector3()
        angular = Vector3()
        kinematics = obj.kinematics

        if velocity.size:
            linear.x = float(velocity[0])
        if kinematics in {"omni", "omni_angular"} and velocity.size > 1:
            linear.y = float(velocity[1])
        if kinematics == "diff" and velocity.size > 1:
            angular.z = float(velocity[1])
        elif kinematics == "omni_angular" and velocity.size > 2:
            angular.z = float(velocity[2])
        elif kinematics == "acker" and state.size > 3 and velocity.size:
            angular.z = float(velocity[0]) * tan(float(state[3])) / obj.wheelbase

        return cls(
            header=Header(seq=int(seq), stamp=float(stamp), frame_id=frame_id),
            child_frame_id=f"{obj.name}/base_link",
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=float(state[0]), y=float(state[1])),
                    orientation=Quaternion.from_yaw(yaw),
                )
            ),
            twist=TwistWithCovariance(twist=Twist(linear=linear, angular=angular)),
        )


@dataclass(slots=True)
class LaserScan(Message):
    """ROS Noetic ``sensor_msgs/LaserScan`` fields for a LiDAR snapshot."""

    ros_type: ClassVar[str] = "sensor_msgs/LaserScan"

    header: Header
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float
    ranges: np.ndarray
    intensities: np.ndarray

    @classmethod
    def from_sensor(
        cls,
        sensor: Any,
        *,
        stamp: float = 0.0,
        seq: int = 0,
        frame_id: str | None = None,
    ) -> LaserScan:
        """Capture a LiDAR sensor without sharing its mutable arrays."""
        scan = sensor.get_scan()
        return cls(
            header=Header(
                seq=int(seq),
                stamp=float(stamp),
                frame_id=frame_id or str(sensor.sensor_type),
            ),
            angle_min=float(scan["angle_min"]),
            angle_max=float(scan["angle_max"]),
            angle_increment=float(scan["angle_increment"]),
            time_increment=float(scan["time_increment"]),
            scan_time=float(scan["scan_time"]),
            range_min=float(scan["range_min"]),
            range_max=float(scan["range_max"]),
            ranges=_float32_array(scan["ranges"]),
            intensities=_float32_array(scan.get("intensities")),
        )


@dataclass(slots=True)
class ObjectState(Message):
    """Topic-shaped messages and simulator metadata for one object."""

    header: Header
    id: int
    name: str
    role: str
    kinematics: str | None
    shape: str
    odom: Odometry
    scan: LaserScan | None
    goal: np.ndarray | None
    static: bool
    arrive: bool
    collision: bool
    collision_ids: list[int] = field(default_factory=list)
    scans: list[LaserScan] = field(default_factory=list)

    @classmethod
    def from_object(
        cls,
        obj: Any,
        *,
        stamp: float = 0.0,
        seq: int = 0,
        frame_id: str = "world",
    ) -> ObjectState:
        """Capture an object as conventional ``odom`` and ``scan`` topics."""
        lidar_sensors = [
            sensor
            for sensor in obj.sensors
            if sensor.sensor_type in {"lidar2d", "fmcw_lidar2d"}
        ]
        scans = [
            LaserScan.from_sensor(
                sensor,
                stamp=stamp,
                seq=seq,
                frame_id=(
                    f"{obj.name}/laser" if index == 0 else f"{obj.name}/laser_{index}"
                ),
            )
            for index, sensor in enumerate(lidar_sensors)
        ]
        return cls(
            header=Header(seq=int(seq), stamp=float(stamp), frame_id=frame_id),
            id=int(obj.id),
            name=str(obj.name),
            role=str(obj.role),
            kinematics=obj.kinematics,
            shape=str(obj.shape),
            odom=Odometry.from_object(
                obj,
                stamp=stamp,
                seq=seq,
                frame_id=frame_id,
            ),
            scan=scans[0] if scans else None,
            goal=_copy_array(obj.goal),
            static=bool(obj.static),
            arrive=bool(obj.arrive),
            collision=bool(obj.collision),
            collision_ids=[int(other.id) for other in obj.collision_obj],
            scans=scans,
        )

    @property
    def sensors(self) -> list[LaserScan]:
        """Return all LiDAR messages as a compatibility alias for ``scans``."""
        return list(self.scans)


@dataclass(slots=True)
class WorldState(Message):
    """Complete point-in-time state of an IR-SIM environment."""

    header: Header
    name: str
    status: str
    step_time: float
    objects: list[ObjectState] = field(default_factory=list)

    @classmethod
    def from_env(cls, env: Any, frame_id: str = "world") -> WorldState:
        """Capture the current state and sensor data from an environment."""
        seq = int(env.world_param.count)
        stamp = float(env.time)
        return cls(
            header=Header(seq=seq, stamp=stamp, frame_id=frame_id),
            name=str(env._world.name),
            status=str(env.status),
            step_time=float(env.step_time),
            objects=[
                ObjectState.from_object(
                    obj,
                    stamp=stamp,
                    seq=seq,
                    frame_id=frame_id,
                )
                for obj in env.objects
            ],
        )

    @property
    def robots(self) -> list[ObjectState]:
        """Return the robot messages in this snapshot."""
        return [obj for obj in self.objects if obj.role == "robot"]

    @property
    def obstacles(self) -> list[ObjectState]:
        """Return the obstacle messages in this snapshot."""
        return [obj for obj in self.objects if obj.role == "obstacle"]
