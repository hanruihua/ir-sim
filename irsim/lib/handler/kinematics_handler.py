from abc import ABC, abstractmethod
from math import atan2, cos, sin
from typing import ClassVar

import numpy as np

from irsim.lib.algorithm.kinematics import (
    ackermann_kinematics,
    differential_kinematics,
    omni_angular_kinematics,
    omni_kinematics,
)
from irsim.util.util import log_warning

# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

_kinematics_registry: dict[str, type["KinematicsHandler"]] = {}


def register_kinematics(name: str):
    """Decorator to register a KinematicsHandler subclass.

    Args:
        name (str): Name used in YAML configs (e.g. ``"diff"``, ``"omni"``).

    Returns:
        Callable: Class decorator that registers and returns the class unchanged.
    """

    # Normalize registry key to ensure consistency with lookup, which
    # lowercases names (see KinematicsFactory usage).
    normalized_name = name.lower()

    def decorator(cls):
        # Prevent accidental overrides when the same (normalized) name is
        # registered for multiple different handler classes.
        existing = _kinematics_registry.get(normalized_name)
        if existing is not None and existing is not cls:
            raise ValueError(
                f"Kinematics handler '{normalized_name}' is already registered "
                f"for class {existing.__name__}"
            )
        _kinematics_registry[normalized_name] = cls
        return cls

    return decorator


# ---------------------------------------------------------------------------
# Base class
# ---------------------------------------------------------------------------


class KinematicsHandler(ABC):
    """
    Abstract base class for handling robot kinematics.

    Subclasses should set the class-attribute metadata described below and
    implement :meth:`step`, :meth:`velocity_to_xy`, :meth:`compute_max_speed`,
    and :meth:`compute_heading`.
    """

    # -- Metadata (override in subclasses) --
    action_dim: int = 2
    min_state_dim: int = 3
    state_dim: int = 3
    vel_max: ClassVar[list[float]] = [1, 1]
    vel_min: ClassVar[list[float]] = [-1, -1]
    acce: ClassVar[list[float]] = [float("inf"), float("inf")]
    color: str = "g"
    obstacle_color: str = "k"
    description: str | None = None
    show_arrow: bool = True

    def __init__(self, name, noise: bool = False, alpha: list | None = None):
        """
        Initialize the KinematicsHandler class.

        Args:
            name (str): Kinematics model name.
            noise (bool): Boolean indicating whether to add noise to the velocity (default False).
            alpha (list): List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]).
        """

        self.name = name
        self.noise = noise
        self.alpha = alpha or [0.03, 0, 0, 0.03]

    @abstractmethod
    def step(
        self, state: np.ndarray, velocity: np.ndarray, step_time: float
    ) -> np.ndarray:
        """
        Calculate the next state using the kinematics model.

        Args:
            state (np.ndarray): Current state.
            velocity (np.ndarray): Velocity vector.
            step_time (float): Time step for simulation.

        Returns:
            np.ndarray: Next state.
        """

    def velocity_to_xy(self, state: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        """Convert velocity to [vx, vy] in world frame.

        The default implementation follows differential-drive conventions:
        ``velocity[0]`` is the linear speed projected through the heading
        angle ``state[2]``. Subclasses with different velocity semantics
        (e.g. omnidirectional) should override this.

        Args:
            state (np.ndarray): Current state vector.
            velocity (np.ndarray): Velocity vector in kinematics frame.

        Returns:
            np.ndarray: (2, 1) array of [vx, vy].
        """
        if len(velocity.shape) == 0:
            return np.zeros((2, 1))
        vel_linear = velocity[0, 0]
        theta = state[2, 0]
        vx = vel_linear * cos(theta)
        vy = vel_linear * sin(theta)
        return np.array([[vx], [vy]])

    def compute_max_speed(self, vel_max: np.ndarray) -> float:
        """Compute the scalar maximum speed from the vel_max vector.

        The default implementation follows differential-drive conventions:
        the first component ``vel_max[0, 0]`` is the translational speed
        limit. Subclasses where max speed is derived differently (e.g.
        omnidirectional using the L2 norm) should override this.

        Args:
            vel_max (np.ndarray): Maximum velocity vector.

        Returns:
            float: Scalar maximum speed.
        """
        return float(vel_max[0, 0])

    def compute_heading(self, state: np.ndarray, velocity: np.ndarray) -> float:
        """Compute the heading angle.

        The default implementation follows differential-drive conventions:
        heading is ``state[2]`` (the orientation component). Returns 0.0
        if the state has fewer than 3 rows.

        Args:
            state (np.ndarray): Current state vector.
            velocity (np.ndarray): Current velocity vector.

        Returns:
            float: Heading in radians.
        """
        return float(state[2, 0]) if state.shape[0] > 2 else 0.0


# ---------------------------------------------------------------------------
# Concrete subclasses
# ---------------------------------------------------------------------------


@register_kinematics("omni")
class OmniKinematics(KinematicsHandler):
    """Omnidirectional model with body-frame translational velocity.

    Velocity is ``[forward, lateral]`` in the robot body frame. The state is
    ``[x, y, theta]`` and ``theta`` is preserved by :meth:`step`.
    """

    action_dim = 2
    min_state_dim = 3
    state_dim = 3
    vel_max: ClassVar[list[float]] = [1, 1]
    vel_min: ClassVar[list[float]] = [-1, -1]
    acce: ClassVar[list[float]] = [float("inf"), float("inf")]
    color = "g"
    obstacle_color = "k"
    description = None
    show_arrow = False

    def __init__(self, name, noise, alpha):
        super().__init__(name, noise, alpha)
        if alpha is None:
            self.alpha = [0.03, 0.03]

    def step(
        self, state: np.ndarray, velocity: np.ndarray, step_time: float
    ) -> np.ndarray:
        """Advance omnidirectional state one step.

        Args:
            state (np.ndarray): Current state [x, y, theta].
            velocity (np.ndarray): Velocity [forward, lateral] in body frame.
            step_time (float): Time step.

        Returns:
            np.ndarray: New state [x, y, theta] (theta preserved).
        """
        return omni_kinematics(state[0:3], velocity, step_time, self.noise, self.alpha)

    def velocity_to_xy(self, state: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        """Convert body-frame translation to world-frame ``[vx, vy]``.

        Args:
            state (np.ndarray): Current state ``[x, y, theta]``.
            velocity (np.ndarray): Body-frame ``[forward, lateral]`` velocity.

        Returns:
            np.ndarray: ``(2, 1)`` world-frame velocity.
        """
        theta = state[2, 0] if state.shape[0] > 2 else 0.0
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        fwd = velocity[0, 0]
        lat = velocity[1, 0]
        vx = fwd * cos_t - lat * sin_t
        vy = fwd * sin_t + lat * cos_t
        return np.array([[vx], [vy]])

    def compute_max_speed(self, vel_max: np.ndarray) -> float:
        """Compute translational speed limit from forward/lateral limits.

        Args:
            vel_max (np.ndarray): Maximum ``[forward, lateral]`` velocity.

        Returns:
            float: Euclidean norm of the translational velocity bound.
        """
        return float(np.linalg.norm(vel_max))

    def compute_heading(self, state: np.ndarray, velocity: np.ndarray) -> float:
        """Compute travel-direction heading for omnidirectional motion.

        Args:
            state (np.ndarray): Current state ``[x, y, theta]``.
            velocity (np.ndarray): Body-frame ``[forward, lateral]`` velocity.

        Returns:
            float: World-frame velocity direction in radians.
        """
        theta = state[2, 0] if state.shape[0] > 2 else 0.0
        return float(atan2(velocity[1, 0], velocity[0, 0])) + theta


@register_kinematics("omni_angular")
class OmniAngularKinematics(KinematicsHandler):
    """Omnidirectional kinematics with angular velocity control.

    Velocity is ``[forward, lateral, yaw_rate]`` in body frame.
    The kinematics function converts to world-frame internally.

    Note: ``compute_heading`` is intentionally inherited from the base class
    (returns ``state[2]``), because this robot has independent yaw control
    and its heading IS the orientation angle, unlike :class:`OmniKinematics`
    which derives heading from velocity direction.
    """

    action_dim = 3
    min_state_dim = 3
    state_dim = 3
    vel_max: ClassVar[list[float]] = [1, 1, 1]
    vel_min: ClassVar[list[float]] = [-1, -1, -1]
    acce: ClassVar[list[float]] = [float("inf"), float("inf"), float("inf")]
    color = "g"
    obstacle_color = "k"
    description = None
    show_arrow = True

    def __init__(self, name, noise, alpha):
        super().__init__(name, noise, alpha)
        if alpha is None:
            self.alpha = [0.03, 0.03, 0.03]

    def step(
        self, state: np.ndarray, velocity: np.ndarray, step_time: float
    ) -> np.ndarray:
        """Advance omnidirectional-angular state one step.

        Args:
            state (np.ndarray): Current state [x, y, theta].
            velocity (np.ndarray): Velocity [forward, lateral, yaw_rate] in body frame.
            step_time (float): Time step.

        Returns:
            np.ndarray: New state [x, y, theta].
        """

        return omni_angular_kinematics(
            state[0:3], velocity, step_time, self.noise, self.alpha
        )

    def velocity_to_xy(self, state: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        """Convert body-frame translation to world-frame ``[vx, vy]``.

        The yaw-rate component is ignored for this projection.

        Args:
            state (np.ndarray): Current state ``[x, y, theta]``.
            velocity (np.ndarray): Body-frame ``[forward, lateral, yaw_rate]``.

        Returns:
            np.ndarray: ``(2, 1)`` world-frame velocity.
        """
        theta = state[2, 0]
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        fwd = velocity[0, 0]
        lat = velocity[1, 0]
        vx = fwd * cos_t - lat * sin_t
        vy = fwd * sin_t + lat * cos_t
        return np.array([[vx], [vy]])

    def compute_max_speed(self, vel_max: np.ndarray) -> float:
        """Compute translational speed limit from the first two components.

        Args:
            vel_max (np.ndarray): Maximum ``[forward, lateral, yaw_rate]`` velocity.

        Returns:
            float: Euclidean norm of forward/lateral velocity bounds.
        """
        return float(np.linalg.norm(vel_max[0:2]))


@register_kinematics("diff")
class DifferentialKinematics(KinematicsHandler):
    """Differential-drive model with ``[linear, angular]`` velocity."""

    action_dim = 2
    min_state_dim = 3
    state_dim = 3
    vel_max: ClassVar[list[float]] = [1, 1]
    vel_min: ClassVar[list[float]] = [-1, -1]
    acce: ClassVar[list[float]] = [float("inf"), float("inf")]
    color = "g"
    obstacle_color = "k"
    description = None
    show_arrow = True

    def __init__(self, name, noise, alpha):
        super().__init__(name, noise, alpha)

    def step(
        self, state: np.ndarray, velocity: np.ndarray, step_time: float
    ) -> np.ndarray:
        """Advance differential-drive state one step.

        Args:
            state (np.ndarray): Current state [x, y, theta].
            velocity (np.ndarray): [linear, angular].
            step_time (float): Time step.

        Returns:
            np.ndarray: Next state.
        """
        return differential_kinematics(
            state, velocity, step_time, self.noise, self.alpha
        )


@register_kinematics("acker")
class AckermannKinematics(KinematicsHandler):
    """Ackermann car-like model with steering or angular-rate control.

    The state is ``[x, y, theta, steer]``. In ``mode="steer"``, velocity is
    interpreted as ``[linear, steer]``; other modes are handled by
    :func:`irsim.lib.algorithm.kinematics.ackermann_kinematics`.
    """

    action_dim = 2
    min_state_dim = 4
    state_dim = 4
    vel_max: ClassVar[list[float]] = [1, 1]
    vel_min: ClassVar[list[float]] = [-1, -1]
    acce: ClassVar[list[float]] = [float("inf"), float("inf")]
    color = "y"
    obstacle_color = "k"
    description = "car_green.png"
    show_arrow = True

    def __init__(
        self,
        name,
        noise: bool = False,
        alpha: list | None = None,
        mode: str = "steer",
        wheelbase: float = 1.0,
    ):
        super().__init__(name, noise, alpha)
        self.mode = mode
        self.wheelbase = wheelbase

    def step(
        self, state: np.ndarray, velocity: np.ndarray, step_time: float
    ) -> np.ndarray:
        """Advance Ackermann-steered state one step.

        Args:
            state (np.ndarray): Current state [x, y, theta, steer].
            velocity (np.ndarray): Depending on mode: [linear, steer] or [linear, angular].
            step_time (float): Time step.

        Returns:
            np.ndarray: Next state.
        """
        return ackermann_kinematics(
            state,
            velocity,
            step_time,
            self.noise,
            self.alpha,
            self.mode,
            self.wheelbase,
        )


class KinematicsFactory:
    """
    Factory class to create kinematics handlers.
    """

    @staticmethod
    def create_kinematics(
        name: str | None = None,
        noise: bool = False,
        alpha: list | None = None,
        mode: str = "steer",
        wheelbase: float | None = None,
        role: str = "robot",
    ) -> KinematicsHandler:
        """Create a kinematics handler from a YAML ``kinematics.name`` value.

        Args:
            name: Registered kinematics name, such as ``diff``, ``omni``,
                ``omni_angular``, or ``acker``. ``None`` uses the fallback.
            noise: Whether to apply motion noise.
            alpha: Noise parameters passed to the handler.
            mode: Ackermann mode, used only by ``acker``.
            wheelbase: Ackermann wheelbase; defaults to ``1.0`` for ``acker``.
            role: Object role used for warnings.

        Returns:
            KinematicsHandler: Registered handler instance, or a differential
            handler fallback when the name is missing or unknown.
        """
        name = name.lower() if name else None

        handler_cls = _kinematics_registry.get(name) if name else None

        if handler_cls is not None:
            # AckermannKinematics accepts extra kwargs
            if issubclass(handler_cls, AckermannKinematics):
                return handler_cls(name, noise, alpha, mode, wheelbase or 1.0)
            return handler_cls(name, noise, alpha)

        # elif name == 'rigid3d':
        #     return Rigid3DKinematics(name, noise, alpha)
        if role == "robot":
            log_warning(
                f"Unknown kinematics type: {name}, the robot will be stationary."
            )

        # Fallback to a stationary kinematics handler (differential with zero wheelbase)
        return DifferentialKinematics(name or "diff", noise, alpha)

    @staticmethod
    def get_handler_class(name: str) -> type[KinematicsHandler] | None:
        """Look up a registered handler class by name without instantiation.

        Args:
            name (str): Kinematics name (e.g. ``"diff"``, ``"omni"``).

        Returns:
            type[KinematicsHandler] | None: The class, or ``None`` if not found.
        """
        return _kinematics_registry.get(name.lower() if name else "")
