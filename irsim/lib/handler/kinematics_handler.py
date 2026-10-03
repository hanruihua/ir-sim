from abc import ABC, abstractmethod
from math import atan2, cos, sin
from typing import Any, ClassVar

import numpy as np

from irsim.config import palette_param
from irsim.lib.algorithm.kinematics import (
    ackermann_kinematics,
    differential_kinematics,
    omni_angular_kinematics,
    omni_kinematics,
)
from irsim.util.util import (
    check_number,
    vel_diff2world,
    vel_omni2world,
    vel_world2omni,
)

# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

_kinematics_registry: dict[str, type["KinematicsHandler"]] = {}

# Ground friction of a body decelerates it by ``friction * gravity``; the
# defaults of both, and of restitution and the drive lag, are world
# parameters (``irsim.config.world_param``), set under ``world`` in YAML.


def register_kinematics(name: str):
    """Decorator to register a KinematicsHandler subclass.

    Any extra key under ``kinematics:`` in YAML is passed to the subclass's
    ``__init__`` (after ``name``, ``noise``, ``alpha``), so a registered model
    can take its own parameters, e.g. ``kinematics: {name: lag, tau: 0.5}``.

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


def _heading(state: np.ndarray) -> float:
    """Read the heading from a state that may not carry one."""
    return state[2, 0] if state.shape[0] > 2 else 0.0


class KinematicsHandler(ABC):
    """
    Abstract base class for handling robot kinematics.

    Subclasses should set the class-attribute metadata described below and
    implement :meth:`step`, :meth:`velocity_to_xy`, :meth:`compute_max_speed`,
    and :meth:`compute_heading`; :meth:`velocity_from_xy` may be overridden
    when a world-frame push maps to the command frame in another way. A
    subclass may add keyword parameters to ``__init__``; any extra key under
    ``kinematics:`` in YAML is passed to it.
    """

    # -- Metadata (override in subclasses) --
    action_dim: int = 2
    min_state_dim: int = 3
    state_dim: int = 3
    vel_max: ClassVar[list[float]] = [1, 1]
    vel_min: ClassVar[list[float]] = [-1, -1]
    acce: ClassVar[list[float]] = [float("inf"), float("inf")]
    color: str | None = (
        None  # explicit robot color; None uses palette_param.<color_key>
    )
    obstacle_color: str | None = (
        None  # explicit obstacle color; None uses palette_param.obstacle
    )
    color_key: str = "robot"  # palette entry robots with this model default to
    description: str | None = None
    show_arrow: bool = True
    # A passive model does not drive the object: its state only changes when a
    # contact pushes it, so the object is static unless its mass is finite.
    passive: bool = False
    default_mass: float = 1.0  # mass of a non-static object given none
    # Row of the velocity vector that is a yaw rate, so a contact that turns
    # the object can be folded into its velocity; ``None`` when there is none.
    yaw_rate_row: int | None = None
    # Rows of the velocity vector that move the body, whose change per step
    # the wheels' grip on the ground caps in ``collision_mode: contact``.
    translation_rows: tuple[int, ...] = (0,)
    # Drive lag set under ``kinematics: {tau: ...}``; ``None`` means the
    # world's ``drive_tau``. It acts in ``collision_mode: contact`` only: the
    # object's velocity then follows its command as a first-order response
    # with this constant, as the PD-driven wheels of a base do.
    tau: float | None = None

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

    @classmethod
    def default_color(cls, role: str = "robot", pushable: bool = False) -> str:
        """Default color of an object with this model.

        The explicit ``color`` / ``obstacle_color`` class attribute wins when
        set; otherwise the value comes from :mod:`irsim.config.palette_param`
        (``obstacle`` for obstacles, the model's ``color_key`` for robots), so
        a palette changed at runtime applies to objects created afterwards.

        Args:
            role: ``"robot"`` or ``"obstacle"``.
            pushable: Whether contacts can move the object. A driven model
                keeps its role color either way; the passive model turns
                orange (``palette_param.pushable``).

        Returns:
            str: A Matplotlib color.
        """
        if role == "obstacle":
            if cls.obstacle_color is not None:
                return cls.obstacle_color
            return palette_param.obstacle
        if cls.color is not None:
            return cls.color
        return getattr(palette_param, cls.color_key)

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
        angle ``state[2]``, which is what :func:`~irsim.util.util.vel_diff2world`
        does. Subclasses with different velocity semantics (e.g.
        omnidirectional) should override this.

        Args:
            state (np.ndarray): Current state vector.
            velocity (np.ndarray): Velocity vector in kinematics frame.

        Returns:
            np.ndarray: (2, 1) array of [vx, vy].
        """
        if len(velocity.shape) == 0:
            return np.zeros((2, 1))

        return vel_diff2world(state[2, 0], velocity)

    def velocity_from_xy(
        self, state: np.ndarray, velocity_xy: np.ndarray
    ) -> np.ndarray:
        """Express a world-frame ``[vx, vy]`` motion in this model's command frame.

        The inverse of :meth:`velocity_to_xy` as far as the model allows; the
        ``contact`` collision mode uses it to fold a push into the velocity.
        The default keeps only the forward component along the heading
        ``state[2]``, which is all a differential or car-like model can
        express: a sideways push has no command counterpart.

        Args:
            state (np.ndarray): Current state vector.
            velocity_xy (np.ndarray): World-frame velocity ``[vx, vy]`` (2x1).

        Returns:
            np.ndarray: ``(action_dim, 1)`` velocity in the command frame.
        """
        theta = float(state[2, 0]) if state.shape[0] > 2 else 0.0
        out = np.zeros((self.action_dim, 1))
        out[0, 0] = float(velocity_xy[0, 0]) * cos(theta) + float(
            velocity_xy[1, 0]
        ) * sin(theta)
        return out

    def coast(
        self,
        velocity: np.ndarray,
        step_time: float,
        friction: float,
        gyration: float = 0.0,
        gravity: float = 9.81,
    ) -> np.ndarray:
        """Velocity the object keeps for one step when nothing commands it.

        A driven model stops: without a command or behavior it stays put. A
        passive body keeps its momentum and is slowed by ground friction.

        Args:
            velocity (np.ndarray): Velocity at the end of the previous step.
            step_time (float): Time step.
            friction (float): The object's friction coefficient.
            gyration (float): Radius of gyration, the lever at which a
                spinning body's rim slides; ``0`` when it cannot spin.
            gravity (float): Gravitational acceleration, the world's.

        Returns:
            np.ndarray: Velocity to integrate this step, same shape as
            ``velocity``.
        """
        return np.zeros_like(velocity, dtype=float)

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
    translation_rows = (0, 1)
    min_state_dim = 3
    state_dim = 3
    vel_max: ClassVar[list[float]] = [1, 1]
    vel_min: ClassVar[list[float]] = [-1, -1]
    acce: ClassVar[list[float]] = [float("inf"), float("inf")]
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
        return vel_omni2world(_heading(state), velocity[0:2])

    def velocity_from_xy(
        self, state: np.ndarray, velocity_xy: np.ndarray
    ) -> np.ndarray:
        """Rotate a world-frame ``[vx, vy]`` into the body frame."""
        return vel_world2omni(_heading(state), velocity_xy)

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
    translation_rows = (0, 1)
    yaw_rate_row = 2
    min_state_dim = 3
    state_dim = 3
    vel_max: ClassVar[list[float]] = [1, 1, 1]
    vel_min: ClassVar[list[float]] = [-1, -1, -1]
    acce: ClassVar[list[float]] = [float("inf"), float("inf"), float("inf")]
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
        return vel_omni2world(_heading(state), velocity[0:2])

    def velocity_from_xy(
        self, state: np.ndarray, velocity_xy: np.ndarray
    ) -> np.ndarray:
        """Rotate a world-frame ``[vx, vy]`` into the body frame; a push adds no yaw rate."""
        out = np.zeros((3, 1))
        out[:2] = vel_world2omni(_heading(state), velocity_xy)
        return out

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
    yaw_rate_row = 1
    min_state_dim = 3
    state_dim = 3
    vel_max: ClassVar[list[float]] = [1, 1]
    vel_min: ClassVar[list[float]] = [-1, -1]
    acce: ClassVar[list[float]] = [float("inf"), float("inf")]
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
    color_key = "robot_acker"
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


class PassiveKinematics(KinematicsHandler):
    """Kinematics of an object that does not drive itself.

    Every object configured without ``kinematics`` gets this model, and so
    does the ``static`` name. Without a finite mass the object is static. In
    ``collision_mode: contact`` a finite ``mass`` lets contacts push the
    body: its velocity is the world-frame ``[vx, vy, yaw_rate]`` a push gave
    it, it keeps that momentum once released, and ground friction slows it by
    ``friction * gravity`` (the world's) per second until it stops, as a box sliding on a
    floor does in a physics engine; a spin slows at the same rate taken at
    the body's radius of gyration.
    """

    action_dim = 3
    min_state_dim = 3
    state_dim = 3
    vel_max: ClassVar[list[float]] = [1, 1, 1]
    vel_min: ClassVar[list[float]] = [-1, -1, -1]
    acce: ClassVar[list[float]] = [float("inf"), float("inf"), float("inf")]
    show_arrow = False
    passive = True
    default_mass = float("inf")
    yaw_rate_row = 2

    def __init__(
        self, name: str | None = None, noise: bool = False, alpha: list | None = None
    ):
        super().__init__(name, noise, alpha)

    @classmethod
    def default_color(cls, role: str = "robot", pushable: bool = False) -> str:
        """The palette's pushable color when contacts can move the body, else the obstacle color."""
        return palette_param.pushable if pushable else palette_param.obstacle

    def step(
        self, state: np.ndarray, velocity: np.ndarray, step_time: float
    ) -> np.ndarray:
        """Integrate the world-frame velocity and yaw rate the body coasts with."""
        new_state = state.astype(float)
        rows = min(3, velocity.shape[0], state.shape[0]) if velocity.ndim == 2 else 0
        new_state[:rows] += velocity[:rows] * step_time
        return new_state

    def coast(
        self,
        velocity: np.ndarray,
        step_time: float,
        friction: float,
        gyration: float = 0.0,
        gravity: float = 9.81,
    ) -> np.ndarray:
        """Keep the momentum of the last push, slowed by ground friction."""
        out = np.zeros_like(velocity, dtype=float)
        if velocity.ndim != 2 or velocity.shape[0] < 2:
            return out
        brake = friction * gravity * step_time
        speed = float(np.hypot(velocity[0, 0], velocity[1, 0]))
        if speed > 0.0:
            out[:2] = velocity[:2] * (max(0.0, speed - brake) / speed)
        if velocity.shape[0] > 2 and gyration > 0.0:
            rate = float(velocity[2, 0])
            out[2, 0] = np.sign(rate) * max(0.0, abs(rate) - brake / gyration)
        return out

    def velocity_to_xy(self, state: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        """The velocity already is the world-frame ``[vx, vy]`` of the last push."""
        out = np.zeros((2, 1))
        if velocity.ndim == 2:
            rows = min(2, velocity.shape[0])
            out[:rows] = velocity[:rows]
        return out

    def velocity_from_xy(
        self, state: np.ndarray, velocity_xy: np.ndarray
    ) -> np.ndarray:
        """A push is stored as the world-frame velocity itself; it adds no spin."""
        out = np.zeros((3, 1))
        out[:2] = np.asarray(velocity_xy, dtype=float).reshape(2, 1)
        return out

    def compute_max_speed(self, vel_max: np.ndarray) -> float:
        """A passive body has no drive of its own."""
        return 0.0


class KinematicsFactory:
    """
    Factory class to create kinematics handlers.
    """

    @staticmethod
    def create_kinematics(
        name: str | None = None,
        noise: bool = False,
        alpha: list | None = None,
        mode: str | None = None,
        wheelbase: float | None = None,
        role: str = "robot",
        tau: float | None = None,
        *,
        shape_wheelbase: float | None = None,
        **kwargs: Any,
    ) -> KinematicsHandler:
        """Create a kinematics handler from a YAML ``kinematics`` block.

        Args:
            name: Registered kinematics name: ``diff``, ``omni``, ``omni_angular``,
                ``acker``, or a custom name registered with
                :func:`register_kinematics`. ``None`` defaults to ``diff``;
                ``static`` gives the :class:`PassiveKinematics` of an object
                that never drives itself.
            noise: Whether to apply motion noise.
            alpha: Noise parameters passed to the handler.
            mode: Steering mode of ``acker`` handlers; forwarded when given.
            wheelbase: Wheelbase set under ``kinematics``; forwarded when given
                and takes precedence over ``shape_wheelbase``.
            role: Object role, retained for API compatibility.
            tau: Drive lag in seconds for ``collision_mode: contact``; see
                :attr:`KinematicsHandler.tau`. ``None`` uses the world's
                ``drive_tau``.
            shape_wheelbase: Wheelbase taken from a car-like shape; the fallback
                for ``acker`` handlers (default ``1.0``) when ``wheelbase`` is
                not given.
            **kwargs: Any other ``kinematics`` key, forwarded to the handler's
                ``__init__`` (a custom handler's own parameters).

        Returns:
            KinematicsHandler: Handler instance for ``name``.

        Raises:
            NotImplementedError: If ``name`` is not registered.
            TypeError: If the handler does not accept a forwarded parameter.
        """
        if tau is not None:
            tau = check_number(tau, "kinematics tau", low=0.0)

        if name is None:
            handler: KinematicsHandler = DifferentialKinematics("diff", noise, alpha)
        elif name.lower() == "static":
            handler = PassiveKinematics("static", noise, alpha)
        else:
            name = name.lower()
            handler_cls = _kinematics_registry.get(name)
            if handler_cls is None:
                raise NotImplementedError(f"Kinematics {name!r} is not registered")
            if mode is not None:
                kwargs["mode"] = mode
            if wheelbase is not None:
                kwargs["wheelbase"] = wheelbase
            if issubclass(handler_cls, AckermannKinematics):
                kwargs.setdefault("wheelbase", shape_wheelbase or 1.0)
            handler = handler_cls(name, noise, alpha, **kwargs)
        handler.tau = tau
        return handler

    @staticmethod
    def get_handler_class(name: str) -> type[KinematicsHandler] | None:
        """Look up a registered handler class by name without instantiation.

        Args:
            name (str): Kinematics name (e.g. ``"diff"``, ``"omni"``).

        Returns:
            type[KinematicsHandler] | None: The class, or ``None`` if not found.
        """
        return _kinematics_registry.get(name.lower() if name else "")
