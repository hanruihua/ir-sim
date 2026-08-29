"""
Class EnvBase is the base class of the environment.

It loads YAML configuration via ``EnvConfig`` to construct the world,
robots, obstacles, and maps, and provides the core simulation loop.
The environment can be reconfigured in-place (same figure) by reloading
the YAML at runtime.

Author: Ruihua Han
"""

from __future__ import annotations

import contextlib
import importlib
from typing import Any, Literal, cast

import matplotlib
import numpy as np
from matplotlib import pyplot as plt
from shapely import Polygon
from shapely.strtree import STRtree

from irsim.config import env_param, path_param, world_param
from irsim.config.env_param import EnvParam
from irsim.config.path_param import PathManager
from irsim.config.world_param import WorldParam
from irsim.env.env_config import EnvConfig
from irsim.gui.mouse_control import MouseControl
from irsim.lib import random_generate_polygon
from irsim.msg import ObjectState, Odometry, WorldState
from irsim.util.message import resolve_message_targets
from irsim.util.random import rng, set_seed
from irsim.util.util import (
    find_duplicates,
    find_object_by_identity,
    normalize_actions,
    plot_only,
    to_numpy,
)
from irsim.world import ObjectBase, ObjectFactory

from .env_logger import EnvLogger

with contextlib.suppress(ImportError):  # KeyboardControl needs pynput or an MPL backend
    from irsim.gui.keyboard_control import KeyboardControl

# Define backend preferences for different operating systems
BACKEND_PREFERENCES = {
    "Darwin": ["MacOSX", "TkAgg", "Qt5Agg", "Agg"],  # macOS
    "Windows": ["TkAgg", "Qt5Agg", "Agg"],  # Windows
    "Linux": ["TkAgg", "Qt5Agg", "Agg"],  # Linux
}


def _set_matplotlib_backend(backend_list: list[str]) -> bool:
    """Attempt to set matplotlib backend from preference list."""
    for backend in backend_list:
        try:
            matplotlib.use(backend)
            return True
        except Exception as e:
            print(f"Failed to use '{backend}' backend: {e}")

    print(
        "All backends failed. Falling back to 'Agg' backend. The environment will not be displayed."
    )
    matplotlib.use("Agg")
    return False


# Get the current operating system from env_param and set backend
_set_matplotlib_backend(BACKEND_PREFERENCES.get(env_param.platform_name, ["Agg"]))


class EnvBase:
    """
    The base class for simulation environments in IR-SIM.

    This class serves as the foundation for creating and managing robotic simulation
    environments. It reads YAML configuration files (through ``EnvConfig``) to create
    worlds, robots, obstacles, and map objects, and provides the core simulation loop
    functionality. The environment supports in-place reload of YAML configuration to
    update the scene in the existing figure without opening a new window.

    Args:
        world_name (str, optional): Path to the world YAML configuration file.
            If None, the environment will attempt to find a default configuration
            or use a minimal setup.
        display (bool): Whether to display the environment visualization.
            Set to False for headless operation. Default is True.
        disable_all_plot (bool): Same as ``headless``; kept for backward compatibility.
            When True, no visualization will be created even if display is True.
            Default is False.
        save_ani (bool): Whether to save the simulation as an animation file.
            Useful for creating videos of simulation runs. Default is False.
        ani_kwargs (dict, optional): Default keyword arguments for animation
            saving (see :py:meth:`.EnvPlot.save_animate`), e.g.
            ``{"suffix": ".mp4"}``. Honored by every save path, including
            quitting with ESC; per-call ``end(**kwargs)`` overrides them.
            Default is None.
        full (bool): Whether to display the visualization in full screen mode.
            Only effective on supported platforms. Default is False.
        log_file (str, optional): Path to the log file for saving simulation logs.
            If None, logs will only be output to console.
        log_level (str): Logging level for the environment. Options include
            'DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'. Default is 'INFO'.
        seed (int, optional): Seed for the random number generator. Default is None.
            If None, the seed will be set to a random value, which will make the simulation non-reproducible.
            If a fixed seed is provided, the random simulation scenario will be reproducible.
        headless (bool): Run without any figure, window, or keyboard/mouse control,
            e.g. for batch training. Implies ``display=False``; rendering, drawing, and
            saving helpers become no-ops. Default is False.
        step_mode ({"internal", "external"}, optional): Override the mode from
            ``world.step_mode`` in YAML. ``internal`` runs the normal IR-SIM
            kinematics step. ``external`` expects callers to update object
            states and velocities before calling :meth:`step`. Default is None,
            which uses the YAML value (or ``internal`` when omitted there).

    Attributes:
        display (bool): Whether to display the environment visualization.
        headless (bool): Whether the environment runs without a figure, window, or
            keyboard/mouse control.
        save_ani (bool): Whether to save animation during simulation.

        env_config (EnvConfig): Configuration loader managing YAML parsing and object creation.

        keyboard (KeyboardControl): Keyboard input handler for manual control.
        mouse (MouseControl): Mouse input handler for zoom and pan.

        pause_flag (bool): Internal flag indicating if simulation is paused.
        quit_flag (bool): Internal flag indicating if simulation should quit.
        debug_flag (bool): Internal flag for debug mode (frame-by-frame stepping).
        debug_count (int): Counter for debug mode frames.
        reset_flag (bool): Internal flag for environment reset.
        reload_flag (bool): Internal flag for YAML reload.
        save_figure_flag (bool): Internal flag to save current figure.

    Example:
        >>> # Create a basic environment
        >>> env = EnvBase("my_world.yaml")
        >>>
        >>> # Create headless environment for training
        >>> env = EnvBase("world.yaml", display=False, log_level="WARNING")
        >>>
        >>> # Create environment with animation saving
        >>> env = EnvBase("world.yaml", save_ani=True, full=True)
        >>>
        >>> # Create environment with a fixed seed for reproducibility
        >>> env = EnvBase("world.yaml", seed=42)
    """

    def __init__(
        self,
        world_name: str | None = None,
        display: bool = True,
        disable_all_plot: bool = False,
        save_ani: bool = False,
        ani_kwargs: dict[str, Any] | None = None,
        full: bool = False,
        log_file: str | None = None,
        log_level: str = "INFO",
        seed: int | None = None,
        step_mode: Literal["internal", "external"] | None = None,
        headless: bool = False,
    ) -> None:
        # Reset object ID counter so each environment starts from 0
        ObjectBase.reset_id_iter()

        # Bind per-instance config objects
        self._env_param = EnvParam()
        self._world_param = WorldParam()
        self._path_manager = PathManager()
        self._bind_config()

        # init env setting: headless means no figure, no window, no input devices
        self.headless = headless or disable_all_plot
        self.display = display and not self.headless
        set_seed(seed)

        # headless builds no figure, so the process backend is left alone:
        # switching it breaks the windows of other environments in this process
        if not self.display and not self.headless:
            matplotlib.use("Agg")

        self.save_ani = save_ani
        # Copy so later external mutation of the caller's dict can't change
        # this env's default animation-save behavior.
        self.ani_kwargs: dict[str, Any] = dict(ani_kwargs) if ani_kwargs else {}

        self._env_param.logger = EnvLogger(log_file, log_level)

        try:
            self.env_config = EnvConfig(
                world_name,
                env_param_instance=self._env_param,
                world_param_instance=self._world_param,
                step_mode=step_mode,
                disable_all_plot=self.headless,
            )
        except Exception as e:
            self.logger.critical(f"YAML Configuration load failed: {e}")
            raise

        (
            self._world,
            self._objects,
            self._env_plot,
            self._robot_collection,
            self._obstacle_collection,
            self._map_collection,
            self._object_groups,
        ) = self.env_config.initialize_objects()

        # Wire env reference to all objects for param access
        self._wire_env_to_objects()

        self.build_tree()
        self._env_param.objects = self._objects
        self.validate_unique_names()

        self.keyboard = None
        self.mouse = None
        if self._env_plot is None:
            # No figure for keyboard/mouse control to attach to.
            if self._world_param.control_mode == "keyboard":
                self.logger.warning(
                    "Keyboard control needs a figure; headless mode forces auto control."
                )
            self._world_param.control_mode = "auto"
        else:
            # Try to initialize keyboard control (pynput or MPL backend inside KeyboardControl)
            try:
                keyboard_config = self.env_config.parse["gui"].get("keyboard", {})
                self.keyboard = KeyboardControl(env_ref=self, **keyboard_config)
            except Exception as e:
                self.logger.error(
                    f"Keyboard control unavailable error: {e}. Auto control applied. "
                    "Install 'pynput' or set backend='mpl' in YAML keyboard config."
                )
                self._world_param.control_mode = "auto"

            mouse_config = self.env_config.parse["gui"].get("mouse", {})
            self.mouse = MouseControl(self._env_plot.ax, **mouse_config)

        # flag for keyboard control
        self.pause_flag = False
        self.quit_flag = False
        self.debug_flag = False
        self.debug_count = 0
        self.reset_flag = False
        self.reload_flag = False
        self.save_figure_flag = False

        if full and not self.headless:  # headless has no figure to enlarge
            mng = plt.get_current_fig_manager()
            if mng is not None:
                mng.full_screen_toggle()

        # Log simulation start
        self.logger.info(
            f"Simulation environment '{self._world.name}' started. Step time {self._world.step_time:.3f} s."
        )

    def __del__(self):
        """Clean up resources when the environment is garbage-collected.

        Note:
            Main resource cleanup is performed in :py:meth:`end`.
        """
        pass

    def __str__(self):
        """Return a human-readable summary of the environment.

        Returns:
            str: Summary string including the world name.
        """
        return f"Environment: {self._world.name}"

    def _bind_config(self) -> None:
        """Bind this env's config instances to the module-level delegates."""
        env_param.bind(self._env_param)
        world_param.bind(self._world_param)
        path_param.bind(self._path_manager)

    def _wire_env_to_objects(self) -> None:
        """Set env reference on all objects for param access."""
        for obj in self._objects:
            obj._env = self

    @normalize_actions
    def step(
        self,
        action: np.ndarray | list[Any] | tuple[Any, ...] | dict[str, Any] | None = None,
        action_id: int | str | list[int | str] | None = None,
    ) -> None:
        """
        Perform a single simulation step in the environment.

        This method advances the simulation by one time step, applying the given actions
        to the specified robots and updating all objects in the environment.

        Args:
            action (Union[np.ndarray, list, tuple, dict], optional): Action(s) to be performed in the environment.
                Can be a single action, a list/tuple of actions, or a dict ``{name: action}``.
                Action format depends on robot type:

                - **Differential robot**: [linear_velocity, angular_velocity]
                - **Omnidirectional robot**: [velocity_x, velocity_y]
                - **Ackermann robot**: [linear_velocity, steering_angle]

                If None, keyboard control is applied when enabled; otherwise robots use configured behaviors.
                Robots without a configured behavior remain static.

        Note - Priority Order:
                    1. Apply keyboard control for the specified ``action_id`` if enabled.
                    2. Apply the provided ``action`` (list of numpy arrays) to robots by ``action_id`` (int or list of int).
                    3. For remaining robots, fall back to their configured behaviors when ``action`` is ``None``.

            action_id (Union[int, str, list], optional): Object id(s) (``obj.id``) or
                name(s) to apply the action(s) to; robots are created first, so ids
                ``0..n-1`` are the robots. ``None`` (default) targets the first robot. If
                action is a list of actions and action_id is a single id, the actions are
                applied to that object and the following ones in order. A flat list of
                numbers (e.g. ``[1.0, 0.5]``) is one action; a dict ``{name: action}``
                can be passed as ``action`` instead of using ``action_id``. Surplus actions
                are dropped with a warning; an unknown id or name raises ``ValueError``.

        Note:
            - If the environment is paused, this method returns without performing any updates.
            - The method automatically handles collision detection, status updates, and plotting.
            - In keyboard control mode, the action parameter is ignored and keyboard input is used.

        Example:
            >>> # Move first robot with differential drive
            >>> env.step([1.0, 0.5])  # 1.0 m/s forward, 0.5 rad/s turn
            >>>
            >>> # Move specific robot by ID
            >>> env.step([0.8, 0.0], action_id=2)  # Move robot with ID 2
            >>>
            >>> # Move multiple robots
            >>> actions = [[1.0, 0.0], [0.5, 0.3]]
            >>> env.step(actions, action_id=[0, 1])  # Move robots 0 and 1
            >>>
            >>> # Dict keyed by robot name
            >>> env.step({"robot_0": [1.0, 0.0], "robot_2": [0.5, 0.3]})
        """

        if self.quit_flag:
            self.quit()

        if (
            self.debug_flag and self._world_param.count > self.debug_count
        ) or self.pause_flag:
            return

        if self.step_mode == "external":
            if any(item is not None for item in action):
                raise ValueError(
                    "env.step(action=...) is unavailable when step_mode='external'. "
                    "Update object states with set_state()/set_velocity(), then call "
                    "env.step() without an action."
                )
            self._objects_refresh(record_trajectory=True)
        else:
            # assign keyboard and group actions in priority order
            action = self._assign_keyboard_action(action)
            action = self._assign_group_action(action)
            self._objects_step(action, sensor_step=False)

        self._objects_sensor_step()
        self._world.step(self.objects)
        self._status_step()

    def _objects_step(self, action: list[Any], sensor_step: bool = True) -> None:
        """Advance all objects by one step with corresponding actions.

        Args:
            action (list[Any]): A list of actions aligned with ``self.objects``.
                If the list is shorter than the number of objects, it is padded
                with ``None`` for the remaining objects.
        """

        action = action + [None] * (len(self.objects) - len(action))
        [
            obj.step(action, sensor_step)
            for obj, action in zip(self.objects, action, strict=True)
        ]

        self.build_tree()

    def _objects_refresh(self, record_trajectory: bool = False) -> None:
        """Synchronize geometry with externally supplied object states.

        Geometry for every object is refreshed before rebuilding the spatial
        index, so the subsequent sensor phase observes one consistent state
        snapshot.

        Args:
            record_trajectory: Append dynamic-object states to their trajectories.
        """
        for obj in self.objects:
            obj.refresh(sensor_step=False)
            if record_trajectory and not obj.static and not obj.stop_flag:
                obj.trajectory.append(obj.state.copy())
        self.build_tree()

    def _objects_sensor_step(self) -> None:
        """step the sensors of all objects with updated states"""
        [obj.sensor_step() for obj in self.objects]

    def _object_step(
        self, action: np.ndarray | list[Any] | None, obj_id: int = 0
    ) -> None:
        """Advance a single object by one step and tick others.

        Deprecated:
            This method is slated for removal in a future release. Prefer
            :py:meth:`step` (with ``action`` and ``action_id``) to control
            specific robots.

        Args:
            action (np.ndarray | list | None): Action applied to the target object.
            obj_id (int): Target object index (or id-aligned index). Default is ``0``.
        """
        if len(self.objects) == 0:
            return

        self.objects[obj_id].step(action)
        [obj.step() for obj in self.objects if obj._id != obj_id]

    def _objects_check_status(self) -> None:
        """Refresh per-object status flags (e.g., arrival, collision)."""
        [obj.check_status() for obj in self.objects]

    def _assign_keyboard_action(self, action: list[Any]) -> list[Any]:
        """
        Assign the keyboard action to the action list.

        The keyboard produces a ``[linear, lateral, angular]`` (3x1) vector.
        For kinematics that expect a different layout the velocity is
        converted here so that the keyboard works regardless of robot type.
        """

        if (
            self._world_param.control_mode == "keyboard"
            and self.key_id < len(action)
            and self.key_id < len(self.robot_list)
        ):
            key_vel = self.key_vel
            robot = self.robot_list[self.key_id]
            kf_name = robot.kf.name if robot.kf else None
            fwd = float(key_vel[0, 0])  # W/S: forward/backward
            axis2 = float(key_vel[1, 0])  # A/D: angular (diff/acker) or lateral (omni)
            rot = float(key_vel[2, 0])  # Q/E: yaw rate (omni_angular only)

            if kf_name in ("omni", "omni_angular"):
                # A/D controls lateral strafe (linear velocity, not angular).
                # Rescale from key_ang_max to key_lv_max so that the lateral
                # speed matches the forward speed range.
                ang_max = self.keyboard.key_ang_max
                if ang_max != 0:
                    axis2 = axis2 / ang_max * self.keyboard.key_lv_max

            if kf_name == "omni":
                key_vel = np.array([[fwd], [axis2]])
            elif kf_name == "omni_angular":
                key_vel = np.array([[fwd], [axis2], [rot]])
            else:
                key_vel = np.array([[fwd], [axis2]])

            action[self.key_id] = key_vel

        return action

    def _assign_group_action(self, action: list[Any]) -> list[Any]:
        """
        Assign the group action to the action list.
        """
        group_actions = [
            ga for group in self._object_groups for ga in group.gen_group_vel()
        ]
        for i, (a, ga) in enumerate(zip(action, group_actions, strict=False)):
            if a is None and ga is not None:
                action[i] = ga

        return action

    # render
    @plot_only
    def render(
        self,
        interval: float = 0.01,
        figure_kwargs: dict[str, Any] | None = None,
        mode: str = "dynamic",
        **kwargs: Any,
    ) -> None:
        """
        Render the environment.

        Args:
            interval (float): Time interval between frames in seconds.
            figure_kwargs (dict): Additional keyword arguments for saving figures,
                see `savefig <https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.savefig.html>`__ for details.
            mode (str): One of {"dynamic", "static", "all"} specifying which types of objects
                to draw and clear each frame.
            kwargs: Additional keyword arguments for drawing components. See
                :py:meth:`.ObjectBase.plot` for details.
        """

        if figure_kwargs is None:
            figure_kwargs = {}
        if self._world.sampling:
            if self.display:
                plt.pause(interval)

            if self.save_ani:
                self.save_figure(save_gif=True, **figure_kwargs)

            self._env_plot.step(mode, self.objects, **kwargs)

        if self.save_figure_flag:
            self.save_figure(save_gif=True, **figure_kwargs)
            self.save_figure_flag = False

        if self.reset_flag:
            self.reset()

        if self.reload_flag:
            self.reload()

    @plot_only
    def show(self) -> None:
        """
        Show the environment figure.
        """

        self._env_plot.show()

    # draw various components
    @plot_only
    def draw_trajectory(
        self, traj: list[Any], traj_type: str = "g-", **kwargs: Any
    ) -> None:
        """
        Draw the trajectory on the environment figure.

        Args:
            traj (list): List of trajectory points. Each point is a 2x1 vector
                or an array of shape (2, N).
            traj_type (str): Matplotlib line style (e.g., "g-", "r--").
            **kwargs: Additional keyword arguments; forwarded to
                :py:meth:`.EnvPlot.draw_trajectory`.
        """

        self._env_plot.draw_trajectory(traj, traj_type, **kwargs)

    @plot_only
    def draw_points(
        self,
        points: list[Any],
        s: int = 30,
        c: str = "b",
        refresh: bool = True,
        **kwargs: Any,
    ) -> None:
        """
        Draw points on the environment figure.

        Args:
            points (list | np.ndarray): Either a list of 2x1 points or a numpy
                array with shape (2, N).
            s (int): Marker size.
            c (str): Marker color.
            refresh (bool): Whether to clear previous points before drawing.
            **kwargs: Additional keyword arguments, forwarded to
                `Axes.scatter <https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.scatter.html>`_.
        """

        self._env_plot.draw_points(points, s, c, refresh, **kwargs)

    @plot_only
    def draw_box(
        self, vertex: np.ndarray, refresh: bool = False, color: str = "-b"
    ) -> None:
        """
        Draw a box by the vertices.

        Args:
            vertex (np.ndarray): Vertices matrix with shape (point_dim, num_vertices).
            refresh (bool): Whether to clear previous boxes before drawing. Default is False.
            color (str): Line style/color for the box (e.g., "-b").
        """
        self._env_plot.draw_box(vertex, refresh, color)

    @plot_only
    def draw_quiver(self, point: Any, refresh: bool = False, **kwargs: Any) -> None:
        """
        Draw a single quiver (arrow) on the environment figure.

        Args:
            point: A tuple ``(x, y, u, v)`` or compatible structure defining the
                arrow's origin and vector.
            refresh (bool): Whether to clear previous quiver before drawing. Default False.
            **kwargs: Additional keyword arguments for drawing the quiver.
        """
        self._env_plot.draw_quiver(point, refresh, **kwargs)

    @plot_only
    def draw_quivers(self, points: Any, refresh: bool = False, **kwargs: Any) -> None:
        """
        Draw multiple quivers (arrows) on the environment figure.

        Args:
            points: Iterable of tuples/lists/arrays compatible with
                ``(x, y, u, v)`` per arrow.
            refresh (bool): Whether to clear previous quivers before drawing. Default False.
            **kwargs: Additional keyword arguments for drawing the quivers.
        """
        self._env_plot.draw_quivers(points, refresh, **kwargs)

    def end(self, ending_time: float = 3.0, **kwargs: Any) -> None:
        """
        End the simulation, save the animation, and close the environment.

        Args:
            ending_time (float): Time in seconds to wait before closing the figure, default is 3 seconds.
            **kwargs: Additional keyword arguments for saving the animation, see :py:meth:`.EnvPlot.save_animate` for detail.
        """

        if not self.headless:
            if self.save_ani:
                # precedence: call kwargs > env ani_kwargs > world-named default
                self._env_plot.save_animate(
                    **{
                        "ani_name": f"animation_{self._world.name}",
                        **self.ani_kwargs,
                        **kwargs,
                    }
                )

            if self.display:
                plt.pause(ending_time)
                self.logger.info(
                    f"Simulation Environment '{self._world.name}' closing in {ending_time:.2f} seconds."
                )

        if self.keyboard is not None:
            self.keyboard.close()

        if self._env_plot is not None:
            self._env_plot.close()
        self._env_param.objects = []

        self.logger.info(
            f"Simulation Environment '{self._world.name}' ended. Total time {self._world.time:.2f} seconds."
        )

    def close(self, ending_time: float = 3.0, **kwargs: Any) -> None:
        """Alias for :py:meth:`end` for Gym-style API compatibility."""
        self.end(ending_time, **kwargs)

    def quit(self) -> None:
        """
        Quit the environment.
        """
        self.quit_flag = True
        self.logger.info("Quit the environment.")
        self.end(ending_time=1.0)
        raise SystemExit(0)

    def done(self, mode: str = "all") -> bool | None:
        """
        Check if the simulation should terminate based on robot completion status.

        This method evaluates whether robots in the environment have reached their
        goals or completed their tasks, using different criteria based on the mode.

        Args:
            mode (str): Termination condition mode. Options are:

                - "all": Simulation is done when ALL robots have completed their tasks
                - "any": Simulation is done when ANY robot has completed its task

                Default is "all".

        Returns:
            bool: True if the termination condition is met based on the specified mode,
            False otherwise. Returns False if no robots are present in the environment.

        Example:
            >>> # Check if all robots have reached their goals
            >>> if env.done(mode="all"):
            ...     print("All robots completed!")
            >>>
            >>> # Check if any robot has completed
            >>> if env.done(mode="any"):
            ...     print("At least one robot completed!")
        """

        done_list = [obj.done() for obj in self.objects if obj.role == "robot"]

        if len(done_list) == 0:
            return False

        if mode == "all":
            return all(done_list)
        if mode == "any":
            return any(done_list)
        return None

    def _status_step(self) -> None:
        """
        Update and log the current status of all robots in the environment.

        This method checks the arrival status of all robots and logs information
        about which robots have reached their goals. It's automatically called
        during each simulation step.

        Note:
            This is an internal method primarily used for status tracking and logging.
            The status information is automatically updated during simulation steps.
        """

        # object status step
        [obj.check_status() for obj in self.objects]

        arrive_list = [obj.arrive for obj in self.objects if obj.role == "robot"]
        collision_list = [obj.collision for obj in self.objects if obj.role == "robot"]

        if len(arrive_list) == 0:
            arrive_list = [False]
        if len(collision_list) == 0:
            collision_list = [False]

        if all(arrive_list):
            self.set_status("Arrived")
        elif any(collision_list):
            self.set_status("Collision")
        elif self.reset_flag:
            self.set_status("Reset")
        elif self.reload_flag:
            self.set_status("Reload")
        elif self.save_figure_flag:
            self.set_status("Save Figure")
        elif self.quit_flag:
            self.set_status("Quit")
        elif self.debug_flag:
            self.set_status("Pause (Debugging)")
        else:
            if self._world_param.control_mode == "keyboard":
                self.set_status("Running (keyboard)")
            else:
                self.set_status("Running")

    def pause(self) -> None:
        """
        Pause the simulation execution.

        When paused, calls to :py:meth:`step` will return immediately without
        performing any simulation updates. The environment status is set to "Pause".

        Example:
            >>> env.pause()
            >>> env.step([1.0, 0.0])  # This will have no effect while paused
        """
        self.set_status("Pause")
        self.pause_flag = True

    def resume(self) -> None:
        """
        Resume the simulation execution after being paused.

        Re-enables simulation updates and sets the environment status back to "Running".
        Subsequent calls to :py:meth:`step` will function normally.

        Example:
            >>> env.pause()
            >>> # ... some time later ...
            >>> env.resume()
            >>> env.step([1.0, 0.0])  # This will now work again
        """
        self.set_status("Running")
        if self.pause_flag:
            self.pause_flag = False

        if self.debug_flag:
            self.debug_flag = False
            self.debug_count = 0

    def reset(self, random: bool = False) -> None:
        """
        Reset the environment to its initial state.

        This method resets all objects, robots, obstacles, and the world to their
        initial configurations. It also resets the visualization and sets the
        environment status to "Reset".

        The reset process includes:
        - Resetting all objects to their initial positions and states
        - Clearing accumulated trajectories and sensor data
        - Resetting the world timer and status
        - Refreshing the visualization plot

        Args:
            random (bool): If True, rebuild the world from the cached YAML
                parse (the config loaded at env creation, not the file on
                disk) so any randomized elements (e.g. ``distribution:
                random``, random shape generators) are re-sampled from the
                current RNG state. Call :func:`irsim.util.random.set_seed`
                before ``reset(random=True)`` to get a reproducible fresh
                scene. Default is False — only restores original initial
                states.

        Example:
            >>> # Reset environment after simulation
            >>> env.reset()
            >>> # Reset and re-sample random distributions / shapes
            >>> env.reset(random=True)
        """

        if random:
            self._rebuild_from_cached_parse()
            return

        self._reset_all()
        self._world.reset()
        self.refresh()
        self.reset_plot()
        self.set_status("Reset")
        self.pause_flag = False
        self.debug_flag = False
        self.debug_count = 0
        self.reset_flag = False

    def _reset_all(self) -> None:
        [obj.reset() for obj in self.objects]

    def refresh(self) -> None:
        """
        Refresh state-derived attributes across the environment without
        advancing the simulation.

        Calls ``ObjectBase.refresh`` on every object (geometry, geometry
        validity, sensor readings), rebuilds the collision STRtree, and
        re-evaluates collision/arrival status. Used after ``reset`` and
        whenever callers mutate object states directly and need derived
        views (geometry, sensors, collisions) brought up to date.
        """

        self._objects_refresh()
        self._objects_sensor_step()
        self._status_step()

    @plot_only
    def reset_plot(self) -> None:
        """
        Reset the environment figure in-place.

        Re-initializes drawing on the current figure/axes using the existing
        ``EnvPlot`` instance; does not create a new figure window.
        """

        self._env_plot.clear_components("all", self.objects)
        self._env_plot._init_plot(self._world, self.objects)

    # region: environment change
    def random_obstacle_position(
        self,
        range_low: list[float] | np.ndarray | None = None,
        range_high: list[float] | np.ndarray | None = None,
        ids: list[int] | None = None,
        non_overlapping: bool = False,
    ) -> None:
        """
        Random obstacle positions in the environment.

        Args:
            range_low (list [x, y, theta]): Lower bound of the random range for the obstacle states. Default is [0, 0, -3.14].
            range_high (list [x, y, theta]): Upper bound of the random range for the obstacle states. Default is [10, 10, 3.14].
            ids (list): A list of IDs of objects for which to set random positions. Default is None.
            non_overlapping (bool): If set, the obstacles that will be reset to random obstacles will not overlap with other obstacles. Default is False.
        """

        range_low = to_numpy(range_low, np.array([0, 0, -3.14]), (3, 1))
        range_high = to_numpy(range_high, np.array([10, 10, 3.14]), (3, 1))

        if ids is None:
            ids = [obs.id for obs in self.obstacle_list]

        selected_obs = [obs for obs in self.obstacle_list if obs.id in ids]
        existing_obj = [obj for obj in self.objects if obj.id not in ids]

        for obj in selected_obs:
            if not non_overlapping:
                obj.set_state(rng.uniform(range_low, range_high, (3, 1)), init=True)
            else:
                counter = 0

                while counter < 100:
                    obj.set_state(rng.uniform(range_low, range_high, (3, 1)), init=True)

                    if any(obj.check_collision(exi_obj) for exi_obj in existing_obj):
                        counter += 1
                    else:
                        existing_obj.append(obj)
                        break

        if self._env_plot is not None:
            self._env_plot.step("all", self.obstacle_list)

    def random_polygon_shape(
        self,
        center_range: list[float] | None = None,
        avg_radius_range: list[float] | None = None,
        irregularity_range: list[float] | None = None,
        spikeyness_range: list[float] | None = None,
        num_vertices_range: list[int] | None = None,
    ) -> None:
        """
        Random polygon shapes for the obstacles in the environment.

        Args:
            center_range (list): Range of the center of the polygon. Default is [0, 0, 10, 10].
            avg_radius_range (list): Range of the average radius of the polygon. Default is [0.1, 1].
            irregularity_range (list): Range of the irregularity of the polygon. Default is [0, 1].
            spikeyness_range (list): Range of the spikeyness of the polygon. Default is [0, 1].
            num_vertices_range (list): Range of the number of vertices of the polygon. Default is [4, 10].

            center (Tuple[float, float]):
                a pair representing the center of the circumference used
                to generate the polygon.
            avg_radius (float):
                the average radius (distance of each generated vertex to
                the center of the circumference) used to generate points
                with a normal distribution.
            irregularity (float): 0 - 1
                variance of the spacing of the angles between consecutive
                vertices.
            spikeyness (float): 0 - 1
                variance of the distance of each vertex to the center of
                the circumference.
            num_vertices (int):
                the number of vertices of the polygon.
        """

        if num_vertices_range is None:
            num_vertices_range = [4, 10]
        if spikeyness_range is None:
            spikeyness_range = [0, 1]
        if irregularity_range is None:
            irregularity_range = [0, 1]
        if avg_radius_range is None:
            avg_radius_range = [0.1, 1]
        if center_range is None:
            center_range = [0, 0, 10, 10]
        vertices_list = random_generate_polygon(
            self.obstacle_number,
            center_range,
            avg_radius_range,
            irregularity_range,
            spikeyness_range,
            num_vertices_range,
        )

        for i, obj in enumerate(self.obstacle_list):
            if obj.shape == "polygon":
                geom = Polygon(vertices_list[i])
                obj.set_original_geometry(geom)

        if self._env_plot is not None:
            self._env_plot.step("all", self.obstacle_list)

    def reload(self, world_name: str | None = None) -> None:
        """
        Reload the environment from YAML and update the current figure.

        This re-parses the YAML and re-creates world/objects, then refreshes
        drawing on the existing figure/axes (no new window is created).

        Args:
            world_name (str): Optional name/path of the world YAML to reload.
                If ``None``, the previous YAML file is used.
        """
        ObjectBase.reset_id_iter()
        self.reset()
        if self._env_plot is not None:
            self._env_plot.clear_components("all", self.objects)
        (
            self._world,
            self._objects,
            self._env_plot,
            self._robot_collection,
            self._obstacle_collection,
            self._map_collection,
            self._object_groups,
        ) = self.env_config.reload_yaml_objects(world_name)
        # Wire env reference to newly created objects
        self._wire_env_to_objects()
        self.build_tree()
        self.validate_unique_names()
        self._env_param.objects = self._objects
        self.reload_flag = False

    def _rebuild_from_cached_parse(self) -> None:
        """Rebuild world and objects from the cached YAML parse.

        Used by ``reset(random=True)`` to re-sample randomized elements
        without re-reading the YAML file from disk (which would pick up
        any on-disk edits since the environment was created). Old objects
        are discarded, so no per-object reset or simulation step is run
        beforehand — those would waste work and could consume RNG draws
        (e.g. random behaviors), breaking reproducibility of the
        re-sampled scene.
        """
        ObjectBase.reset_id_iter()
        (
            self._world,
            self._objects,
            self._env_plot,
            self._robot_collection,
            self._obstacle_collection,
            self._map_collection,
            self._object_groups,
        ) = self.env_config.reload_objects()
        self._wire_env_to_objects()
        self.build_tree()
        self.validate_unique_names()
        self._env_param.objects = self._objects
        self.set_status("Reset")
        self.pause_flag = False
        self.debug_flag = False
        self.debug_count = 0
        self.reset_flag = False

    # endregion: environment change

    # region: object operation

    def create_obstacle(self, **kwargs: Any):
        """
        Create an obstacle in the environment.

        Args:
            **kwargs: Additional parameters for obstacle creation.
                see ObjectFactory.create_obstacle for detail

        Returns:
            Obstacle: An instance of an obstacle.
        """

        return self.object_factory.create_obstacle(**kwargs)

    def create_robot(self, **kwargs: Any):
        """
        Create a robot in the environment.

        Args:
            **kwargs: Additional parameters for robot creation.
                see ObjectFactory.create_robot for detail

        Returns:
            Robot: An instance of a robot.
        """

        return self.object_factory.create_robot(**kwargs)

    def add_object(self, obj: ObjectBase) -> None:
        """
        Add the object to the environment, enforcing unique names.

        Args:
            obj (ObjectBase): The object to be added to the environment.
        """
        if find_object_by_identity(self.objects, obj.name) is not None:
            raise ValueError(f"Object name '{obj.name}' already exists.")
        obj._env = self
        self._objects.append(obj)
        if not self.headless:
            obj._init_plot(self._env_plot.ax)
            obj._step_plot()
        self.build_tree()

    def add_objects(self, objs: list[ObjectBase]) -> None:
        """
        Add the objects to the environment, enforcing unique names (both within
        the new list and against existing objects).

        Args:
            objs (list): List of objects to be added to the environment.
        """
        new_names = [o.name for o in objs]
        dupes_in_new = find_duplicates(new_names)
        if dupes_in_new:
            raise ValueError(f"Duplicate names within new objects: {dupes_in_new}")
        existing_names = {o.name for o in self.objects}
        conflicts = [n for n in new_names if n in existing_names]
        if conflicts:
            raise ValueError(f"Object names already exist: {conflicts}")
        for obj in objs:
            obj._env = self
            if not self.headless:
                obj._init_plot(self._env_plot.ax)
                obj._step_plot()
        self._objects.extend(objs)
        self.build_tree()

    def delete_object(self, target_id: int) -> None:
        """
        Delete the object with the given id.

        Args:
            target_id (int): ID of the object to be deleted.
        """

        target = find_object_by_identity(self._objects, object_id=target_id)

        if target is not None:
            target.plot_clear()
            self._objects.remove(target)

        self.build_tree()

    def delete_objects(self, target_ids: list[int]) -> None:
        """
        Delete the objects with the given ids.

        Args:
            target_ids (list): List of IDs of objects to be deleted.
        """

        del_obj = [obj for obj in self._objects if obj.id in target_ids]

        for obj in del_obj:
            obj.plot_clear()
            self._objects.remove(obj)

        self.build_tree()

    def build_tree(self) -> None:
        """
        Build the geometry tree for the objects in the environment to detect the possible collision objects.
        """

        self._env_param.GeometryTree = STRtree([obj.geometry for obj in self.objects])

    def validate_unique_names(self) -> None:
        """Validate that all object names are unique.

        Raises:
            ValueError: If duplicates exist.
        """
        duplicates = find_duplicates(obj.name for obj in self.objects)
        if duplicates:
            raise ValueError(f"Duplicate object names: {duplicates}")

    # endregion: object operation

    # region: get information

    def get_robot_state(self) -> np.ndarray:
        """
        Get the current state of the robot.

        Returns:
            np.ndarray: Current robot state as a column vector. Differential and
            omnidirectional robots return ``[x, y, theta]``; Ackermann robots return
            ``[x, y, theta, steer]``.

        """

        return self.robot._state

    def get_msg(self) -> WorldState:
        """Get a snapshot of the complete simulation environment.

        The returned message owns copies of all state and sensor arrays. It
        therefore remains a stable point-in-time record when the environment
        advances or its objects are modified later.

        Returns:
            WorldState: Current world metadata with ``odom`` and
            ``scan`` messages for each object.

        Example:
            >>> msg = env.get_msg()
            >>> msg.header.stamp
            0.0
            >>> msg.robots[0].odom.pose.pose.position.x
            1.0
            >>> msg.robots[0].scan.ranges
            array([...])
        """

        return WorldState.from_env(self)

    def receive_msg(
        self,
        msg: WorldState | ObjectState | Any,
        *,
        object_name: str | None = None,
        object_id: int | None = None,
        refresh: bool = True,
    ) -> int:
        """Apply odometry messages to simulation objects.

        ``WorldState`` updates every contained object, while ``ObjectState``
        uses its embedded name and id. A standalone IR-SIM or native ROS
        ``Odometry`` message targets the primary robot unless ``object_name``
        or ``object_id`` is supplied. World/object metadata, goals, scans, and
        the simulation clock remain owned by the receiving environment.

        Object poses are matched by name first and id second. The incoming
        planar pose and body-frame twist are converted to the target object's
        local state and velocity layout. All messages are validated before any
        object is changed, so a failed receive does not partially update the
        environment.

        An Ackermann object carries its steering as a yaw rate, which vanishes
        at zero speed: a stopped car conveys no steering angle, so the local
        object keeps the one it already had.

        Args:
            msg: A :class:`~irsim.msg.WorldState`,
                :class:`~irsim.msg.ObjectState`, or ROS-compatible
                ``nav_msgs/Odometry`` object.
            object_name: Explicit target name for a standalone odometry or
                object message.
            object_id: Explicit target object id for a standalone odometry or
                object message. Names take precedence when both are provided.
            refresh: Recompute geometry, sensors, collision tree, and status
                after applying all updates. Set ``False`` when batching manual
                changes and call :meth:`refresh` afterwards. Default is True.

        Returns:
            int: Number of simulation objects updated.

        Raises:
            TypeError: If ``msg`` is not a supported message shape.
            ValueError: If a target cannot be found, a target appears more
                than once, or odometry contains an invalid planar pose.

        Example:
            >>> external_odom = source_env.get_msg().robots[0].odom
            >>> target_env.receive_msg(
            ...     external_odom, object_name="message_robot"
            ... )
            1
        """

        updates = resolve_message_targets(
            self.objects,
            msg,
            object_name,
            object_id,
            default_target=lambda: self.robot,
        )
        prepared = [
            (target, *Odometry.from_msg(odom).to_state_velocity(target))
            for target, odom in updates
        ]

        for target, state, velocity in prepared:
            target.set_state(state)
            target.set_velocity(velocity)

        if refresh and prepared:
            self.refresh()

        return len(prepared)

    def get_lidar_scan(self, id: int = 0) -> dict[str, Any]:
        """
        Get the LiDAR scan of the robot with the given id.

        Args:
            id (int): Id of the robot.

        Returns:
            Dict: Dict of lidar scan points, see :py:meth:`.world.sensors.lidar2d.Lidar2D.get_scan` for detail.
        """

        return self.robot_list[id].get_lidar_scan()

    def get_lidar_offset(self, id: int = 0) -> list[float]:
        """
        Get the LiDAR offset of the robot with the given id.


        Args:
            id (int): Id of the robot.

        Returns:
            list of float: Lidar offset of the robot, [x, y, theta]
        """

        return self.robot_list[id].get_lidar_offset()

    def get_obstacle_info_list(self) -> list[Any]:
        """
        Get the information of the obstacles in the environment.

        Returns:
            list of ObstacleInfo: List of obstacle information, see :py:meth:`.ObjectBase.get_obstacle_info` for detail.
        """

        return [obj.get_obstacle_info() for obj in self.obstacle_list]

    def get_robot_info(self, id: int = 0) -> Any:
        """
        Get the information of the robot with the given id.

        Args:
            id (int): Id of the robot.

        Returns:
            see :py:meth:`.ObjectBase.get_info` for detail
        """

        return self.robot_list[id].get_info()

    def get_robot_info_list(self) -> list[Any]:
        """
        Get the information of the robots in the environment.

        Returns:
            list of ObjectInfo: List of robot information, see :py:meth:`.ObjectBase.get_info` for detail.
        """

        return [obj.get_info() for obj in self.robot_list]

    def get_map(self, resolution: float = 0.1) -> Any:
        """
        Get the map of the environment with the given resolution.

        Args:
            resolution (float): Resolution of the map. Default is 0.1.

        Returns:
            The map of the environment with the specified resolution.
        """
        return self._world.get_map(resolution, self.obstacle_list)

    def get_group_by_name(self, group_name: str) -> list[ObjectBase]:
        """
        Get the objects with the given group name.

        Args:
            group_name (str): Group name of the robot.

        Returns:
            list[ObjectBase]: The object list with the given group name.
        """
        return [obj for obj in self.objects if obj.group_name == group_name]

    def get_object_by_name(self, name: str) -> ObjectBase | None:
        """
        Get the object with the given name.
        """
        return find_object_by_identity(self.objects, name=name)

    def get_object_by_id(self, target_id: int) -> ObjectBase | None:
        """
        Get the object with the given id.
        """
        return find_object_by_identity(self.objects, object_id=target_id)

    # endregion: get information

    @plot_only
    def set_title(self, title: str) -> None:
        """
        Set the title of the plot.
        """

        self._env_plot.title = title

    def set_random_seed(self, seed: int | None = None, reload: bool = False) -> None:
        """
        Set IR-SIM's random seed for reproducibility.

        Args:
            seed (int, optional): Seed for IR-SIM's project RNG. If ``None``, a
                new unseeded generator is created (non-reproducible). This
                controls randomness that goes through IR-SIM's RNG. Custom
                code using ``np.random.*`` or Python ``random`` must be
                seeded separately or migrated to use IR-SIM's RNG.
            reload (bool): If True, reload the environment to regenerate
                random obstacles with the new seed. Default is False (only
                sets seed).

        Example:
            >>> env.set_random_seed(100)  # Only set seed, no regeneration
            >>> env.set_random_seed(100, reload=True)  # Set seed and regenerate env by yaml file
        """
        set_seed(seed)
        if reload:
            self.reload()

    def set_status(self, status: str) -> None:
        """
        Set the status of the environment.
        """
        self._world.status = status

    @plot_only
    def save_figure(
        self,
        save_name: str | None = None,
        include_index: bool = False,
        save_gif: bool = False,
        **kwargs: Any,
    ) -> None:
        """
        Save the current figure.

        Args:
            save_name (str): Name of the file with format to save the figure. Default is None.
            include_index (bool): Flag to include index in the saved file name. Default is False.
            save_gif (bool): Flag to save as GIF format. Default is False.
            **kwargs: Additional keyword arguments for saving the figure, see `savefig <https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html>`__ function for detail.
        """
        file_save_name = save_name or self._world.name + ".png"

        if "." not in file_save_name:
            file_name = file_save_name
            file_format = "png"
        else:
            file_name, file_format = file_save_name.rsplit(".", 1)

        self._env_plot.save_figure(
            file_name, file_format, include_index, save_gif, **kwargs
        )

    def load_behavior(self, behaviors: str = "behavior_methods") -> None:
        """
        Load behavior parameters from the script. Please refer to the behavior_methods.py file for more details.
        Please make sure the python file is placed in the same folder with the implemented script.

        This method imports the specified module and reinitializes all behaviors
        (both individual and group) so that newly registered behaviors are available.

        Args:
            behaviors (str): name of the behavior script.
        """

        try:
            importlib.import_module(behaviors)
        except ImportError as e:
            self.logger.error(f"Failed to load module '{behaviors}': {e}")
            return

        # Reinitialize individual behaviors for all objects
        for obj in self.objects:
            if hasattr(obj, "obj_behavior") and obj.obj_behavior is not None:
                obj.obj_behavior._init_behavior_class()

        # Reinitialize group behaviors for all object groups
        for group in self._object_groups:
            if hasattr(group, "group_behavior") and group.group_behavior is not None:
                group.group_behavior._init_group_behavior_class()

    # region: property
    @property
    def robot_list(self) -> list[ObjectBase]:
        """
        Get the list of robots in the environment.

        Returns:
            list: List of robot objects [].
        """

        return [obj for obj in self.objects if obj.role == "robot"]

    @property
    def obstacle_list(self) -> list[ObjectBase]:
        """
        Get the list of obstacles in the environment.

        Returns:
            list: List of obstacle objects.
        """
        return [obj for obj in self.objects if obj.role == "obstacle"]

    @property
    def objects(self) -> list[ObjectBase]:
        """
        Get all objects in the environment.

        Returns:
            list: List of all objects in the environment.
        """
        return self._objects

    @property
    def static_objects(self) -> list[ObjectBase]:
        """
        Get all static objects in the environment.

        Returns:
            list: List of static objects in the environment.
        """
        return [obj for obj in self.objects if obj.static]

    @property
    def dynamic_objects(self) -> list[ObjectBase]:
        """
        Get all dynamic objects in the environment.

        Returns:
            list: List of dynamic objects in the environment.
        """
        return [obj for obj in self.objects if not obj.static]

    @property
    def disable_all_plot(self) -> bool:
        """Alias of :attr:`headless`, kept for backward compatibility."""
        return self.headless

    @property
    def step_time(self) -> float:
        """
        Get the step time of the simulation.

        Returns:
            float: Step time of the simulation from the world.
        """
        return self._world.step_time

    @property
    def step_mode(self) -> str:
        """Get the active state-advancement mode."""
        return self._world_param.step_mode

    @property
    def world_param(self):
        """
        Get the world parameters of the simulation.

        Returns:
            WorldParam: World parameters including time, control_mode,
                collision_mode, step_mode, step_time, and count.
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
    def path_param(self):
        """
        Get the path manager for the simulation.

        Returns:
            PathManager: Path manager including root_path, ani_buffer_path,
                ani_path, and fig_path.
        """
        return self._path_manager

    @property
    def config(self) -> dict[str, Any]:
        """
        Get the parsed YAML configuration of the environment.

        The sections are returned as they were read, so a value can be looked
        up without opening the file again. ``custom`` holds whatever the
        scenario defines for its own use; IR-SIM stores it and nothing more.

        Returns:
            dict: Parsed configuration, keyed by section - ``world``, ``gui``,
            ``robot``, ``obstacle`` and ``custom``.

        Example:
            >>> env.config["world"]["step_time"]
            0.1
            >>> env.config["custom"]["safe_margin"]
            0.15
        """
        return self.env_config.parse

    @property
    def time(self) -> float:
        """
        Get the time of the simulation.
        """
        return self._world.time

    @property
    def status(self) -> str:
        """
        Get the status of the environment.
        """
        return self._world.status

    @property
    def robot(self) -> ObjectBase:
        """
        Get the first robot in the environment.

        Returns:
            Robot: The first robot object in the robot list.

        Raises:
            IndexError: If no robots exist in the environment.
        """
        if not self.robot_list:
            raise IndexError("No robots in the environment. Add a robot first.")
        return self.robot_list[0]

    @property
    def obstacle_number(self) -> int:
        """
        Get the number of obstacles in the environment.

        Returns:
            int: Number of obstacles in the environment.
        """
        return len(self.obstacle_list)

    @property
    def robot_number(self) -> int:
        """
        Get the number of robots in the environment.

        Returns:
            int: Number of robots in the environment.
        """
        return len(self.robot_list)

    @property
    def logger(self) -> EnvLogger:
        """
        Get the environment logger.

        Returns:
            EnvLogger: The logger instance for the environment.
        """
        return cast(EnvLogger, self._env_param.logger)

    @property
    def key_vel(self) -> Any:
        """Get current keyboard velocity command.

        Returns:
            Any: A 3x1 vector ``[[linear], [lateral], [angular]]`` from keyboard input.
        """
        return self.keyboard.key_vel

    @property
    def key_id(self) -> int:
        """Get current keyboard-controlled robot id.

        Returns:
            int: The robot id currently controlled by keyboard.
        """
        return self.keyboard.key_id

    @property
    def mouse_pos(self) -> Any:
        """Get current mouse position on the canvas.

        Returns:
            Any: Mouse coordinates ``(x, y)`` or ``None`` if outside axes.
        """
        return self.mouse.mouse_pos

    @property
    def mouse_left_pos(self) -> Any:
        """Get last left-click position.

        Returns:
            Any: Position array or ``None`` if not set.
        """
        return self.mouse.left_click_pos

    @property
    def mouse_right_pos(self) -> Any:
        """Get last right-click position.

        Returns:
            Any: Position array or ``None`` if not set.
        """
        return self.mouse.right_click_pos

    @property
    def names(self) -> list[str]:
        """Get the names of all objects in the environment."""
        return [obj.name for obj in self.objects]

    @property
    def object_factory(self) -> ObjectFactory:
        """Get the object factory of the environment."""
        return self.env_config.object_factory

    # endregion: property
