"""
Class EnvBase is the base class of the environment.

It loads YAML configuration via ``EnvConfig`` to construct the world,
robots, obstacles, and maps, and provides the core simulation loop.
The environment can be reconfigured in-place (same figure) by reloading
the YAML at runtime.

Author: Ruihua Han
"""

from __future__ import annotations

import importlib
from collections import Counter
from typing import Any, Optional, Union

import matplotlib
import numpy as np
from matplotlib import pyplot as plt
from shapely import Polygon
from shapely.strtree import STRtree

from irsim.config import env_param, world_param
from irsim.env.env_config import EnvConfig
from irsim.gui.mouse_control import MouseControl
from irsim.lib import random_generate_polygon
from irsim.world import ObjectBase, ObjectFactory

from .env_logger import EnvLogger

try:
    from irsim.gui.keyboard_control import KeyboardControl

    keyboard_module = True
except ImportError:
    keyboard_module = False

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
backends = BACKEND_PREFERENCES.get(env_param.platform_name, ["Agg"])
backend_set = _set_matplotlib_backend(backends)


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
        disable_all_plot (bool): Whether to disable all plots and figures completely.
            When True, no visualization will be created even if display is True.
            Default is False.
        save_ani (bool): Whether to save the simulation as an animation file.
            Useful for creating videos of simulation runs. Default is False.
        full (bool): Whether to display the visualization in full screen mode.
            Only effective on supported platforms. Default is False.
        log_file (str, optional): Path to the log file for saving simulation logs.
            If None, logs will only be output to console.
        log_level (str): Logging level for the environment. Options include
            'DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'. Default is 'INFO'.

    Attributes:
        display (bool): Display flag for the environment.
        disable_all_plot (bool): Plot disable flag.
        save_ani (bool): Animation saving flag.
        objects (list): List of all objects in the environment.
        world (World): The world object containing environment configuration.
        robot_collection (list): List of robot objects.
        obstacle_collection (list): List of obstacle objects.
        map_collection (list): List of map objects.

    Example:
        >>> # Create a basic environment
        >>> env = EnvBase("my_world.yaml")
        >>>
        >>> # Create headless environment for training
        >>> env = EnvBase("world.yaml", display=False, log_level="WARNING")
        >>>
        >>> # Create environment with animation saving
        >>> env = EnvBase("world.yaml", save_ani=True, full=True)
    """

    def __init__(
        self,
        world_name: Optional[str] = None,
        display: bool = True,
        disable_all_plot: bool = False,
        save_ani: bool = False,
        full: bool = False,
        log_file: Optional[str] = None,
        log_level: str = "INFO",
    ) -> None:
        # init env setting
        self.display = display

        if not self.display:
            matplotlib.use("Agg")

        self.disable_all_plot = disable_all_plot
        self.save_ani = save_ani

        env_param.logger = EnvLogger(log_file, log_level)

        self.env_config = EnvConfig(world_name)

        (
            self._world,
            self._objects,
            self._env_plot,
            self._robot_collection,
            self._obstacle_collection,
            self._map_collection,
        ) = self.env_config.initialize_objects()

        self.build_tree()
        env_param.objects = self._objects
        self.validate_unique_names()

        # Try to initialize keyboard control (pynput or MPL backend inside KeyboardControl)
        try:
            keyboard_config = self.env_config.parse["gui"].get("keyboard", {})
            self.keyboard = KeyboardControl(env_ref=self, **keyboard_config)
        except Exception as e:
            self.logger.error(
                f"Keyboard control unavailable error: {e}. Auto control applied. "
                "Install 'pynput' or set backend='mpl' in YAML keyboard config."
            )
            world_param.control_mode = "auto"

        mouse_config = self.env_config.parse["gui"].get("mouse", {})
        self.mouse = MouseControl(self._env_plot.ax, **mouse_config)

        # flag
        self.pause_flag = False
        self.quit_flag = False

        if full:
            mng = plt.get_current_fig_manager()
            if mng is not None:
                mng.full_screen_toggle()

        # Log simulation start
        self.logger.info(
            f"Simulation environment '{self._world.name}' has been initialized and started."
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

    def step(
        self,
        action: Optional[Union[np.ndarray, list[Any]]] = None,
        action_id: Optional[Union[int, list[int]]] = 0,
    ) -> None:
        """
        Perform a single simulation step in the environment.

        This method advances the simulation by one time step, applying the given actions
        to the specified robots and updating all objects in the environment.

        Args:
            action (Union[np.ndarray, list], optional): Action(s) to be performed in the environment.
                Can be a single action or a list of actions. Action format depends on robot type:

                - **Differential robot**: [linear_velocity, angular_velocity]
                - **Omnidirectional robot**: [velocity_x, velocity_y]
                - **Ackermann robot**: [linear_velocity, steering_angle]

                If None, robots will use their default behavior or keyboard control if enabled.

                Note - Priority Order:
                    1. perform the keyboard control for a specific action_id if enabled.
                    2. perform the action argument as a list of numpy array, and the action_id argument as a list of int.
                    3. perform the default behavior for the rest of the objects if the action is None.

            action_id (Union[int, list], optional): ID(s) of the robot(s) to apply the action(s) to.
                Can be a single robot ID or a list of IDs. Default is 0 (first robot).
                If action is a list and action_id is a single int, all actions will be
                applied to robots sequentially starting from action_id.

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
        """

        if self.quit_flag:
            self.quit()

        if self.pause_flag:
            return

        actions = [None] * len(self.objects)

        if action is not None:
            if isinstance(action, list):
                if isinstance(action_id, list):
                    for a, ai in zip(action, action_id):
                        actions[ai] = a
                else:
                    actions[int(action_id) : int(action_id) + len(action)] = action[:]

            elif isinstance(action, np.ndarray):
                if isinstance(action_id, list):
                    for ai in action_id:
                        actions[ai] = action
                else:
                    actions[int(action_id)] = action

        if world_param.control_mode == "keyboard":
            actions[self.key_id] = self.key_vel

        self._objects_step(actions, sensor_step=False)
        self._objects_sensor_step()
        self._world.step()
        self._status_step()

    def _objects_step(self, action: list[Any], sensor_step: bool = True) -> None:
        """Advance all objects by one step with corresponding actions.

        Args:
            action (list[Any]): A list of actions aligned with ``self.objects``.
                If the list is shorter than the number of objects, it is padded
                with ``None`` for the remaining objects.
        """
        action = action + [None] * (len(self.objects) - len(action))
        [obj.step(action, sensor_step) for obj, action in zip(self.objects, action)]

        self.build_tree()

    def _objects_sensor_step(self) -> None:
        """step the sensors of all objects with updated states"""
        [obj.sensor_step() for obj in self.objects]

    def _object_step(
        self, action: np.ndarray | list[Any] | None, obj_id: int = 0
    ) -> None:
        """Advance a single object by one step and tick others.
        No use this function. It will be deprecated in the future.

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

    # render
    def render(
        self,
        interval: float = 0.01,
        figure_kwargs: Optional[dict[str, Any]] = None,
        mode: str = "dynamic",
        **kwargs: Any,
    ) -> None:
        """
        Render the environment.

        Args:
            interval(float) :  Time interval between frames in seconds.
            figure_kwargs(dict) : Additional keyword arguments for saving figures, see `savefig <https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html>`_ for detail.
            mode(str) : "dynamic", "static", "all" to specify which type of objects to draw and clear.
            kwargs: Additional keyword arguments for drawing components. see :py:meth:`.ObjectBase.plot` function for detail.
        """

        if figure_kwargs is None:
            figure_kwargs = {}
        if not self.disable_all_plot and self._world.sampling:
            if self.display:
                plt.pause(interval)

            if self.save_ani:
                self.save_figure(save_gif=True, **figure_kwargs)

            self._env_plot.step(mode, self.objects, **kwargs)

    def show(self) -> None:
        """
        Show the environment figure.
        """

        self._env_plot.show()

    # draw various components
    def draw_trajectory(
        self, traj: list[Any], traj_type: str = "g-", **kwargs: Any
    ) -> None:
        """
        Draw the trajectory on the environment figure.

        Args:
            traj (list): List of trajectory points (2 * 1 vector).
            traj_type: Type of the trajectory line, see matplotlib plot function for detail.
            **kwargs: Additional keyword arguments for drawing the trajectory, see :py:meth:`.EnvPlot.draw_trajectory` for detail.
        """

        self._env_plot.draw_trajectory(traj, traj_type, **kwargs)

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
            points (list): List of points (2*1) to be drawn.
                or (np.array): (2, Num) to be drawn.
            s (int): Size of the points.
            c (str): Color of the points.
            refresh (bool): Flag to refresh the points in the figure.
            **kwargs: Additional keyword arguments for drawing the points, see `ax.scatter <https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.scatter.html>`_ function for detail.
        """

        self._env_plot.draw_points(points, s, c, refresh, **kwargs)

    def draw_box(
        self, vertex: np.ndarray, refresh: bool = False, color: str = "-b"
    ) -> None:
        """
        Draw a box by the vertices.

        Args:
            vertex (np.ndarray): matrix of vertices, point_dim*vertex_num
            refresh (bool): whether to refresh the plot, default True
            color (str): color of the box, default '-b'
        """
        self._env_plot.draw_box(vertex, refresh, color)

    def draw_quiver(self, point: Any, refresh: bool = False, **kwargs: Any) -> None:
        """
        Draw a single quiver (arrow) on the environment figure.

        Args:
            point: Point data for the quiver
            refresh (bool): Flag to refresh the quiver in the figure, default False
            **kwargs: Additional keyword arguments for drawing the quiver
        """
        self._env_plot.draw_quiver(point, refresh, **kwargs)

    def draw_quivers(self, points: Any, refresh: bool = False, **kwargs: Any) -> None:
        """
        Draw multiple quivers (arrows) on the environment figure.

        Args:
            points: Points data for the quivers
            refresh (bool): Flag to refresh the quivers in the figure, default False
            **kwargs: Additional keyword arguments for drawing the quivers
        """
        self._env_plot.draw_quivers(points, refresh, **kwargs)

    def end(self, ending_time: float = 3.0, **kwargs: Any) -> None:
        """
        End the simulation, save the animation, and close the environment.

        Args:
            ending_time (float): Time in seconds to wait before closing the figure, default is 3 seconds.
            **kwargs: Additional keyword arguments for saving the animation, see :py:meth:`.EnvPlot.save_animate` for detail.
        """

        if self.disable_all_plot:
            return

        if self.save_ani:
            self._env_plot.save_animate(**kwargs)

        if self.display:
            plt.pause(ending_time)
            self.logger.info(f"Figure will be closed within {ending_time:.2f} seconds.")

        plt.close("all")
        env_param.objects = []
        ObjectBase.reset_id_iter()

        if hasattr(self, "keyboard"):
            # Stop pynput listener if present; otherwise disconnect MPL callbacks
            try:
                if (
                    hasattr(self.keyboard, "listener")
                    and self.keyboard.listener is not None
                ):
                    self.keyboard.listener.stop()
                else:
                    fig = plt.gcf()
                    if hasattr(self.keyboard, "_mpl_press_cid"):
                        fig.canvas.mpl_disconnect(self.keyboard._mpl_press_cid)
                    if hasattr(self.keyboard, "_mpl_release_cid"):
                        fig.canvas.mpl_disconnect(self.keyboard._mpl_release_cid)
            except Exception:
                pass

        self.logger.info(
            f"The simulated environment has ended. Total simulation time: {round(self._world.time, 2)} seconds."
        )

    def quit(self) -> None:
        """
        Quit the environment.
        """
        self.quit_flag = True
        self.logger.info("Quit the environment.")
        self.end(ending_time=1.0)
        raise SystemExit(0)

    def done(self, mode: str = "all") -> Optional[bool]:
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
            self._world.status = "Arrived"
        elif any(collision_list):
            self._world.status = "Collision"
        else:
            if world_param.control_mode == "keyboard":
                self._world.status = "Running (keyboard)"
            else:
                self._world.status = "Running"

    def pause(self) -> None:
        """
        Pause the simulation execution.

        When paused, calls to :py:meth:`step` will return immediately without
        performing any simulation updates. The environment status is set to "Pause".

        Example:
            >>> env.pause()
            >>> env.step([1.0, 0.0])  # This will have no effect while paused
        """
        self._world.status = "Pause"
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
        self._world.status = "Running"
        self.pause_flag = False

    def reset(self) -> None:
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

        Example:
            >>> # Reset environment after simulation
            >>> env.reset()
            >>> # Environment is now ready for a new simulation run
        """

        self._reset_all()
        self.step(action=np.zeros((2, 1)))
        self._world.reset()
        self.reset_plot()
        self._world.status = "Reset"
        self.pause_flag = False

    def _reset_all(self) -> None:
        [obj.reset() for obj in self.objects]

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
        range_low: Optional[list[float]] = None,
        range_high: Optional[list[float]] = None,
        ids: Optional[list[int]] = None,
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
        if range_high is None:
            range_high = [10, 10, 3.14]
        if range_low is None:
            range_low = [0, 0, -3.14]
        if ids is None:
            ids = [obs.id for obs in self.obstacle_list]
        if isinstance(range_low, list):
            range_low = np.c_[range_low]

        if isinstance(range_high, list):
            range_high = np.c_[range_high]

        selected_obs = [obs for obs in self.obstacle_list if obs.id in ids]
        existing_obj = [obj for obj in self.objects if obj.id not in ids]

        for obj in selected_obs:
            if not non_overlapping:
                obj.set_state(
                    np.random.uniform(range_low, range_high, (3, 1)), init=True
                )
            else:
                counter = 0

                while counter < 100:
                    obj.set_state(
                        np.random.uniform(range_low, range_high, (3, 1)), init=True
                    )

                    if any(obj.check_collision(exi_obj) for exi_obj in existing_obj):
                        counter += 1
                    else:
                        existing_obj.append(obj)
                        break

        self._env_plot.step("all", self.obstacle_list)

    def random_polygon_shape(
        self,
        center_range: Optional[list[float]] = None,
        avg_radius_range: Optional[list[float]] = None,
        irregularity_range: Optional[list[float]] = None,
        spikeyness_range: Optional[list[float]] = None,
        num_vertices_range: Optional[list[int]] = None,
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

        self._env_plot.step("all", self.obstacle_list)

    def reload(self, world_name: Optional[str] = None) -> None:
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
        self._env_plot.clear_components("all", self.objects)
        (
            self._world,
            self._objects,
            self._env_plot,
            self._robot_collection,
            self._obstacle_collection,
            self._map_collection,
        ) = self.env_config.reload_yaml_objects(world_name)
        self.build_tree()
        self.validate_unique_names()
        env_param.objects = self._objects

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

    def add_object(self, obj: ObjectBase) -> None:
        """
        Add the object to the environment, enforcing unique names.

        Args:
            obj (ObjectBase): The object to be added to the environment.
        """
        if any(existing.name == obj.name for existing in self.objects):
            raise ValueError(f"Object name '{obj.name}' already exists.")
        self._objects.append(obj)
        self.build_tree()

    def add_objects(self, objs: list[ObjectBase]) -> None:
        """
        Add the objects to the environment, enforcing unique names (both within
        the new list and against existing objects).

        Args:
            objs (list): List of objects to be added to the environment.
        """
        new_names = [o.name for o in objs]
        dupes_in_new = [n for n, c in Counter(new_names).items() if c > 1]
        if dupes_in_new:
            raise ValueError(f"Duplicate names within new objects: {dupes_in_new}")
        existing_names = {o.name for o in self.objects}
        conflicts = [n for n in new_names if n in existing_names]
        if conflicts:
            raise ValueError(f"Object names already exist: {conflicts}")
        self._objects.extend(objs)
        self.build_tree()

    def delete_object(self, target_id: int) -> None:
        """
        Delete the object with the given id.

        Args:
            target_id (int): ID of the object to be deleted.
        """

        for obj in self._objects:
            if obj.id == target_id:
                obj.plot_clear()
                self._objects.remove(obj)
                break

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

        env_param.GeometryTree = STRtree([obj.geometry for obj in self.objects])

    def validate_unique_names(self) -> None:
        """Validate that all object names are unique.

        Raises:
            ValueError: If duplicates exist.
        """
        names = [obj.name for obj in self.objects]
        duplicates = [n for n, c in Counter(names).items() if c > 1]
        if duplicates:
            raise ValueError(f"Duplicate object names: {duplicates}")

    # endregion: object operation

    # region: get information

    def get_robot_state(self) -> np.ndarray:
        """
        Get the current state of the robot.

        Returns:
            state: 3*1 vector [x, y, theta]

        """

        return self.robot._state

    def get_lidar_scan(self, id: int = 0) -> dict[str, Any]:
        """
        Get the lidar scan of the robot with the given id.

        Args:
            id (int): Id of the robot.

        Returns:
            Dict: Dict of lidar scan points, see :py:meth:`.world.sensors.lidar2d.Lidar2D.get_scan` for detail.
        """

        return self.robot_list[id].get_lidar_scan()

    def get_lidar_offset(self, id: int = 0) -> list[float]:
        """
        Get the lidar offset of the robot with the given id.


        Args:
            id (int): Id of the robot.

        Returns:
            list of float: Lidar offset of the robot, [x, y, theta]
        """

        return self.robot_list[id].get_lidar_offset()

    def get_obstacle_info_list(self) -> list[dict[str, Any]]:
        """
        Get the information of the obstacles in the environment.

        Returns:
            list of dict: List of obstacle information, see :py:meth:`.ObjectBase.get_obstacle_info` for detail.
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

    def get_robot_info_list(self) -> list[dict[str, Any]]:
        """
        Get the information of the robots in the environment.

        Returns:
            list of dict: List of robot information, see :py:meth:`.ObjectBase.get_info` for detail.
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

    def get_object_by_name(self, name: str) -> Optional[ObjectBase]:
        """
        Get the object with the given name.
        """
        return next((obj for obj in self.objects if obj.name == name), None)

    def get_object_by_id(self, target_id: int) -> Optional[ObjectBase]:
        """
        Get the object with the given id.
        """
        return next((obj for obj in self.objects if obj.id == target_id), None)

    # endregion: get information

    def set_title(self, title: str) -> None:
        """
        Set the title of the plot.
        """

        self._env_plot.title = title

    def save_figure(
        self,
        save_name: Optional[str] = None,
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
            **kwargs: Additional keyword arguments for saving the figure, see `savefig <https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html>`_ function for detail.
        """
        file_save_name = save_name or self._world.name + ".png"

        file_name, file_format = file_save_name.split(".")

        self._env_plot.save_figure(
            file_name, file_format, include_index, save_gif, **kwargs
        )

    def load_behavior(self, behaviors: str = "behavior_methods") -> None:
        """
        Load behavior parameters from the script. Please refer to the behavior_methods.py file for more details.
        Please make sure the python file is placed in the same folder with the implemented script.

        Args:
            behaviors (str): name of the bevavior script.
        """

        try:
            importlib.import_module(behaviors)
        except ImportError as e:
            print(f"Failed to load module '{behaviors}': {e}")

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
    def step_time(self) -> float:
        """
        Get the step time of the simulation.

        Returns:
            float: Step time of the simulation from the world.
        """
        return self._world.step_time

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
        """
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
        return env_param.logger  # type: ignore[return-value]

    @property
    def key_vel(self) -> Any:
        """Get current keyboard velocity command.

        Returns:
            Any: A 2x1 vector ``[[linear], [angular]]`` from keyboard input.
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
