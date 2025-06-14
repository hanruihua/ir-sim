"""
Class EnvBase is the base class of the environment. This class will read the yaml file and create the world, robot, obstacle, and map objects.

Author: Ruihua Han
"""

import matplotlib
from irsim.global_param import env_param

# Define backend preferences for different operating systems
BACKEND_PREFERENCES = {
    "Darwin": ["MacOSX", "TkAgg", "Qt5Agg", "Agg"],  # macOS
    "Windows": ["TkAgg", "Qt5Agg", "Agg"],  # Windows
    "Linux": ["TkAgg", "Qt5Agg", "Agg"],  # Linux
}

# Get the current operating system from env_param
backends = BACKEND_PREFERENCES.get(
    env_param.platform_name, ["Agg"]
)  # Default to Agg if OS not recognized
backend_set = False

for backend in backends:
    try:
        matplotlib.use(backend)
        backend_set = True
        print(f"Successfully set matplotlib backend to {backend}")
        break
    except Exception as e:
        print(f"Failed to use '{backend}' backend: {e}")

if not backend_set:
    print("All backends failed. Falling back to 'Agg' backend.")
    matplotlib.use("Agg")

from irsim.env.env_config import EnvConfig
from irsim.world import World
from .env_plot import EnvPlot
from irsim.global_param import world_param, env_param
from irsim.world.object_factory import ObjectFactory
from matplotlib import pyplot as plt
import platform
import numpy as np
from .env_logger import EnvLogger
from irsim.lib import random_generate_polygon
from shapely import Polygon
from typing import Optional, Union
import importlib
from irsim.world import ObjectBase
from shapely.strtree import STRtree
from operator import attrgetter
from mpl_toolkits.mplot3d import Axes3D
from irsim.gui.mouse_control import MouseControl

try:
    from irsim.gui.keyboard_control import KeyboardControl
    keyboard_module = True
except ImportError:
    keyboard_module = False

class EnvBase:
    """
    The base class of environment. This class will read the yaml file and create the world, robot, obstacle, and map objects.

    Args:
        world_name (str): Path to the world yaml file.
        display (bool): Flag to display the environment.
        disable_all_plot (bool): Flag to disable all plots and figures.
        save_ani (bool): Flag to save the animation.
        full (bool): Flag to full screen the figure.
        log_file (str): Name of the log file.
        log_level (str): Level of the log output.
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
        self.object_factory = ObjectFactory()
        # init objects (world, obstacle, robot)

        self._world = World(world_name, **self.env_config.parse["world"])

        self._robot_collection = self.object_factory.create_from_parse(
            self.env_config.parse["robot"], "robot"
        )
        self._obstacle_collection = self.object_factory.create_from_parse(
            self.env_config.parse["obstacle"], "obstacle"
        )
        self._map_collection = self.object_factory.create_from_map(
            self._world.obstacle_positions, self._world.buffer_reso
        )

        self._objects = (
            self._robot_collection + self._obstacle_collection + self._map_collection
        )

        self._objects.sort(key=attrgetter("id"))
        self.build_tree()

        # env parameters
        self._env_plot = EnvPlot(
            self._world.grid_map,
            self.objects,
            self._world.x_range,
            self._world.y_range,
            **self._world.plot_parse,
        )

        env_param.objects = self.objects

        if world_param.control_mode == "keyboard":
            
            if not keyboard_module:
                self.logger.error(
                    "Keyboard module is not installed. Auto control applied. Please install the dependency by 'pip install ir-sim[keyboard]'."
                )
                world_param.control_mode = "auto"
            else:
                self.keyboard = KeyboardControl(env_ref=self, **self.env_config.parse["keyboard"])

        self.mouse = MouseControl(self._env_plot.ax)

        if full:
            system_platform = platform.system()
            if system_platform == "Linux":
                plt.get_current_fig_manager().full_screen_toggle()

            elif system_platform == "Windows":
                mng = plt.get_current_fig_manager()
                mng.full_screen_toggle()

    def __del__(self):
        # env_param.objects = []

        print(
            "INFO: Simulated Environment End with sim time elapsed: {} seconds".format(
                round(self._world.time, 2)
            )
        )

    def __str__(self):
        return f"Environment: {self._world.name}"

    def step(
        self, action: Union[np.ndarray, list] = None, action_id: Union[int, list] = 0
    ):
        """
        Perform a simulation step in the environment.

        Args:
            action (list or numpy array 2*1): Action to be performed in the environment.

                - differential robot action:  linear velocity, angular velocity
                - omnidirectional robot action: v_x -- velocity in x; v_y -- velocity in y
                - Ackermann robot action: linear velocity, Steering angle

            action_id (int or list of int): Apply the action(s) to the robot(s) with the given id(s).
        """

        if isinstance(action, list):
            if isinstance(action_id, list):
                for a, ai in zip(action, action_id):
                    self._object_step(a, ai)
            else:
                self._objects_step(action)
        else:
            if world_param.control_mode == "keyboard":
                self._object_step(self.key_vel, self.key_id)
            else:
                if isinstance(action_id, list):
                    self._object_step(action, action_id[0])
                else:
                    self._object_step(action, action_id)

        self.build_tree()
        self._objects_check_status()
        self._world.step()

    def _objects_step(self, action: Optional[list] = None):
        action = action + [None] * (len(self.objects) - len(action))
        [obj.step(action) for obj, action in zip(self.objects, action)]

    def _object_step(self, action: np.ndarray, obj_id: int = 0):
        self.objects[obj_id].step(action)
        [obj.step() for obj in self.objects if obj._id != obj_id]

    def _objects_check_status(self):
        [obj.check_status() for obj in self.objects]

    # render
    def render(
        self,
        interval: float = 0.02,
        figure_kwargs=dict(),
        mode: str = "dynamic",
        **kwargs,
    ):
        """
        Render the environment.

        Args:
            interval(float) :  Time interval between frames in seconds.
            figure_kwargs(dict) : Additional keyword arguments for saving figures, see `savefig <https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html>`_ for detail.
            mode(str) : "dynamic", "static", "all" to specify which type of objects to draw and clear.
            kwargs: Additional keyword arguments for drawing components. see :py:meth:`.ObjectBase.plot` function for detail.
        """

        if not self.disable_all_plot:
            if self._world.sampling:

                if self.display:
                    plt.pause(interval)

                if self.save_ani:
                    self.save_figure(save_gif=True, **figure_kwargs)

                if isinstance(self._env_plot.ax, Axes3D):
                    self._env_plot.clear_components(mode, self.objects)
                    self._env_plot.draw_components(mode, self.objects, **kwargs)
                else:
                    self._env_plot.clear_components(mode)
                    self._env_plot.step_objects_plot(mode, self.objects, **kwargs)

    def show(self):
        """
        Show the environment figure.
        """

        self._env_plot.show()

    # draw various components
    def draw_trajectory(self, traj: list, traj_type: str = "g-", **kwargs):
        """
        Draw the trajectory on the environment figure.

        Args:
            traj (list): List of trajectory points (2 * 1 vector).
            traj_type: Type of the trajectory line, see matplotlib plot function for detail.
            **kwargs: Additional keyword arguments for drawing the trajectory, see :py:meth:`.EnvPlot.draw_trajectory` for detail.
        """

        self._env_plot.draw_trajectory(traj, traj_type, **kwargs)

    def draw_points(
        self, points: list, s: int = 30, c: str = "b", refresh: bool = True, **kwargs
    ):
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

    def draw_box(self, vertex: np.ndarray, refresh: bool = False, color: str = "-b"):
        """
        Draw a box by the vertices.

        Args:
            vertex (np.ndarray): matrix of vertices, point_dim*vertex_num
            refresh (bool): whether to refresh the plot, default True
            color (str): color of the box, default '-b'
        """
        self._env_plot.draw_box(vertex, refresh, color)

    def draw_quiver(self, point, refresh=False, **kwargs):
        """
        Draw a single quiver (arrow) on the environment figure.

        Args:
            point: Point data for the quiver
            refresh (bool): Flag to refresh the quiver in the figure, default False
            **kwargs: Additional keyword arguments for drawing the quiver
        """
        self._env_plot.draw_quiver(point, refresh, **kwargs)

    def draw_quivers(self, points, refresh=False, **kwargs):
        """
        Draw multiple quivers (arrows) on the environment figure.

        Args:
            points: Points data for the quivers
            refresh (bool): Flag to refresh the quivers in the figure, default False
            **kwargs: Additional keyword arguments for drawing the quivers
        """
        self._env_plot.draw_quivers(points, refresh, **kwargs)

    def end(self, ending_time: float = 3.0, **kwargs):
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

        if world_param.control_mode == "keyboard":
            self.keyboard.listener.stop()

    def done(self, mode: str = "all"):
        """
        Check if the simulation is done.

        Args:
            mode (str): Mode to check if all or any of the objects are done.

                - all (str): Check if all objects are done.
                - any (str): Check if any of the objects are done.

        Returns:
            bool: True if the simulation is done based on the specified mode, False otherwise.
        """

        done_list = [obj.done() for obj in self.objects if obj.role == "robot"]

        if len(done_list) == 0:
            return False

        if mode == "all":
            return all(done_list)
        elif mode == "any":
            return any(done_list)

    def reset(self):
        """
        Reset the environment.
        """

        self._reset_all()
        self.step(action=np.zeros((2, 1)))
        self._world.reset()
        self.reset_plot()

    def _reset_all(self):
        [obj.reset() for obj in self.objects]

    def reset_plot(self):
        """
        Reset the environment figure.
        """

        self._env_plot.clear_components("all", self.objects)
        self._env_plot.init_plot(self._world.grid_map, self.objects)

    # region: environment change
    def random_obstacle_position(
        self,
        range_low: list = [0, 0, -3.14],
        range_high: list = [10, 10, 3.14],
        ids: list = None,
        non_overlapping: bool = False,
    ):
        """
        Random obstacle positions in the environment.

        Args:
            range_low (list [x, y, theta]): Lower bound of the random range for the obstacle states. Default is [0, 0, -3.14].
            range_high (list [x, y, theta]): Upper bound of the random range for the obstacle states. Default is [10, 10, 3.14].
            ids (list): A list of IDs of objects for which to set random positions. Default is None.
            non_overlapping (bool): If set, the obstacles that will be reset to random obstacles will not overlap with other obstacles. Default is False.
        """
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

                    if any([obj.check_collision(exi_obj) for exi_obj in existing_obj]):
                        counter += 1
                    else:
                        existing_obj.append(obj)
                        break

        self._env_plot.clear_components("all", self.obstacle_list)
        self._env_plot.draw_components("all", self.obstacle_list)

    def random_polygon_shape(
        self,
        center_range: list = [0, 0, 10, 10],
        avg_radius_range: list = [0.1, 1],
        irregularity_range: list = [0, 1],
        spikeyness_range: list = [0, 1],
        num_vertices_range: list = [4, 10],
    ):
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

        self._env_plot.clear_components("all", self.obstacle_list)
        self._env_plot.draw_components("all", self.obstacle_list)

    # endregion: environment change

    # region: object operation

    def create_obstacle(self, **kwargs):
        """
        Create an obstacle in the environment.

        Args:
            **kwargs: Additional parameters for obstacle creation.
                see ObjectFactory.create_obstacle for detail

        Returns:
            Obstacle: An instance of an obstacle.
        """

        return self.object_factory.create_obstacle(**kwargs)

    def add_object(self, obj: ObjectBase):
        """
        Add the object to the environment.

        Args:
            obj (ObjectBase): The object to be added to the environment.
        """
        self._objects.append(obj)
        self.build_tree()

    def add_objects(self, objs: list):
        """
        Add the objects to the environment.

        Args:
            objs (list): List of objects to be added to the environment.
        """
        self._objects.extend(objs)
        self.build_tree()

    def delete_object(self, target_id: int):
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

    def delete_objects(self, target_ids: list):
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

    def build_tree(self):
        """
        Build the geometry tree for the objects in the environment to detect the possible collision objects.
        """

        env_param.GeometryTree = STRtree([obj.geometry for obj in self.objects])

    # endregion: object operation

    # region: get information

    def get_robot_state(self):
        """
        Get the current state of the robot.

        Returns:
            state: 3*1 vector [x, y, theta]

        """

        return self.robot._state

    def get_lidar_scan(self, id: int = 0):
        """
        Get the lidar scan of the robot with the given id.

        Args:
            id (int): Id of the robot.

        Returns:
            Dict: Dict of lidar scan points, see :py:meth:`.world.sensors.lidar2d.Lidar2D.get_scan` for detail.
        """

        return self.robot_list[id].get_lidar_scan()

    def get_lidar_offset(self, id: int = 0):
        """
        Get the lidar offset of the robot with the given id.


        Args:
            id (int): Id of the robot.

        Returns:
            list of float: Lidar offset of the robot, [x, y, theta]
        """

        return self.robot_list[id].get_lidar_offset()

    def get_obstacle_info_list(self):
        """
        Get the information of the obstacles in the environment.

        Returns:
            list of dict: List of obstacle information, see :py:meth:`.ObjectBase.get_obstacle_info` for detail.
        """

        return [obj.get_obstacle_info() for obj in self.obstacle_list]

    def get_robot_info(self, id: int = 0):
        """
        Get the information of the robot with the given id.

        Args:
            id (int): Id of the robot.

        Returns:
            see :py:meth:`.ObjectBase.get_info` for detail
        """

        return self.robot_list[id].get_info()

    def get_robot_info_list(self):
        """
        Get the information of the robots in the environment.

        Returns:
            list of dict: List of robot information, see :py:meth:`.ObjectBase.get_info` for detail.
        """

        return [obj.get_info() for obj in self.robot_list]

    def get_map(self, resolution: float = 0.1):
        """
        Get the map of the environment with the given resolution.

        Args:
            resolution (float): Resolution of the map. Default is 0.1.

        Returns:
            The map of the environment with the specified resolution.
        """
        return self._world.get_map(resolution, self.obstacle_list)

    # endregion: get information

    def save_figure(
        self,
        save_name: str = None,
        include_index: bool = False,
        save_gif: bool = False,
        **kwargs,
    ):
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

    def load_behavior(self, behaviors: str = "behavior_methods"):
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
    def robot_list(self):
        """
        Get the list of robots in the environment.

        Returns:
            list: List of robot objects [].
        """

        return [obj for obj in self.objects if obj.role == "robot"]

    @property
    def obstacle_list(self):
        """
        Get the list of obstacles in the environment.

        Returns:
            list: List of obstacle objects.
        """
        return [obj for obj in self.objects if obj.role == "obstacle"]

    @property
    def objects(self):
        """
        Get all objects in the environment.

        Returns:
            list: List of all objects in the environment.
        """
        return self._objects

    @property
    def static_objects(self):
        """
        Get all static objects in the environment.

        Returns:
            list: List of static objects in the environment.
        """
        return [obj for obj in self.objects if obj.static]

    @property
    def dynamic_objects(self):
        """
        Get all dynamic objects in the environment.

        Returns:
            list: List of dynamic objects in the environment.
        """
        return [obj for obj in self.objects if not obj.static]

    @property
    def step_time(self):
        """
        Get the step time of the simulation.

        Returns:
            float: Step time of the simulation from the world.
        """
        return self._world.step_time

    @property
    def robot(self):
        """
        Get the first robot in the environment.

        Returns:
            Robot: The first robot object in the robot list.
        """
        return self.robot_list[0]

    @property
    def obstacle_number(self):
        """
        Get the number of obstacles in the environment.

        Returns:
            int: Number of obstacles in the environment.
        """
        return len(self.obstacle_list)

    @property
    def robot_number(self):
        """
        Get the number of robots in the environment.

        Returns:
            int: Number of robots in the environment.
        """
        return len(self.robot_list)

    @property
    def logger(self):
        """
        Get the environment logger.

        Returns:
            EnvLogger: The logger instance for the environment.
        """
        return env_param.logger

    @property
    def key_vel(self):
        return self.keyboard.key_vel

    @property
    def key_id(self):
        return self.keyboard.key_id

    @property
    def mouse_pos(self):
        return self.mouse.mouse_pos
    
    @property
    def mouse_left_pos(self):
        return self.mouse.left_click_pos
    
    @property
    def mouse_right_pos(self):
        return self.mouse.right_click_pos

    # endregion: property
