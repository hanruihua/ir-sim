"""
This file is the implementation of the environment plot to visualize the environment objects.

Author: Ruihua Han
"""

import matplotlib.pyplot as plt
import logging
from irsim.global_param.path_param import path_manager as pm
from irsim.global_param import world_param, env_param
import os
import imageio.v3 as imageio
import shutil
import glob
from math import sin, cos
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from typing import Optional


class EnvPlot:
    """
    EnvPlot class for visualizing the environment.

    Args:
        world: The world object containing environment information including grid_map, x_range, y_range.
        objects (list, optional): List of objects in the environment. Default is [].
        saved_figure (dict, optional): Keyword arguments for saving the figure.
            See https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for details.
            Default is dict().
        figure_pixels (list, optional): Width and height of the figure in pixels. Default is [1180, 1080].
        show_title (bool, optional): Whether to show the title. Default is True.
        kwargs: Additional options such as color_map, no_axis, and tight.
    """

    def __init__(
        self,
        world,
        objects=[],
        saved_figure=dict(),
        figure_pixels: list = [1000, 800],
        show_title: bool = True,
        **kwargs,
    ) -> None:
        """
        Initialize the EnvPlot instance.

        Sets up the matplotlib figure, configures plotting parameters,
        and initializes the plot with world data and objects.
        """
        # Store world and basic properties
        self.world = world
        self.x_range = world.x_range
        self.y_range = world.y_range
        self.show_title = show_title
        self.title = None

        # Configure figure saving options
        self.saved_figure_kwargs = {
            "dpi": 100,
            "bbox_inches": "tight",
        }
        self.saved_figure_kwargs.update(saved_figure)

        # Create matplotlib figure and axes
        self.fig, self.ax = plt.subplots(
            figsize=(
                figure_pixels[0] / self.saved_figure_kwargs["dpi"],
                figure_pixels[1] / self.saved_figure_kwargs["dpi"],
            ),
            dpi=self.saved_figure_kwargs["dpi"],
        )

        # Initialize plot settings and appearance
        self.color_map = {
            "robot": "g",
            "obstacle": "k",
            "landmark": "b",
            "target": "pink",
        }
        self.color_map.update(kwargs.get("color_map", dict()))

        # Configure save options
        self.saved_ani_kwargs = {}

        # Initialize dynamic plotting lists
        self.dyna_line_list = []
        self.dyna_point_list = []
        self.dyna_quiver_list = []

        # Initialize the plot with world data
        self.init_plot(world.grid_map, objects, **kwargs)

    def init_plot(self, grid_map, objects, no_axis=False, tight=True, **kwargs):
        """
        Initialize the plot with the given grid map and objects.

        Args:
            grid_map (optional): The grid map of the environment.
            objects (list): List of objects to plot.
            no_axis (bool, optional): Whether to show the axis. Default is False.
            tight (bool, optional): Whether to show the axis tightly. Default is True.
        """

        if isinstance(self.ax, Axes3D):
            self.ax.set_box_aspect([1, 1, 1])
        else:
            self.ax.set_aspect("equal")

        self.ax.set_xlim(self.x_range)
        self.ax.set_ylim(self.y_range)

        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")

        # self.draw_components("all", objects)
        self.init_objects_plot(objects)
        self.draw_grid_map(grid_map)

        if no_axis:
            plt.axis("off")
        if tight:
            self.fig.tight_layout()

    def step(self, mode="dynamic", objects=[], **kwargs):

        if self.show_title:
            self.update_title()

        if isinstance(self.ax, Axes3D):
            self.clear_components(mode, objects)
            self.draw_components(mode, objects, **kwargs)
        else:
            self.clear_components(mode)
            self.step_objects_plot(mode, objects, **kwargs)

    def init_objects_plot(self, objects, **kwargs):

        if self.show_title:
            self.update_title()

        [obj._init_plot(self.ax, **kwargs) for obj in objects]
        self.step_objects_plot("all", objects, **kwargs)

    def step_objects_plot(self, mode="dynamic", objects=[], **kwargs):
        """
        Update the plot for the objects.
        """
        if mode == "dynamic":
            [obj._step_plot(**kwargs) for obj in objects if not obj.static]
        elif mode == "static":
            [obj._step_plot(**kwargs) for obj in objects if obj.static]
        elif mode == "all":
            [obj._step_plot(**kwargs) for obj in objects]
        else:
            self.logger.error("Error: Invalid draw mode")

    def draw_components(self, mode="all", objects=[], **kwargs):
        """
        Draw the components in the environment.

        Args:
            mode (str): 'static', 'dynamic', or 'all' to specify which objects to draw.
            objects (list): List of objects to draw.
            kwargs: Additional plotting options.
        """
        if mode == "static":
            [obj.plot(self.ax, **kwargs) for obj in objects if obj.static]
        elif mode == "dynamic":
            [obj.plot(self.ax, **kwargs) for obj in objects if not obj.static]
        elif mode == "all":
            [obj.plot(self.ax, **kwargs) for obj in objects]
        else:
            self.logger.error("Error: Invalid draw mode")

    def clear_components(self, mode="all", objects=[]):
        """
        Clear the components in the environment.

        Args:
            mode (str): 'static', 'dynamic', or 'all' to specify which objects to clear.
            objects (list): List of objects to clear.
        """
        if mode == "dynamic":
            [obj.plot_clear() for obj in objects if not obj.static]
            [line.pop(0).remove() for line in self.dyna_line_list]
            [points.remove() for points in self.dyna_point_list]
            [quiver.remove() for quiver in self.dyna_quiver_list]

            self.dyna_line_list = []
            self.dyna_point_list = []
            self.dyna_quiver_list = []

        elif mode == "static":
            [obj.plot_clear() for obj in objects if obj.static]

        elif mode == "all":
            [obj.plot_clear(all=True) for obj in objects]

            [line.pop(0).remove() for line in self.dyna_line_list]
            [points.remove() for points in self.dyna_point_list]
            [quiver.remove() for quiver in self.dyna_quiver_list]

            self.dyna_line_list = []
            self.dyna_point_list = []
            self.dyna_quiver_list = []

    def draw_grid_map(self, grid_map=None, **kwargs):
        """
        Draw the grid map on the plot.

        Args:
            grid_map (optional): The grid map to draw.
        """
        if grid_map is not None:
            self.ax.imshow(
                grid_map.T,
                cmap="Greys",
                origin="lower",
                extent=self.x_range + self.y_range,
                zorder=0,
            )

            if isinstance(self.ax, Axes3D):
                print("Map will not show in 3D plot")

    def draw_trajectory(
        self,
        traj,
        traj_type="g-",
        label="trajectory",
        show_direction=False,
        refresh=False,
        **kwargs,
    ):
        """
        Draw a trajectory on the plot.

        Args:
            traj (list or np.ndarray): List of points or array of points [x, y, theta].
            traj_type (str): Type of trajectory line (e.g., 'g-').
                See https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.plot.html for details.
            label (str): Label for the trajectory.
            show_direction (bool): Whether to show the direction of the trajectory.
            refresh (bool): Whether to refresh the plot.
            kwargs: Additional plotting options for ax.plot()
        """
        if isinstance(traj, list):
            path_x_list = [p[0, 0] for p in traj]
            path_y_list = [p[1, 0] for p in traj]
        elif isinstance(traj, np.ndarray):
            path_x_list = [p[0] for p in traj.T]
            path_y_list = [p[1] for p in traj.T]

        line = self.ax.plot(path_x_list, path_y_list, traj_type, label=label, **kwargs)

        if show_direction:
            if isinstance(traj, list):
                u_list = [cos(p[2, 0]) for p in traj]
                v_list = [sin(p[2, 0]) for p in traj]
            elif isinstance(traj, np.ndarray):
                u_list = [cos(p[2]) for p in traj.T]
                v_list = [sin(p[2]) for p in traj.T]

            if isinstance(self.ax, Axes3D):
                path_z_list = [0] * len(path_x_list)
                w_list = [0] * len(u_list)

                self.ax.quiver(
                    path_x_list, path_y_list, path_z_list, u_list, v_list, w_list
                )

            else:
                self.ax.quiver(path_x_list, path_y_list, u_list, v_list, width=0.003)

        if refresh:
            self.dyna_line_list.append(line)

    def draw_points(self, points, s=10, c="m", refresh=True, **kwargs):
        """
        Draw points on the plot.

        Args:
            points (list): List of points, each point as [x, y] or (2, 1) array
                or (np.array): points array: (2, N), NL number of points
            s (int): Size of the points.
            c (str): Color of the points.
            refresh (bool): Whether to refresh the plot.
            kwargs: Additional plotting options.
        """

        if points is None:
            return

        if isinstance(points, list):
            x_coordinates = [point[0] for point in points]
            y_coordinates = [point[1] for point in points]

        elif isinstance(points, np.ndarray):

            if points.shape[1] > 1:
                x_coordinates = [point[0] for point in points.T]
                y_coordinates = [point[1] for point in points.T]
            else:
                x_coordinates = points[0]
                y_coordinates = points[1]

        points_plot = self.ax.scatter(x_coordinates, y_coordinates, s, c, **kwargs)

        if refresh:
            self.dyna_point_list.append(points_plot)

    def draw_quiver(self, point, refresh=False, color="black", **kwargs):
        """
        Draw a quiver plot on the plot.

        Args:
            points (4*1 np.ndarray): List of points, each point as [x, y, u, v]. u, v are the components of the vector.
            kwargs: Additional plotting options.
        """

        if point is None:
            return

        ax_point = self.ax.scatter(point[0], point[1], color=color)

        ax_quiver = self.ax.quiver(
            point[0],
            point[1],  # starting positions
            point[2],
            point[3],  # vector components (direction)
            color=color,
            **kwargs,
        )

        if refresh:
            self.dyna_quiver_list.append(ax_quiver)
            self.dyna_point_list.append(ax_point)

    def draw_quivers(self, points, refresh=False, color="black", **kwargs):
        """
        Draw a series of quiver plot on the plot.

        Args:
            points (list or np.ndarray): List of points, each point as [x, y, u, v]. u, v are the components of the vector.

        """

        for point in points:
            self.draw_quiver(point, refresh, color=color, **kwargs)

    def draw_box(self, vertices, refresh=False, color="b-"):
        """
        Draw a box by the vertices.

        Args:
            vertices (np.ndarray): 2xN array of vertices.
            refresh (bool): Whether to refresh the plot.
            color (str): Color and line type of the box.
        """
        temp_vertex = np.c_[vertices, vertices[0:2, 0]]
        box_line = self.ax.plot(temp_vertex[0, :], temp_vertex[1, :], color)

        if refresh:
            self.dyna_line_list.append(box_line)

    def update_title(self):
        if self.title is not None:
            self.ax.set_title(self.title, pad=3)
        else:
            self.ax.set_title(
                f"Simulation Time: {self.world.time:.2f}s, Status: {self.world.status}",
                pad=3,
            )

    def save_figure(
        self,
        file_name="",
        file_format="png",
        include_index=False,
        save_gif=False,
        **kwargs,
    ):
        """
        Save the current figure.

        Args:
            file_name (str): Name of the figure. Default is ''.
            file_format (str): Format of the figure. Default is 'png'.
            kwargs: Additional arguments for saving the figure.
                See https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for details.
        """

        if save_gif:
            fp = pm.ani_buffer_path
        else:
            fp = pm.fig_path

        if not os.path.exists(fp):
            os.makedirs(fp)

        self.saved_figure_kwargs.update(kwargs)

        if include_index or save_gif:
            order = str(world_param.count).zfill(3)
            full_name = fp + "/" + file_name + "_" + order + "." + file_format
        else:
            full_name = fp + "/" + file_name + "." + file_format

        self.fig.savefig(full_name, format=file_format, **self.saved_figure_kwargs)

    def save_animate(
        self,
        ani_name="animation",
        suffix=".gif",
        last_frame_duration=1,
        rm_fig_path=True,
        **kwargs,
    ):
        """
        Save the animation.

        Args:
            ani_name (str): Name of the animation. Default is 'animation'.
            last_frame_duration (int): Duration of the last frame for the gif. Default is 1 second.
            suffix (str): Suffix of the animation file. Default is '.gif'.
            rm_fig_path (bool): Whether to remove the figure path after saving. Default is True.
            kwargs: Additional arguments for saving the animation.
                See `imageio.imwrite <https://imageio.readthedocs.io/en/stable/_autosummary/imageio.v3.imwrite.html#imageio.v3.imwrite>`_ for details.
        """

        self.saved_ani_kwargs.update(kwargs)

        self.logger.info("Start to create animation")

        ap = pm.ani_path
        fp = pm.ani_buffer_path

        if not os.path.exists(ap):
            os.makedirs(ap)

        images = list(glob.glob(fp + "/*.png"))
        images.sort()
        image_list = [imageio.imread(str(file_name)) for file_name in images]

        if suffix == ".gif":
            # default arguments for gif
            durations = [100] * (len(image_list) - 1) + [last_frame_duration * 1000]
            self.saved_ani_kwargs.update(
                {"plugin": "pillow", "duration": durations, "loop": 0}
            )

        full_name = ap + "/" + ani_name + suffix
        imageio.imwrite(full_name, image_list, **self.saved_ani_kwargs)

        self.logger.info(f"{ani_name} created successfully, saved in {ap}")

        if rm_fig_path:
            shutil.rmtree(fp)

    def show(self):
        """
        Display the plot.
        """
        plt.show()

    def close(self):
        """
        Close the plot.
        """
        plt.close()

    @property
    def logger(self):
        return env_param.logger


def linewidth_from_data_units(linewidth, axis, reference="y"):
    """
    Convert a linewidth in data units to linewidth in points.

    Parameters
    ----------
    linewidth: float
        Linewidth in data units of the respective reference-axis
    axis: matplotlib axis
        The axis which is used to extract the relevant transformation
        data (data limits and size must not change afterwards)
    reference: string
        The axis that is taken as a reference for the data width.
        Possible values: 'x' and 'y'. Defaults to 'y'.

    Returns
    -------
    linewidth: float
        Linewidth in points
    """
    fig = axis.get_figure()
    if reference == "x":
        length = fig.bbox_inches.width * axis.get_position().width
        value_range = np.diff(axis.get_xlim()).item()
    elif reference == "y":
        length = fig.bbox_inches.height * axis.get_position().height
        value_range = np.diff(axis.get_ylim()).item()
    # Convert length to points
    length *= 72
    # Scale linewidth to value range
    return linewidth * (length / value_range)
