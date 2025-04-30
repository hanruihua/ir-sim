
'''
This file is the implementation of the environment plot to visualize the environment objects.

Author: Ruihua Han
'''

import matplotlib.pyplot as plt
import logging
from irsim.global_param.path_param import path_manager as pm
from irsim.global_param import world_param, env_param
import os
import imageio
import shutil
import glob
from math import sin, cos
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


class EnvPlot:
    """
    EnvPlot class for visualizing the environment.

    Args:
        grid_map (optional): The grid map of the environment (PNG file).
        objects: List of objects in the environment.
        x_range (list): The range of x-axis values. Default is [0, 10].
        y_range (list): The range of y-axis values. Default is [0, 10].
        saved_figure (dict): Keyword arguments for saving the figure.
            See https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for details.
        dpi: Dots per inch for the figure. Default is 100.
        figure_pixels: Width and height of the figure in pixels. Default is [1920, 1080].
        kwargs: Additional options such as color_map, no_axis, and tight.
    """

    def __init__(
        self,
        grid_map=None,
        objects=[],
        x_range=[0, 10],
        y_range=[0, 10],
        saved_figure=dict(),
        figure_pixels: list = [1920, 1080],
        **kwargs,
    ) -> None:
        """
        Initialize the EnvPlot instance.
        """

        dpi = saved_figure.get('dpi', 100)

        self.fig, self.ax = plt.subplots(
            figsize=(figure_pixels[0] / dpi, figure_pixels[1] / dpi), dpi=dpi
        )
        self.x_range = x_range
        self.y_range = y_range

        self.init_plot(grid_map, objects, **kwargs)
        self.color_map = {
            "robot": "g",
            "obstacle": "k",
            "landmark": "b",
            "target": "pink",
        }

        self.color_map.update(kwargs.get("color_map", dict()))

        self.saved_figure_kwargs = {
            "dpi": 100,
            "bbox_inches": "tight",
        }

        self.saved_figure_kwargs.update(saved_figure) 
        self.saved_ani_kwargs = {}

        self.dpi = dpi
        self.figure_pixels = figure_pixels

        self.dyna_line_list = []
        self.dyna_point_list = []
        self.dyna_quiver_list = []

    def init_plot(self, grid_map, objects, no_axis=False, tight=True):
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
            self.ax.set_aspect("equal")

        self.ax.set_xlim(self.x_range)
        self.ax.set_ylim(self.y_range)

        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")

        self.draw_components("all", objects)
        self.draw_grid_map(grid_map)

        if no_axis:
            plt.axis("off")
        if tight:
            self.fig.tight_layout()

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
            logging.error("Error: Invalid draw mode")

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
            pass

        elif mode == "all":
            if objects:
                [obj.plot_clear() for obj in objects]
            else:
                plt.cla()

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
                self.ax.quiver(path_x_list, path_y_list, u_list, v_list)

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
        keep_len=30,
        rm_fig_path=True,
        **kwargs,
    ):
        """
        Save the animation.

        Args:
            ani_name (str): Name of the animation. Default is 'animation'.
            keep_len (int): Length of the last frame. Default is 30.
            suffix (str): Suffix of the animation file. Default is '.gif'.
            rm_fig_path (bool): Whether to remove the figure path after saving. Default is True.
            kwargs: Additional arguments for saving the animation.
                See `format_gif <https://imageio.readthedocs.io/en/v2.8.0/format_gif-pil.html>`_ for details.
        """
        
        if suffix == ".gif":
            self.saved_ani_kwargs.update({"subrectangles": True, "loop": 0})

        self.saved_ani_kwargs.update(kwargs)    

        self.logger.info("Start to create animation")

        ap = pm.ani_path
        fp = pm.ani_buffer_path

        if not os.path.exists(ap):
            os.makedirs(ap)

        images = list(glob.glob(fp + "/*.png"))

        images.sort()
        image_list = []
        for i, file_name in enumerate(images):
            if i == 0:
                continue

            image_list.append(imageio.imread(str(file_name)))
            if i == len(images) - 1:
                for j in range(keep_len):
                    image_list.append(imageio.imread(str(file_name)))

        imageio.mimsave(
            ap + "/" + ani_name + suffix, image_list, **self.saved_ani_kwargs
        )

        self.logger.info("Animation created successfully, saved in " + ap)

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
        value_range = np.diff(axis.get_xlim())
    elif reference == "y":
        length = fig.bbox_inches.height * axis.get_position().height
        value_range = np.diff(axis.get_ylim())
    # Convert length to points
    length *= 72
    # Scale linewidth to value range
    return linewidth * (length / value_range)
