"""
This file is the implementation of the environment plot to visualize the environment objects.

Author: Ruihua Han
"""

from __future__ import annotations

import glob
import os
import shutil
from collections.abc import Iterable
from math import cos, pi, sin
from typing import Any, Optional

import imageio.v3 as imageio
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import mpl_toolkits.mplot3d.art3d as art3d
import numpy as np
from matplotlib import image
from matplotlib.lines import Line2D
from matplotlib.patches import Arrow, Circle, Ellipse, Polygon, Rectangle, Wedge
from mpl_toolkits.mplot3d import Axes3D

from irsim.config.path_param import path_manager as pm
from irsim.util.util import file_check, points_to_xy_list, traj_to_xy_list


class EnvPlot:
    """
    EnvPlot class for visualizing the environment.

    Args:
        world: The world object with ranges and a ``plot_parse`` dictionary that
            configures plotting (e.g., saved_figure, figure_pixels, show_title,
            no_axis, tight).
        objects (list, optional): Initial objects to draw. Default is ``[]``.
        kwargs: Plot overrides that update ``world.plot_parse`` at runtime
            (e.g., ``saved_figure``, ``figure_pixels``, ``show_title``,
            ``no_axis``, ``tight``).
    """

    def __init__(
        self,
        world: Any,
        objects: Optional[list[Any]] = None,
        **kwargs: Any,
    ) -> None:
        """
        Initialize the EnvPlot instance.

        Sets up the matplotlib figure, configures plotting parameters,
        and initializes the plot with world data and objects. Any provided
        ``kwargs`` override values in ``world.plot_parse``.
        """

        world.plot_parse.update(kwargs)

        # default saved figure kwargs
        self.saved_figure_kwargs: dict[str, Any] = {
            "dpi": 100,
            "bbox_inches": "tight",
        }

        self.saved_figure_kwargs.update(world.plot_parse.get("saved_figure", {}))
        figure_pixels = world.plot_parse.get("figure_pixels", [1000, 800])

        if objects is None:
            objects = []

        # Create matplotlib figure and axes
        self.fig, self.ax = plt.subplots(
            figsize=(
                figure_pixels[0] / self.saved_figure_kwargs["dpi"],
                figure_pixels[1] / self.saved_figure_kwargs["dpi"],
            ),
            dpi=self.saved_figure_kwargs["dpi"],
        )

        self.viewpoint = world.plot_parse.get("viewpoint", None)

        # Initialize dynamic plotting lists
        self.dyna_line_list: list[Any] = []
        self.dyna_point_list: list[Any] = []
        self.dyna_quiver_list: list[Any] = []

        # Initialize the plot with world data
        self._init_plot(world, objects, **kwargs)

    def _init_plot(
        self,
        world: Any,
        objects: list[Any],
        **kwargs: Any,
    ) -> None:
        """
        Initialize or re-initialize drawing on the existing figure/axes.

        Safe to call when the world and objects change (e.g., YAML reload);
        it reuses the current figure without creating a new window.

        Args:
            world: World instance providing ranges and grid map.
            objects (list): List of objects to initialize and draw.
            **kwargs: Plot overrides merged into ``world.plot_parse``.
        """

        world.plot_parse.update(kwargs)
        self.world = world
        self.saved_ani_kwargs: dict[str, Any] = {}
        self.title = world.plot_parse.get("title", None)
        self.show_title = world.plot_parse.get("show_title", True)
        self.saved_figure_kwargs.update(world.plot_parse.get("saved_figure", {}))

        if isinstance(self.ax, Axes3D):
            self.ax.set_box_aspect([1, 1, 1])
        else:
            self.ax.set_aspect("equal")

        self.ax.set_xlim(self.world.x_range)
        self.ax.set_ylim(self.world.y_range)

        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")

        self.init_objects_plot(objects)
        self.draw_grid_map(world.grid_map)

        if world.plot_parse.get("no_axis", False):
            plt.axis("off")
        if world.plot_parse.get("tight", True):
            self.fig.tight_layout()

    def step(
        self, mode: str = "dynamic", objects: Optional[list[Any]] = None, **kwargs: Any
    ) -> None:
        """Advance the plot by one step for the given objects.

        Args:
            mode (str): Which objects to update: "dynamic", "static", or "all".
            objects (list | None): The objects to update/draw. Defaults to empty list.
            **kwargs: Extra drawing options passed through to objects' plot methods.
        """
        if objects is None:
            objects = []

        if self.show_title:
            self.update_title()

        self.set_ax_viewpoint(objects)

        if isinstance(self.ax, Axes3D):
            self.clear_components(mode, objects)
            self.draw_components(mode, objects, **kwargs)
        else:
            self.clear_components(mode)
            self.step_objects_plot(mode, objects, **kwargs)

    def init_objects_plot(self, objects: list[Any], **kwargs: Any) -> None:
        """Initialize plot state for provided objects, then render once.

        Args:
            objects (list): Objects to be initialized on the axes.
            **kwargs: Extra drawing options passed to initialization/plot.
        """
        if self.show_title:
            self.update_title()

        [obj._init_plot(self.ax, **kwargs) for obj in objects]
        self.step_objects_plot("all", objects, **kwargs)

    def step_objects_plot(
        self, mode: str = "dynamic", objects: Optional[list[Any]] = None, **kwargs: Any
    ) -> None:
        """
        Update the plot for the objects by transform based on the object's original geometry.
        """
        if objects is None:
            objects = []
        if mode == "dynamic":
            [obj._step_plot(**kwargs) for obj in objects if not obj.static]
        elif mode == "static":
            [obj._step_plot(**kwargs) for obj in objects if obj.static]
        elif mode == "all":
            [obj._step_plot(**kwargs) for obj in objects]
        else:
            self.logger.error("Error: Invalid draw mode")

    def draw_components(
        self, mode: str = "all", objects: Optional[list[Any]] = None, **kwargs: Any
    ) -> None:
        """
        Draw the components in the environment with global axis.

        Args:
            mode (str): 'static', 'dynamic', or 'all' to specify which objects to draw.
            objects (list): List of objects to draw.
            kwargs: Additional plotting options.
        """
        if objects is None:
            objects = []
        if mode == "static":
            [obj.plot(self.ax, **kwargs) for obj in objects if obj.static]
        elif mode == "dynamic":
            [obj.plot(self.ax, **kwargs) for obj in objects if not obj.static]
        elif mode == "all":
            [obj.plot(self.ax, **kwargs) for obj in objects]
        else:
            self.logger.error("Error: Invalid draw mode")

    def clear_components(
        self, mode: str = "all", objects: Optional[list[Any]] = None
    ) -> None:
        """
        Clear the components in the environment.

        Args:
            mode (str): 'static', 'dynamic', or 'all' to specify which objects to clear.
            objects (list): List of objects to clear.
        """
        if objects is None:
            objects = []
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

    def draw_grid_map(self, grid_map: Optional[Any] = None, **kwargs: Any) -> None:
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
        traj: list[Any] | np.ndarray,
        traj_type: str = "g-",
        label: str = "trajectory",
        show_direction: bool = False,
        refresh: bool = False,
        **kwargs: Any,
    ) -> None:
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
        path_x_list, path_y_list = traj_to_xy_list(traj)

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

    def draw_points(
        self,
        points: Optional[list[Any] | np.ndarray],
        s: int = 10,
        c: str = "m",
        refresh: bool = True,
        **kwargs: Any,
    ) -> None:
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

        x_coordinates, y_coordinates = points_to_xy_list(points)

        points_plot = self.ax.scatter(x_coordinates, y_coordinates, s, c, **kwargs)

        if refresh:
            self.dyna_point_list.append(points_plot)

    def draw_quiver(
        self,
        point: Optional[np.ndarray],
        refresh: bool = False,
        color: str = "black",
        **kwargs: Any,
    ) -> None:
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

    def draw_quivers(
        self,
        points: Iterable[np.ndarray],
        refresh: bool = False,
        color: str = "black",
        **kwargs: Any,
    ) -> None:
        """
        Draw a series of quiver plot on the plot.

        Args:
            points (list or np.ndarray): List of points, each point as [x, y, u, v]. u, v are the components of the vector.

        """

        for point in points:
            self.draw_quiver(point, refresh, color=color, **kwargs)

    def draw_box(
        self, vertices: np.ndarray, refresh: bool = False, color: str = "b-"
    ) -> None:
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

    def update_title(self) -> None:
        """Update the figure title with current time/status or a custom title."""
        if self.title is not None:
            self.ax.set_title(self.title, pad=3)
        else:
            self.ax.set_title(
                f"Simulation Time: {self.world.time:.2f}s, Status: {self.world.status}",
                pad=3,
            )

    def save_figure(
        self,
        file_name: str = "",
        file_format: str = "png",
        include_index: bool = False,
        save_gif: bool = False,
        **kwargs: Any,
    ) -> None:
        """
        Save the current figure.

        Args:
            file_name (str): Name of the figure. Default is ''.
            file_format (str): Format of the figure. Default is 'png'.
            kwargs: Additional arguments for saving the figure.
                See https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for details.
        """

        fp = pm.ani_buffer_path if save_gif else pm.fig_path

        if not os.path.exists(fp):
            os.makedirs(fp)

        self.saved_figure_kwargs.update(kwargs)

        if include_index or save_gif:
            order = str(self._world_param.count).zfill(4)
            full_name = fp + "/" + file_name + "_" + order + "." + file_format
        else:
            full_name = fp + "/" + file_name + "." + file_format

        self.fig.savefig(full_name, format=file_format, **self.saved_figure_kwargs)

    def save_animate(
        self,
        ani_name: str = "animation",
        suffix: str = ".gif",
        last_frame_duration: int = 1,
        rm_fig_path: bool = True,
        **kwargs: Any,
    ) -> None:
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

    def set_ax_viewpoint(self, objects: Optional[list[Any]] = None) -> None:
        """
        Set the viewpoint of the plot windows by the viewpoint parameter.

        Args:
            objects (list): List of objects to search for the viewpoint.
        """
        if self.viewpoint is None:
            point = None
        elif isinstance(self.viewpoint, str):
            if objects is None:
                target = None
            else:
                target = next(
                    (
                        obj
                        for obj in objects
                        if getattr(obj, "name", None) == self.viewpoint
                    ),
                    None,
                )

            point = target.state[0:2].flatten().tolist() if target is not None else None

        elif isinstance(self.viewpoint, list):
            point = self.viewpoint
        else:
            point = None

        if point is None:
            pass
        else:
            # Preserve current zoom span if available; otherwise use world size
            cur_xlim = self.ax.get_xlim()
            cur_ylim = self.ax.get_ylim()

            cur_width = cur_xlim[1] - cur_xlim[0]
            cur_height = cur_ylim[1] - cur_ylim[0]

            view_point_x1 = point[0] - cur_width / 2.0
            view_point_x2 = point[0] + cur_width / 2.0
            view_point_y1 = point[1] - cur_height / 2.0
            view_point_y2 = point[1] + cur_height / 2.0

            self.ax.set_xlim([view_point_x1, view_point_x2])
            self.ax.set_ylim([view_point_y1, view_point_y2])

    def show(self) -> None:
        """
        Display the plot.
        """
        plt.show()

    def close(self) -> None:
        """
        Close the plot.
        """
        plt.close()

    @property
    def _world_param(self):
        """Access world_param via world's instance if available."""
        if hasattr(self.world, "_wp"):
            return self.world._wp
        from irsim.config import world_param

        return world_param

    @property
    def _env_param(self):
        """Access env_param, fallback to global."""
        from irsim.config import env_param

        return env_param

    @property
    def logger(self):
        return self._env_param.logger

    @property
    def x_range(self):
        return self.world.x_range

    @property
    def y_range(self):
        return self.world.y_range


def linewidth_from_data_units(
    linewidth: float, axis: Any, reference: str = "y"
) -> float:
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


def draw_patch(
    ax: Any,
    shape: str,
    state: Optional[np.ndarray] = None,
    radius: Optional[float] = None,
    vertices: Optional[np.ndarray] = None,
    color: Optional[str] = None,
    zorder: Optional[int] = None,
    linestyle: Optional[str] = None,
    **kwargs: Any,
) -> Any:
    """
    Draw a geometric element (patch or line) on the given axes.

    Supported shapes and expected inputs (refer to irsim.world.object_base plotting patterns):
    - circle: use ``state`` (x,y,theta) and ``radius``; created at origin and transformed
    - rectangle: prefer ``vertices`` (2xN) else use ``width``/``height`` with ``state`` transform
    - polygon: use ``vertices`` (2xN)
    - ellipse: use ``width``/``height`` with ``state`` transform
    - wedge: use ``radius`` and either ``theta1``/``theta2`` (deg) or ``fov`` (rad); transformed by ``state``
    - arrow: use ``state`` for position/orientation or provide ``theta``; supports ``arrow_length`` and ``arrow_width``
    - line|linestring: use ``vertices`` (2xN) to draw a line element

    Styling:
    - color/edgecolor/facecolor, alpha, zorder, linestyle are respected when applicable.

    Returns the created matplotlib artist (patch, line, or list from ax.plot).
    """

    created_element = None

    # Normalize some common styling kwargs
    facecolor = kwargs.pop("facecolor", None)
    edgecolor = kwargs.pop("edgecolor", None)
    alpha = kwargs.pop("alpha", None)
    fill = kwargs.pop("fill", None)

    state = state if state is not None else np.zeros((3, 1))

    # Circle
    if shape == "circle":
        if radius is None:
            raise ValueError("circle requires radius")

        use_radius = radius
        # Create at origin; translate/rotate with transform
        patch = Circle((0.0, 0.0), use_radius)
        created_element = ax.add_patch(patch)
        set_patch_property(
            created_element,
            ax,
            state=state,
            color=color,
            facecolor=facecolor,
            edgecolor=edgecolor,
            alpha=alpha,
            zorder=zorder,
            linestyle=linestyle,
            fill=fill,
        )

    # Rectangle (prefer vertices; otherwise width/height + transform)
    elif shape == "rectangle":
        if vertices is not None:
            patch = Polygon(vertices.T)
            created_element = ax.add_patch(patch)
            set_patch_property(
                created_element,
                ax,
                state=state,  # vertices are absolute
                color=color,
                facecolor=facecolor,
                edgecolor=edgecolor,
                alpha=alpha,
                zorder=zorder,
                linestyle=linestyle,
                fill=fill,
            )
        else:
            width = kwargs.pop("width", None)
            height = kwargs.pop("height", None)
            if width is None or height is None:
                raise ValueError("rectangle requires either vertices or width/height")

            xy = (float(vertices[0, 0]), float(vertices[1, 0]))
            patch = Rectangle(xy, width, height)
            created_element = ax.add_patch(patch)
            set_patch_property(
                created_element,
                ax,
                state=state,
                color=color,
                facecolor=facecolor,
                edgecolor=edgecolor,
                alpha=alpha,
                zorder=zorder,
                linestyle=linestyle,
                fill=fill,
            )

    # Polygon
    elif shape == "polygon":
        if vertices is None:
            raise ValueError("polygon requires vertices (2xN)")
        patch = Polygon(vertices.T)
        created_element = ax.add_patch(patch)
        set_patch_property(
            created_element,
            ax,
            state=state,  # vertices are absolute
            color=color,
            facecolor=facecolor,
            edgecolor=edgecolor,
            alpha=alpha,
            zorder=zorder,
            linestyle=linestyle,
            fill=fill,
        )

    # Ellipse
    elif shape == "ellipse":
        width = kwargs.pop("width", None)
        height = kwargs.pop("height", None)
        if width is None or height is None:
            raise ValueError("ellipse requires width and height")
        patch = Ellipse((0.0, 0.0), width, height, angle=0.0)
        created_element = ax.add_patch(patch)
        set_patch_property(
            created_element,
            ax,
            state=state,
            color=color,
            facecolor=facecolor,
            edgecolor=edgecolor,
            alpha=alpha,
            zorder=zorder,
            linestyle=linestyle,
            fill=fill,
        )

    # Wedge (FOV)
    elif shape == "wedge":
        use_radius = radius if radius is not None else kwargs.pop("radius", None)
        if use_radius is None:
            raise ValueError("wedge requires radius")
        if "theta1" in kwargs and "theta2" in kwargs:
            theta1 = kwargs.pop("theta1")
            theta2 = kwargs.pop("theta2")
        else:
            # build from fov in radians centered at 0
            fov = kwargs.pop("fov", np.pi)
            theta1 = -180 * fov / (2 * np.pi)
            theta2 = 180 * fov / (2 * np.pi)
        patch = Wedge((0.0, 0.0), use_radius, theta1, theta2)
        created_element = ax.add_patch(patch)
        set_patch_property(
            created_element,
            ax,
            state=state,
            color=color,
            facecolor=facecolor,
            edgecolor=edgecolor,
            alpha=alpha,
            zorder=zorder,
            linestyle=linestyle,
            fill=fill,
        )

    # Arrow (velocity/heading)
    elif shape == "arrow":
        arrow_length = kwargs.pop("arrow_length", 0.4)
        arrow_width = kwargs.pop("arrow_width", 0.6)
        # Orientation: use provided theta or state[2]
        theta = kwargs.pop("theta", float(state[2, 0]) if state.shape[0] >= 3 else 0.0)
        x = float(state[0, 0])
        y = float(state[1, 0])
        dx = float(arrow_length * cos(theta))
        dy = float(arrow_length * sin(theta))
        patch = Arrow(x, y, dx, dy, width=arrow_width)
        created_element = ax.add_patch(patch)
        set_patch_property(
            created_element,
            ax,
            state=None,  # absolute coords already applied
            color=color if color is not None else kwargs.pop("arrow_color", None),
            alpha=alpha,
            zorder=zorder if zorder is not None else kwargs.pop("arrow_zorder", None),
            fill=fill,
        )

    # Line / LineString
    elif shape in ("line", "linestring"):
        if vertices is None:
            raise ValueError("line/linestring requires vertices (2xN)")
        if isinstance(ax, Axes3D):
            line3d = art3d.Line3D(
                vertices[0, :], vertices[1, :], zs=kwargs.pop("z", np.zeros((3,)))
            )
            if color is not None:
                line3d.set_color(color)
            if alpha is not None:
                line3d.set_alpha(alpha)
            if zorder is not None:
                line3d.set_zorder(zorder)
            if fill is not None:
                line3d.set_fill(fill)
            ax.add_line(line3d)
            created_element = line3d
        else:
            line2d = Line2D(vertices[0, :], vertices[1, :])
            if linestyle is not None:
                line2d.set_linestyle(linestyle)
            if color is not None:
                line2d.set_color(color)
            if alpha is not None:
                line2d.set_alpha(alpha)
            if zorder is not None:
                line2d.set_zorder(zorder)
            ax.add_line(line2d)
            created_element = line2d

    else:
        raise ValueError(f"Unsupported shape type: {shape}")

    # 3D conversion for patches if needed
    if (
        isinstance(ax, Axes3D)
        and created_element is not None
        and shape not in ("line", "linestring")
    ):
        art3d.patch_2d_to_3d(created_element, z=kwargs.pop("z", 0), zdir="z")

    return created_element


def set_patch_property(
    element: Any,
    ax: Any,
    state: Optional[np.ndarray] = None,
    **kwargs: Any,
) -> None:
    """
    Apply transform and style properties to a patch/artist, matching ObjectBase semantics.

    - If ``state`` provided with at least 3 rows, apply rotation by theta and translation by (x, y).
    - Apply color/facecolor/edgecolor, alpha, zorder, linestyle when supported.
    """

    # Transform by state (x, y, theta)
    if state is not None and state.shape[0] >= 3:
        x = float(state[0, 0])
        y = float(state[1, 0])
        theta = float(state[2, 0])
        transform = mtransforms.Affine2D().rotate(theta).translate(x, y) + ax.transData
        if hasattr(element, "set_transform"):
            element.set_transform(transform)

    # Styling
    color = kwargs.get("color")
    facecolor = kwargs.get("facecolor")
    edgecolor = kwargs.get("edgecolor")
    alpha = kwargs.get("alpha")
    zorder = kwargs.get("zorder")
    linestyle = kwargs.get("linestyle")
    linewidth = kwargs.get("linewidth")
    fill = kwargs.get("fill")

    if color is not None and hasattr(element, "set_color"):
        element.set_color(color)
    if facecolor is not None and hasattr(element, "set_facecolor"):
        element.set_facecolor(facecolor)
    if edgecolor is not None and hasattr(element, "set_edgecolor"):
        element.set_edgecolor(edgecolor)
    if alpha is not None and hasattr(element, "set_alpha"):
        element.set_alpha(alpha)
    if zorder is not None and hasattr(element, "set_zorder"):
        element.set_zorder(zorder)
    if linestyle is not None and hasattr(element, "set_linestyle"):
        element.set_linestyle(linestyle)
    if linewidth is not None and hasattr(element, "set_linewidth"):
        element.set_linewidth(linewidth)
    if fill is not None and hasattr(element, "set_fill"):
        element.set_fill(fill)


def draw_text(
    ax: Any,
    text: str,
    state: np.ndarray,
    offset: Optional[list[float]] = None,
    color: str = "k",
    size: int = 10,
    alpha: float = 1.0,
    zorder: int = 2,
    **kwargs: Any,
) -> Any:
    """
    Draw text at a position relative to the given state.

    Args:
        ax: Matplotlib axis.
        text: Text string to display.
        state: State vector [x, y, theta, ...].
        offset: Position offset [dx, dy] from state position. Defaults to [0, 0].
        color: Text color.
        size: Font size.
        alpha: Text transparency.
        zorder: Drawing order.
        **kwargs: Additional text kwargs.

    Returns:
        Text artist.
    """
    if offset is None:
        offset = [0, 0]

    x = float(state[0, 0]) + offset[0]
    y = float(state[1, 0]) + offset[1]

    if isinstance(ax, Axes3D):
        z = float(state[2, 0]) if state.shape[0] >= 6 else 0
        text_artist = ax.text(
            x, y, z, text, fontsize=size, color=color, alpha=alpha, zorder=zorder, **kwargs
        )
    else:
        text_artist = ax.text(
            x, y, text, fontsize=size, color=color, alpha=alpha, zorder=zorder, **kwargs
        )

    return text_artist


def update_text_position(
    text_artist: Any,
    state: np.ndarray,
    offset: Optional[list[float]] = None,
) -> None:
    """
    Update text artist position based on state and offset.

    Args:
        text_artist: Matplotlib text artist.
        state: Current state vector [x, y, theta, ...].
        offset: Position offset [dx, dy] from state position. Defaults to [0, 0].
    """
    if offset is None:
        offset = [0, 0]

    x = float(state[0, 0]) + offset[0]
    y = float(state[1, 0]) + offset[1]
    text_artist.set_position((x, y))


def update_text_style(
    text_artist: Any,
    color: Optional[str] = None,
    size: Optional[int] = None,
    alpha: Optional[float] = None,
    zorder: Optional[int] = None,
) -> None:
    """
    Update text artist style properties.

    Args:
        text_artist: Matplotlib text artist.
        color: Text color.
        size: Font size.
        alpha: Text transparency.
        zorder: Drawing order.
    """
    if color is not None:
        text_artist.set_color(color)
    if size is not None:
        text_artist.set_fontsize(size)
    if alpha is not None:
        text_artist.set_alpha(alpha)
    if zorder is not None:
        text_artist.set_zorder(zorder)


def draw_trajectory_line(
    ax: Any,
    trajectory: list[Any],
    color: str,
    style: str = "-",
    width: float = 1.0,
    alpha: float = 0.5,
    zorder: int = 0,
    shape: str = "circle",
    keep_length: int = 0,
) -> Any:
    """
    Draw a trajectory line on the given axis.

    Args:
        ax: Matplotlib axis.
        trajectory: List of state arrays.
        color: Line color.
        style: Line style (-, --, :, -.).
        width: Line width in data units (will be converted).
        alpha: Line transparency.
        zorder: Drawing order.
        shape: Object shape (affects cap style).
        keep_length: Number of points to keep (0 = all).

    Returns:
        Line artist (list from ax.plot).
    """
    if keep_length > 0:
        trajectory = trajectory[-keep_length:]

    if not trajectory:
        return ax.plot([], [], color=color, linestyle=style, alpha=alpha, zorder=zorder)

    x_list = [t[0, 0] for t in trajectory]
    y_list = [t[1, 0] for t in trajectory]

    linewidth = linewidth_from_data_units(width, ax, "y")

    if isinstance(ax, Axes3D):
        linewidth = width * 10  # 3D scaling factor

    solid_capstyle = "round" if shape == "circle" else "butt"

    return ax.plot(
        x_list,
        y_list,
        color=color,
        linestyle=style,
        linewidth=linewidth,
        solid_joinstyle="round",
        solid_capstyle=solid_capstyle,
        alpha=alpha,
        zorder=zorder,
    )


def update_trajectory_line(
    line: Any,
    ax: Any,
    trajectory: list[Any],
    width: float = 1.0,
    keep_length: int = 0,
    color: Optional[str] = None,
    style: Optional[str] = None,
    alpha: Optional[float] = None,
    zorder: Optional[int] = None,
) -> None:
    """
    Update trajectory line data and style.

    Args:
        line: Line artist (list from ax.plot).
        ax: Matplotlib axis.
        trajectory: List of state arrays.
        width: Line width in data units.
        keep_length: Number of points to keep (0 = all).
        color: Line color to update.
        style: Line style to update.
        alpha: Line transparency to update.
        zorder: Drawing order to update.
    """
    if keep_length > 0:
        trajectory = trajectory[-keep_length:]

    if not trajectory:
        x_list, y_list = [], []
    else:
        x_list = [t[0, 0] for t in trajectory]
        y_list = [t[1, 0] for t in trajectory]

    if isinstance(line, list) and len(line) > 0:
        line_obj = line[0]
        if isinstance(ax, Axes3D):
            z_list = [0] * len(x_list)
            line_obj.set_data_3d(x_list, y_list, z_list)
        else:
            line_obj.set_data(x_list, y_list)

        linewidth = linewidth_from_data_units(width, ax, "y")
        line_obj.set_linewidth(linewidth)

        if color is not None:
            line_obj.set_color(color)
        if style is not None:
            line_obj.set_linestyle(style)
        if alpha is not None:
            line_obj.set_alpha(alpha)
        if zorder is not None:
            line_obj.set_zorder(zorder)


def update_line_data(
    line: Any,
    ax: Any,
    vertices: np.ndarray,
    state: np.ndarray,
) -> None:
    """
    Update line data by transforming vertices with state.

    Args:
        line: Line2D artist.
        ax: Matplotlib axis.
        vertices: Original vertices (2, N).
        state: Current state [x, y, theta, ...].
    """
    x = float(state[0, 0])
    y = float(state[1, 0])
    theta = float(state[2, 0])

    cos_phi = np.cos(theta)
    sin_phi = np.sin(theta)
    rotation_matrix = np.array([[cos_phi, -sin_phi], [sin_phi, cos_phi]])
    rotated_vertices = np.dot(vertices.T, rotation_matrix.T).T
    translated_vertices = rotated_vertices + np.array([[x], [y]])

    line.set_data(translated_vertices[0, :], translated_vertices[1, :])


def draw_object(
    ax: Any,
    shape: str,
    state: np.ndarray,
    radius: Optional[float] = None,
    vertices: Optional[np.ndarray] = None,
    color: str = "k",
    linestyle: str = "-",
    zorder: int = 1,
    **kwargs: Any,
) -> Any:
    """
    Draw an object patch on the given axis.

    Args:
        ax: Matplotlib axis.
        shape: Shape type ('circle', 'rectangle', 'polygon', etc.).
        state: State vector [x, y, theta, ...].
        radius: Radius for circle shapes.
        vertices: Vertices for polygon/rectangle shapes.
        color: Object color.
        linestyle: Line style for outline.
        zorder: Drawing layer order.
        **kwargs: Additional kwargs passed to draw_patch.

    Returns:
        The created patch artist.
    """
    return draw_patch(
        ax,
        shape=shape,
        state=state,
        radius=radius,
        vertices=vertices,
        color=color,
        linestyle=linestyle,
        zorder=zorder,
        **kwargs,
    )


def draw_object_image(
    ax: Any,
    description: str,
    state: np.ndarray,
    vertices: np.ndarray,
    length: float,
    width: float,
    zorder: int = 2,
    root_path: Optional[str] = None,
) -> Optional[Any]:
    """
    Draw an object using an image file.

    Args:
        ax: Matplotlib axis.
        description: Path or name of the image file.
        state: State vector [x, y, theta, ...].
        vertices: Vertices for positioning the image.
        length: Length of the object.
        width: Width of the object.
        zorder: Drawing layer order.
        root_path: Root path for image search. Defaults to path_manager root.

    Returns:
        The image artist, or None if image not found.
    """
    if root_path is None:
        root_path = pm.root_path + "/world/description/"

    image_path = file_check(description, root_path=root_path)
    if image_path is None:
        return None

    start_x = float(vertices[0, 0])
    start_y = float(vertices[1, 0])
    r_phi = float(state[2, 0])
    r_phi_ang = 180 * r_phi / pi

    img_data = image.imread(image_path)

    img_artist = ax.imshow(
        img_data,
        extent=[
            start_x,
            start_x + length,
            start_y,
            start_y + width,
        ],
        zorder=zorder,
    )

    trans_data = (
        mtransforms.Affine2D().rotate_deg_around(start_x, start_y, r_phi_ang)
        + ax.transData
    )
    img_artist.set_transform(trans_data)

    return img_artist


def draw_goal(
    ax: Any,
    shape: str,
    goal_state: np.ndarray,
    radius: Optional[float] = None,
    vertices: Optional[np.ndarray] = None,
    color: str = "k",
    alpha: float = 0.5,
    zorder: int = 1,
    **kwargs: Any,
) -> Any:
    """
    Draw a goal marker on the given axis.

    Args:
        ax: Matplotlib axis.
        shape: Shape type of the goal marker.
        goal_state: State vector [x, y, theta, ...] for goal position.
        radius: Radius for circle shapes.
        vertices: Vertices for polygon/rectangle shapes.
        color: Goal marker color.
        alpha: Transparency.
        zorder: Drawing layer order.
        **kwargs: Additional kwargs passed to draw_patch.

    Returns:
        The created patch artist.
    """
    return draw_patch(
        ax,
        shape=shape,
        state=goal_state,
        radius=radius,
        vertices=vertices,
        color=color,
        alpha=alpha,
        zorder=zorder,
        **kwargs,
    )


def draw_arrow(
    ax: Any,
    state: np.ndarray,
    theta: Optional[float] = None,
    arrow_length: float = 0.4,
    arrow_width: float = 0.6,
    color: str = "gold",
    zorder: int = 4,
    **kwargs: Any,
) -> Any:
    """
    Draw a velocity/heading arrow on the given axis.

    Args:
        ax: Matplotlib axis.
        state: State vector [x, y, theta, ...] for arrow position.
        theta: Arrow direction in radians. If None, uses state[2].
        arrow_length: Length of the arrow.
        arrow_width: Width of the arrow.
        color: Arrow color.
        zorder: Drawing layer order.
        **kwargs: Additional kwargs passed to draw_patch.

    Returns:
        The created arrow patch.
    """
    if theta is None:
        theta = float(state[2, 0]) if state.shape[0] > 2 else 0.0

    return draw_patch(
        ax,
        shape="arrow",
        state=state,
        color=color,
        zorder=zorder,
        arrow_length=arrow_length,
        arrow_width=arrow_width,
        theta=theta,
        **kwargs,
    )


def draw_trail(
    ax: Any,
    shape: str,
    state: np.ndarray,
    vertices: Optional[np.ndarray] = None,
    radius: Optional[float] = None,
    length: Optional[float] = None,
    width: Optional[float] = None,
    edgecolor: str = "k",
    facecolor: str = "k",
    fill: bool = False,
    alpha: float = 0.7,
    linewidth: float = 0.8,
    zorder: int = 0,
    **kwargs: Any,
) -> Any:
    """
    Draw a trail/ghost element on the given axis.

    Args:
        ax: Matplotlib axis.
        shape: Shape type for the trail.
        state: State vector [x, y, theta, ...] for trail position.
        vertices: Vertices for polygon/rectangle shapes.
        radius: Radius for circle shapes.
        length: Length for rectangle/ellipse shapes.
        width: Width for rectangle/ellipse shapes.
        edgecolor: Edge color of the trail.
        facecolor: Fill color of the trail.
        fill: Whether to fill the trail shape.
        alpha: Transparency.
        linewidth: Line width of the trail edge.
        zorder: Drawing layer order.
        **kwargs: Additional kwargs passed to draw_patch.

    Returns:
        The created patch artist.
    """
    return draw_patch(
        ax,
        shape=shape,
        state=state,
        vertices=vertices,
        radius=radius,
        width=length,
        height=width,
        edgecolor=edgecolor,
        facecolor=facecolor,
        fill=fill,
        alpha=alpha,
        linewidth=linewidth,
        zorder=zorder,
        **kwargs,
    )


def draw_fov(
    ax: Any,
    fov: float,
    fov_radius: float,
    facecolor: str = "lightblue",
    edgecolor: str = "blue",
    alpha: float = 0.5,
    zorder: int = 1,
    **kwargs: Any,
) -> Any:
    """
    Draw a field of view wedge/circle on the given axis.

    The FOV is created at the origin and should be positioned using transforms.

    Args:
        ax: Matplotlib axis.
        fov: Field of view angle in radians.
        fov_radius: Radius of the field of view.
        facecolor: Fill color of the FOV.
        edgecolor: Edge color of the FOV.
        alpha: Transparency.
        zorder: Drawing layer order.
        **kwargs: Additional kwargs passed to draw_patch.

    Returns:
        The created patch artist.
    """
    # Create FOV wedge at origin with no rotation
    start_degree = -180 * fov / (2 * pi)
    end_degree = 180 * fov / (2 * pi)

    state = np.array([[0.0], [0.0], [0.0]])

    # Use circle if FOV is full 360 degrees
    shape = "circle" if abs(fov - 2 * pi) < 0.01 else "wedge"

    return draw_patch(
        ax,
        shape=shape,
        state=state,
        radius=fov_radius,
        theta1=start_degree,
        theta2=end_degree,
        facecolor=facecolor,
        edgecolor=edgecolor,
        alpha=alpha,
        zorder=zorder,
        **kwargs,
    )
