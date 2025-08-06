from .env_plot import EnvPlot
import mpl_toolkits.mplot3d.art3d as art3d
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin
from typing import List, Union, Optional, Any, Dict


class EnvPlot3D(EnvPlot):

    def __init__(
        self,
        world: Any,
        objects: Optional[List[Any]] = None,
        saved_figure: Optional[Dict[str, Any]] = None,
        figure_pixels: Optional[List[int]] = None,
        show_title: bool = True,
        **kwargs: Any,
    ) -> None:
        if objects is None:
            objects = []
        if saved_figure is None:
            saved_figure = {}
        if figure_pixels is None:
            figure_pixels = [1180, 1080]

        super().__init__(
            world, objects, saved_figure, figure_pixels, show_title, **kwargs
        )

        self.ax = self.fig.add_subplot(projection="3d")
        self.z_range = world.z_range

        self.init_plot(world.grid_map, objects, **kwargs)
        self.ax.set_zlim(self.z_range)

    def draw_points(
        self,
        points: Optional[Union[List, np.ndarray]],
        s: int = 10,
        c: str = "m",
        refresh: bool = True,
        **kwargs: Any,
    ) -> None:
        """
        Draw points on the plot.

        Args:
            points (list): List of points, each point as [x, y, z].
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
            z_coordinates = [point[2] for point in points]

        elif isinstance(points, np.ndarray):

            if points.shape[1] > 1:
                x_coordinates = [point[0] for point in points.T]
                y_coordinates = [point[1] for point in points.T]
                z_coordinates = [point[2] for point in points.T]
            else:
                x_coordinates = points[0]
                y_coordinates = points[1]
                z_coordinates = points[2]

        points = self.ax.scatter(
            x_coordinates, y_coordinates, z_coordinates, "z", s, c, **kwargs
        )

        if refresh:
            self.dyna_point_list.append(points)

    def draw_quiver(
        self, point: Optional[np.ndarray], refresh: bool = False, **kwargs: Any
    ) -> None:
        """
        Draw a quiver plot on the plot.

        Args:
            points (6*1 np.ndarray): List of points, each point as [x, y, z, u, v, w]. u, v, w are the components of the vector.
            kwargs: Additional plotting options.
        """

        if point is None:
            return

        ax_point = self.ax.scatter(
            point[0],
            point[1],
            point[2],
            color=kwargs.get("point_color", "blue"),
            label="Points",
        )

        ax_quiver = self.ax.quiver(
            point[0],
            point[1],
            point[2],  # starting positions
            point[3],
            point[4],
            point[5],  # vector components (direction)
            length=0.2,
            normalize=True,
            color=kwargs.get("quiver_color", "red"),
            label="Direction",
        )

        if refresh:
            self.dyna_quiver_list.append(ax_quiver)
            self.dyna_point_list.append(ax_point)

    def draw_quivers(
        self, points: Union[List, np.ndarray], refresh: bool = False, **kwargs: Any
    ) -> None:
        """
        Draw a series of quiver plot on the plot.

        Args:
            points (list or np.ndarray): List of points, each point as [x, y, z, u, v, w]. u, v, w are the components of the vector.

        """

        for point in points.T:
            self.draw_quiver(point, refresh, **kwargs)

    def draw_trajectory(
        self,
        traj: Union[List, np.ndarray],
        traj_type: str = "g-",
        label: str = "trajectory",
        show_direction: bool = False,
        refresh: bool = False,
        **kwargs: Any,
    ) -> None:
        """
        Draw a trajectory on the plot.

        Args:
            traj (list or np.ndarray): List of points or array of points [x, y, z].
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
            path_z_list = [p[2, 0] for p in traj]
        elif isinstance(traj, np.ndarray):
            path_x_list = [p[0] for p in traj.T]
            path_y_list = [p[1] for p in traj.T]
            path_z_list = [p[2] for p in traj.T]

        line = self.ax.plot(
            path_x_list, path_y_list, path_z_list, traj_type, label=label, **kwargs
        )

        if show_direction:
            print("Not support currently")
            # if isinstance(traj, list):
            #     u_list = [cos(p[2, 0]) for p in traj]
            #     v_list = [sin(p[2, 0]) for p in traj]
            # elif isinstance(traj, np.ndarray):
            #     u_list = [cos(p[2]) for p in traj.T]
            #     v_list = [sin(p[2]) for p in traj.T]

            # if isinstance(self.ax, Axes3D):
            #     path_z_list = [0] * len(path_x_list)
            #     w_list = [0] * len(u_list)

            #     self.ax.quiver(path_x_list, path_y_list, path_z_list, u_list, v_list, w_list)

            # else:
            #     self.ax.quiver(path_x_list, path_y_list, u_list, v_list)

        if refresh:
            self.dyna_line_list.append(line)

    def update_title(self) -> None:
        """
        Override the parent's update_title method to handle 3D plots properly.
        """
        if not self.show_title:
            return

        if self.title is not None:
            self.fig.suptitle(self.title, fontsize=12)
        else:
            self.fig.suptitle(
                f"Simulation Time: {self.world.time:.2f}s, Status: {self.world.status}",
                fontsize=12,
            )
