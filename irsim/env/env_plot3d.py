from .env_plot import EnvPlot
import mpl_toolkits.mplot3d.art3d as art3d
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin


class EnvPlot3D(EnvPlot):

    def __init__(
        self,
        grid_map=None,
        objects=[],
        x_range=[0, 10],
        y_range=[0, 10],
        z_range=[0, 10],
        saved_figure=dict(),
        saved_ani=dict(),
        dpi: int = 100,
        figure_pixels: list = [1920, 1080],
        **kwargs,
    ):
        super().__init__(
            grid_map,
            objects,
            x_range,
            y_range,
            saved_figure,
            saved_ani,
            dpi,
            figure_pixels,
            **kwargs,
        )

        self.clear_components()
        self.ax.remove()

        self.ax = self.fig.add_subplot(projection="3d")
        self.z_range = z_range

        self.init_plot(grid_map, objects, **kwargs)
        self.ax.set_zlim(z_range)

    def draw_points(self, points, s=10, c="m", refresh=True, **kwargs):
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
