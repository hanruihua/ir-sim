from typing import Any

import numpy as np

from irsim.util.util import points_to_xy_list, traj_to_xy_list

from .env_plot import EnvPlot


class EnvPlot3D(EnvPlot):
    def __init__(
        self,
        world: Any,
        objects: list[Any] | None = None,
        **kwargs: Any,
    ) -> None:
        """Create a 3D plot for the environment.

        Args:
            world: World-like object that provides ranges and grid map.
            objects (list | None): Objects to initialize on the plot.
            saved_figure (dict | None): Savefig keyword arguments.
            figure_pixels (list[int] | None): Figure size in pixels [w, h].
            show_title (bool): Whether to show the title.
            **kwargs: Additional drawing options passed downstream.
        """
        if objects is None:
            objects = []

        super().__init__(world, objects, **kwargs)

        self.ax = self.fig.add_subplot(projection="3d")
        self._init_plot(world, objects, **kwargs)
        self.ax.set_zlim(world.z_range)

    def draw_points(
        self,
        points: list | np.ndarray | None,
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

        x_coordinates, y_coordinates, z_coordinates = points_to_xy_list(
            points, three_d=True
        )

        points = self.ax.scatter(
            x_coordinates, y_coordinates, z_coordinates, "z", s, c, **kwargs
        )

        if refresh:
            self.dyna_point_list.append(points)

    def draw_quiver(
        self, point: np.ndarray | None, refresh: bool = False, **kwargs: Any
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
        self, points: list | np.ndarray, refresh: bool = False, **kwargs: Any
    ) -> None:
        """
        Draw a series of quiver plot on the plot.

        Args:
            points (list or np.ndarray): List of points, each point as [x, y, z, u, v, w]. u, v, w are the components of the vector.

        """

        for point in points.T if isinstance(points, np.ndarray) else points:
            self.draw_quiver(point, refresh, **kwargs)

    def draw_trajectory(
        self,
        traj: list | np.ndarray,
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

        path_x_list, path_y_list, path_z_list = traj_to_xy_list(traj, three_d=True)

        line = self.ax.plot(
            path_x_list, path_y_list, path_z_list, traj_type, label=label, **kwargs
        )

        if show_direction:
            print("Not support currently")

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
