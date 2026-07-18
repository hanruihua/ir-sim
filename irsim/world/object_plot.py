"""Matplotlib rendering for :class:`irsim.world.object_base.ObjectBase`."""

from __future__ import annotations

from math import pi
from typing import TYPE_CHECKING, Any

import matplotlib.transforms as mtransforms
import numpy as np
from matplotlib import image
from matplotlib.patches import Arrow, Circle, Wedge
from mpl_toolkits.mplot3d import Axes3D

from irsim.config.path_param import path_manager
from irsim.env.env_plot import draw_patch, linewidth_from_data_units, set_patch_property
from irsim.util.util import file_check

if TYPE_CHECKING:
    from irsim.world.object_base import ObjectBase

_PLOT_ATTRIBUTES = (
    "object_patch",
    "object_line",
    "object_img",
    "goal_patch",
    "_text",
    "_goal_text",
    "arrow_patch",
    "trajectory_line",
    "fov_patch",
)


class ObjectPlot:
    """Render and update the Matplotlib artists for one simulation object.

    Artist attributes intentionally remain on ``ObjectBase`` during this
    refactor. This preserves the existing plotting API while moving rendering
    behavior out of the simulation model.
    """

    def __init__(self, owner: ObjectBase) -> None:
        self._owner = owner

    def __getattr__(self, name: str) -> Any:
        """Read simulation and compatibility state from the owning object."""
        owner = object.__getattribute__(self, "_owner")
        return getattr(owner, name)

    def plot(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        **kwargs: Any,
    ) -> None:
        """Plot the object on ``ax`` using optional state and vertices."""
        state = self.state if state is None else state
        vertices = self.vertices if vertices is None else vertices
        self.draw(ax, state, vertices, **kwargs)

    def init(self, ax: Any, **kwargs: Any) -> list[str]:
        """Create the object's artists at its original state."""
        if (
            self.kf is not None
            and not self.static
            and "show_arrow" not in self.plot_kwargs
            and "show_arrow" not in kwargs
        ):
            kwargs.setdefault("show_arrow", self.kf.show_arrow)

        return self.draw(
            ax,
            self.original_state,
            self.original_vertices,
            initial=True,
            **kwargs,
        )

    def draw(
        self,
        ax: Any,
        state: np.ndarray,
        vertices: np.ndarray,
        initial: bool = False,
        **kwargs: Any,
    ) -> list[str]:
        """Create the configured artists for one object."""
        owner = self._owner
        owner.plot_attr_list = list(_PLOT_ATTRIBUTES)

        self.plot_kwargs.update(kwargs)
        owner.ax = ax

        owner.show_goal = self.plot_kwargs.get("show_goal", False)
        owner.show_goal_text = self.plot_kwargs.get("show_goal_text", False)
        owner.show_goals = self.plot_kwargs.get("show_goals", False)
        show_text = self.plot_kwargs.get("show_text", False)
        show_arrow = self.plot_kwargs.get("show_arrow", False)
        show_trajectory = self.plot_kwargs.get("show_trajectory", False)
        owner.show_trail = self.plot_kwargs.get("show_trail", False)
        owner.show_sensor = self.plot_kwargs.get("show_sensor", True)
        show_fov = self.plot_kwargs.get("show_fov", False)
        owner.trail_freq = self.plot_kwargs.get("trail_freq", 2)

        if self.shape != "map":
            self.plot_object(ax, state, vertices, **self.plot_kwargs)

        if self.show_goal:
            goal_state = state if initial else self.goal
            goal_vertices = vertices if initial else self.goal_vertices
            self.plot_goal(ax, goal_state, goal_vertices, **self.plot_kwargs)

        if show_text:
            self.plot_text(ax, state, **self.plot_kwargs)

        if show_arrow:
            current_velocity = self.velocity_xy if np.any(state) else np.zeros((2, 1))
            arrow_theta = 0.0 if initial else self.heading
            self.plot_arrow(
                ax, state, current_velocity, arrow_theta, **self.plot_kwargs
            )

        if show_trajectory:
            trajectory_data = self.trajectory if np.any(state) else []
            self.plot_trajectory(ax, trajectory_data, **self.plot_kwargs)

        if (
            self.show_trail
            and self._world_param.count % self.trail_freq == 0
            and self._world_param.count > 0
        ):
            self.plot_trail(ax, state, self.vertices, **self.plot_kwargs)

        if self.show_sensor:
            for sensor in self.sensors:
                sensor.plot(ax, state, **self.plot_kwargs)

        if show_fov:
            self.plot_fov(ax, **self.plot_kwargs)

        return owner.plot_attr_list

    def step(self, **kwargs: Any) -> None:
        """Update existing artists from the owner's current state."""
        state = self.state
        x = float(state[0, 0])
        y = float(state[1, 0])
        heading = float(state[2, 0])

        self._update_object_patch(kwargs)
        self._update_object_line(x, y, heading, kwargs)
        self._update_object_image(heading)
        self._update_goal_patch(kwargs)
        self._update_arrow_patch(x, y, kwargs)
        self._update_trajectory(kwargs)
        self._update_fov_patch(x, y, heading, kwargs)
        self._update_text(x, y, kwargs)
        self._update_goal_text(kwargs)

        if self.show_trail and self._world_param.count % self.trail_freq == 0:
            self.plot_trail(self.ax, state, self.original_vertices, **self.plot_kwargs)

        if self.show_sensor:
            for sensor in self.sensors:
                sensor.step_plot()

    def _artist(self, name: str) -> Any | None:
        """Return an artist stored on the owner, if it has been created."""
        return getattr(self._owner, name, None)

    def _update_object_patch(self, kwargs: dict[str, Any]) -> None:
        element = self._artist("object_patch")
        if element is None:
            return

        set_patch_property(
            element,
            self.ax,
            state=self.state,
            color=kwargs.get("obj_color", self.color),
            alpha=kwargs.get("obj_alpha"),
            zorder=kwargs.get("obj_zorder"),
            linestyle=kwargs.get("obj_linestyle"),
        )

    def _update_object_line(
        self,
        x: float,
        y: float,
        heading: float,
        kwargs: dict[str, Any],
    ) -> None:
        element = self._artist("object_line")
        if element is None:
            return

        cos_heading = np.cos(heading)
        sin_heading = np.sin(heading)
        rotation = np.array([[cos_heading, -sin_heading], [sin_heading, cos_heading]])
        vertices = rotation @ self.vertices + np.array([[x], [y]])
        element.set_data(vertices[0, :], vertices[1, :])

        if "obj_linestyle" in kwargs:
            element.set_linestyle(kwargs["obj_linestyle"])
        if "obj_color" in kwargs:
            element.set_color(kwargs["obj_color"])
        if "obj_alpha" in kwargs:
            element.set_alpha(kwargs["obj_alpha"])
        if "obj_zorder" in kwargs:
            element.set_zorder(kwargs["obj_zorder"])

    def _update_object_image(self, heading: float) -> None:
        element = self._artist("object_img")
        if element is None or isinstance(self.ax, Axes3D):
            return

        start_x = float(self.vertices[0, 0])
        start_y = float(self.vertices[1, 0])
        element.set_extent(
            [
                start_x,
                start_x + self.length,
                start_y,
                start_y + self.width,
            ]
        )
        transform = (
            mtransforms.Affine2D().rotate_around(start_x, start_y, heading)
            + self.ax.transData
        )
        element.set_transform(transform)

    def _update_goal_patch(self, kwargs: dict[str, Any]) -> None:
        element = self._artist("goal_patch")
        if element is None:
            return

        if self.goal is None:
            element.set_visible(False)
            return

        element.set_visible(True)
        goal_state = (
            self.goal
            if self.goal.shape[0] > 2
            else np.pad(self.goal, (0, 1), "constant", constant_values=0)
        )
        set_patch_property(
            element,
            self.ax,
            state=goal_state,
            color=kwargs.get("goal_color"),
            alpha=kwargs.get("goal_alpha"),
            zorder=kwargs.get("goal_zorder"),
        )

    def _update_arrow_patch(
        self,
        x: float,
        y: float,
        kwargs: dict[str, Any],
    ) -> None:
        element = self._artist("arrow_patch")
        if not isinstance(element, Arrow):
            return

        arrow_state = np.array([[x], [y], [self.heading]])
        set_patch_property(
            element,
            self.ax,
            state=arrow_state,
            color=kwargs.get("arrow_color"),
            alpha=kwargs.get("arrow_alpha"),
            zorder=kwargs.get("arrow_zorder"),
        )

    def _update_fov_patch(
        self,
        x: float,
        y: float,
        heading: float,
        kwargs: dict[str, Any],
    ) -> None:
        element = self._artist("fov_patch")
        if not isinstance(element, Wedge | Circle):
            return

        direction = heading if self.state_dim >= 3 else 0.0
        set_patch_property(
            element,
            self.ax,
            state=np.array([[x], [y], [direction]]),
            facecolor=kwargs.get("fov_color"),
            edgecolor=kwargs.get("fov_edge_color"),
            alpha=kwargs.get("fov_alpha"),
            zorder=kwargs.get("fov_zorder"),
        )

    def _update_trajectory(self, kwargs: dict[str, Any]) -> None:
        """Update a trajectory line and its runtime style properties."""
        element = self._artist("trajectory_line")
        if not isinstance(element, list) or not element:
            return

        line = element[0]
        trajectory = self.trajectory[-self.keep_traj_length :]
        x_list = [state[0, 0] for state in trajectory]
        y_list = [state[1, 0] for state in trajectory]

        if isinstance(self.ax, Axes3D):
            line.set_data_3d(x_list, y_list, [0] * len(x_list))
        else:
            line.set_data(x_list, y_list)

        ax = line.axes
        if ax is not None:
            width = kwargs.get("traj_width", self.width)
            line.set_linewidth(linewidth_from_data_units(width, ax, "y"))

        if "traj_color" in kwargs:
            line.set_color(kwargs["traj_color"])
        if "traj_style" in kwargs:
            line.set_linestyle(kwargs["traj_style"])
        if "traj_alpha" in kwargs:
            line.set_alpha(kwargs["traj_alpha"])
        if "traj_zorder" in kwargs:
            line.set_zorder(kwargs["traj_zorder"])

    def _update_text(self, x: float, y: float, kwargs: dict[str, Any]) -> None:
        """Update the object label when it exists."""
        if not hasattr(self, "_text"):
            return

        text = self._text
        default_position = [-self.radius - 0.1, self.radius + 0.1]
        text_position = kwargs.get(
            "text_position",
            self.plot_kwargs.get("text_position", default_position),
        )
        text.set_position((x + text_position[0], y + text_position[1]))
        text.set_text(self._get_text())
        self._set_text_properties(text, kwargs)

    def _update_goal_text(self, kwargs: dict[str, Any]) -> None:
        """Update the goal label when the object has a goal."""
        if self.goal is None or not hasattr(self, "_goal_text"):
            return

        default_position = [-self.radius - 0.1, self.radius + 0.1]
        text_position = kwargs.get(
            "text_position",
            self.plot_kwargs.get("text_position", default_position),
        )
        goal_text = self._goal_text
        goal_text.set_position(
            (
                self.goal[0, 0] + text_position[0],
                self.goal[1, 0] + text_position[1],
            )
        )
        goal_text.set_text(self._get_goal_text())
        self._set_text_properties(goal_text, kwargs)

    @staticmethod
    def _set_text_properties(text: Any, kwargs: dict[str, Any]) -> None:
        """Apply supplied runtime properties to a Matplotlib text artist."""
        if "text_color" in kwargs:
            text.set_color(kwargs["text_color"])
        if "text_size" in kwargs:
            text.set_fontsize(kwargs["text_size"])
        if "text_alpha" in kwargs:
            text.set_alpha(kwargs["text_alpha"])
        if "text_zorder" in kwargs:
            text.set_zorder(kwargs["text_zorder"])

    def plot_object(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        **kwargs: Any,
    ) -> None:
        """Draw the object's geometry or configured description image."""
        obj_linestyle = kwargs.get("obj_linestyle", "-")
        obj_zorder = kwargs.get("obj_zorder", 3) if self.role == "robot" else 1
        obj_color = kwargs.get("obj_color", self.color)
        obj_alpha = kwargs.get("obj_alpha")
        state = self.state if state is None else state
        vertices = self.vertices if vertices is None else vertices

        if self.description is None or isinstance(ax, Axes3D):
            try:
                if self.shape != "map":
                    self._owner.object_patch = draw_patch(
                        ax,
                        shape=self.shape,
                        state=state,
                        radius=self.radius,
                        center=(
                            self.original_centroid if self.shape == "circle" else None
                        ),
                        vertices=vertices,
                        color=obj_color,
                        alpha=obj_alpha,
                        linestyle=obj_linestyle,
                        zorder=obj_zorder,
                    )
                    self.plot_patch_list.append(self.object_patch)
            except Exception as exc:
                self.logger.error(f"Error occurred while plotting object: {exc!s}")
                raise
        else:
            self.plot_object_image(ax, state, vertices, self.description, **kwargs)

    def plot_object_image(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        description: str | None = None,
        **kwargs: Any,
    ) -> None:
        """Draw an image description at the object's pose."""
        if vertices is None or state is None:
            return

        image_path = file_check(
            description,
            root_path=path_manager.root_path + "/world/description/",
        )
        if image_path is None:
            return

        start_x = float(vertices[0, 0])
        start_y = float(vertices[1, 0])
        angle_degrees = 180 * float(state[2, 0]) / pi
        obj_zorder = kwargs.get("obj_zorder", 2)
        image_data = image.imread(image_path)

        object_image = ax.imshow(
            image_data,
            extent=[
                start_x,
                start_x + self.length,
                start_y,
                start_y + self.width,
            ],
            zorder=obj_zorder,
        )
        transform = (
            mtransforms.Affine2D().rotate_deg_around(start_x, start_y, angle_degrees)
            + ax.transData
        )
        object_image.set_transform(transform)

        self.plot_patch_list.append(object_image)
        self._owner.object_img = object_image

    def plot_trajectory(
        self,
        ax: Any,
        trajectory: list[Any] | None = None,
        keep_traj_length: int = 0,
        **kwargs: Any,
    ) -> None:
        """Draw the object's trajectory."""
        trajectory = self.trajectory if trajectory is None else trajectory
        self._owner.keep_traj_length = keep_traj_length

        traj_color = kwargs.get("traj_color", self.color)
        traj_style = kwargs.get("traj_style", "-")
        traj_width = kwargs.get("traj_width", self.width)
        traj_alpha = kwargs.get("traj_alpha", 0.5)
        traj_zorder = kwargs.get("traj_zorder", 0)

        kept_trajectory = trajectory[-self.keep_traj_length :]
        x_list = [state[0, 0] for state in kept_trajectory]
        y_list = [state[1, 0] for state in kept_trajectory]

        linewidth = linewidth_from_data_units(traj_width, ax, "y")
        if isinstance(ax, Axes3D):
            linewidth = traj_width * 10

        solid_capstyle = "round" if self.shape == "circle" else "butt"
        self._owner.trajectory_line = ax.plot(
            x_list,
            y_list,
            color=traj_color,
            linestyle=traj_style,
            linewidth=linewidth,
            solid_joinstyle="round",
            solid_capstyle=solid_capstyle,
            alpha=traj_alpha,
            zorder=traj_zorder,
        )
        self.plot_line_list.append(self.trajectory_line)

    def plot_goal(
        self,
        ax: Any,
        goal_state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        goal_color: str | None = None,
        goal_zorder: int | None = 1,
        goal_alpha: float | None = 0.5,
        **kwargs: Any,
    ) -> None:
        """Draw the object's goal geometry."""
        if goal_state is None:
            return

        goal_color = self.color if goal_color is None else goal_color
        self._owner.goal_patch = draw_patch(
            ax,
            shape=self.shape,
            state=goal_state,
            radius=self.radius,
            vertices=vertices,
            color=goal_color,
            alpha=goal_alpha,
            zorder=goal_zorder,
        )
        self.plot_patch_list.append(self.goal_patch)

    def plot_text(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        **kwargs: Any,
    ) -> None:
        """Draw the object label and optional goal label."""
        state = self.state if state is None else state
        text_color = kwargs.get("text_color", "k")
        text_size = kwargs.get("text_size", 10)
        text_position = kwargs.get(
            "text_position", [-self.radius - 0.1, self.radius + 0.1]
        )
        text_zorder = kwargs.get("text_zorder", 2)
        text_alpha = kwargs.get("text_alpha", 1)
        x, y = state[0, 0], state[1, 0]

        if isinstance(ax, Axes3D):
            self._owner._text = ax.text(
                x + text_position[0],
                y + text_position[1],
                self.z,
                self._get_text(),
                fontsize=text_size,
                color=text_color,
                zorder=text_zorder,
                alpha=text_alpha,
            )
        else:
            self._owner._text = ax.text(
                x + text_position[0],
                y + text_position[1],
                self._get_text(),
                fontsize=text_size,
                color=text_color,
                zorder=text_zorder,
                alpha=text_alpha,
            )
        self.plot_text_list.append(self._text)

        if self.show_goal and self.show_goal_text and self.goal is not None:
            self._plot_goal_text(
                ax,
                text_position,
                text_color,
                text_size,
                text_zorder,
                text_alpha,
            )

    def _plot_goal_text(
        self,
        ax: Any,
        text_position: list[float],
        text_color: str,
        text_size: int,
        text_zorder: int,
        text_alpha: float,
    ) -> None:
        """Create the goal label for ``plot_text``."""
        goal_x, goal_y = self.goal[0, 0], self.goal[1, 0]
        args = [
            goal_x + text_position[0],
            goal_y + text_position[1],
        ]
        if isinstance(ax, Axes3D):
            args.append(self.z)

        self._owner._goal_text = ax.text(
            *args,
            self._get_goal_text(),
            fontsize=text_size,
            color=text_color,
            zorder=text_zorder,
            alpha=text_alpha,
        )
        self.plot_text_list.append(self._goal_text)

    def plot_arrow(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        velocity: np.ndarray | None = None,
        arrow_theta: float | None = 0.0,
        arrow_length: float = 0.4,
        arrow_width: float = 0.6,
        arrow_color: str | None = None,
        arrow_zorder: int = 3,
        **kwargs: Any,
    ) -> None:
        """Draw an arrow indicating the object's velocity orientation."""
        state = self.state if state is None else state
        arrow_color = "gold" if arrow_color is None else arrow_color

        self._owner.arrow_patch = draw_patch(
            ax,
            shape="arrow",
            state=state,
            color=arrow_color,
            alpha=kwargs.get("arrow_alpha"),
            zorder=arrow_zorder,
            arrow_length=arrow_length,
            arrow_width=arrow_width,
            theta=arrow_theta,
        )
        self.plot_patch_list.append(self.arrow_patch)

    def plot_trail(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        keep_trail_length: int = 0,
        **kwargs: Any,
    ) -> None:
        """Draw a historical outline of the object."""
        vertices = self.original_vertices if vertices is None else vertices
        trail_type = kwargs.get("trail_type", self.shape)
        trail_edgecolor = kwargs.get("trail_edgecolor", self.color)
        trail_linewidth = kwargs.get("trail_linewidth", 0.8)
        trail_alpha = kwargs.get("trail_alpha", 0.7)
        trail_fill = kwargs.get("trail_fill", False)
        trail_color = kwargs.get("trail_color", self.color)
        trail_zorder = kwargs.get("trail_zorder", 0)

        trail = draw_patch(
            ax,
            shape=trail_type,
            state=state,
            vertices=vertices,
            radius=self.radius,
            center=(self.original_centroid if trail_type == "circle" else None),
            width=self.length,
            height=self.width,
            edgecolor=trail_edgecolor,
            facecolor=trail_color,
            fill=trail_fill,
            alpha=trail_alpha,
            linewidth=trail_linewidth,
            zorder=trail_zorder,
        )
        self.plot_trail_list.append(trail)

        if len(self.plot_trail_list) > keep_trail_length and keep_trail_length > 0:
            self.plot_trail_list.pop(0).remove()

    def plot_fov(self, ax: Any, **kwargs: Any) -> None:
        """Draw the object's configured field of view."""
        if self.fov is None or self.fov_radius is None:
            return

        fov_color = kwargs.get("fov_color", "lightblue")
        fov_edge_color = kwargs.get("fov_edge_color", "blue")
        fov_zorder = kwargs.get("fov_zorder", 1)
        fov_alpha = kwargs.get("fov_alpha", 0.5)
        start_degree = -180 * self.fov / (2 * pi)
        end_degree = 180 * self.fov / (2 * pi)
        shape = "circle" if abs(self.fov - 2 * pi) < 0.01 else "wedge"

        self._owner.fov_patch = draw_patch(
            ax,
            shape=shape,
            state=np.zeros((3, 1)),
            radius=self.fov_radius,
            theta1=start_degree,
            theta2=end_degree,
            facecolor=fov_color,
            edgecolor=fov_edge_color,
            alpha=fov_alpha,
            zorder=fov_zorder,
        )
        self.plot_patch_list.append(self.fov_patch)

    def plot_uncertainty(self, ax: Any, **kwargs: Any) -> None:
        """Reserved for future uncertainty rendering."""

    def clear(self, all: bool = False) -> None:
        """Remove this object's artists, optionally including its trails."""
        for patch in self.plot_patch_list:
            patch.remove()
        for line in self.plot_line_list:
            line.pop(0).remove()
        for text in self.plot_text_list:
            text.remove()

        if all:
            for trail in self.plot_trail_list:
                trail.remove()
            self._owner.plot_trail_list = []

        self._owner.plot_patch_list = []
        self._owner.plot_line_list = []
        self._owner.plot_text_list = []

        for sensor in self.sensors:
            sensor.plot_clear()
