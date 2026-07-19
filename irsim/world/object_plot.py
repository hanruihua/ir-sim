"""Matplotlib rendering for :class:`irsim.world.object_base.ObjectBase`."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field, replace
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


@dataclass(frozen=True, slots=True)
class VisibilityOptions:
    """Control which components of an object are rendered."""

    goal: bool = False  # Draw the goal geometry.
    goal_text: bool = False  # Draw the goal label with object text.
    goals: bool = False  # Preserve the legacy multi-goal flag.
    text: bool = False  # Draw the object label.
    arrow: bool = False  # Draw the velocity-direction arrow.
    trajectory: bool = False  # Draw the continuous trajectory line.
    trail: bool = False  # Draw historical shape snapshots.
    sensor: bool = True  # Draw attached sensor overlays.
    fov: bool = False  # Draw the field-of-view region.


@dataclass(frozen=True, slots=True)
class PatchStyle:
    """Style shared by object and goal patches."""

    color: Any = "k"  # Matplotlib-compatible color.
    alpha: float = 1.0  # Opacity from fully transparent to opaque.
    zorder: int = 1  # Layer order; larger values render above.
    linestyle: str = "-"  # Patch boundary style.
    line_width: float = 1.0  # Boundary width in Matplotlib points.


@dataclass(frozen=True, slots=True)
class ImageStyle:
    """Style specific to an object's description image."""

    zorder: int = 2  # Preserve the legacy image layer default.


@dataclass(frozen=True, slots=True)
class LineStyle:
    """Style and history length for a trajectory line."""

    color: Any = "k"  # Matplotlib-compatible line color.
    style: str = "-"  # Line style, such as "-" or "--".
    width: float = 1.0  # Width in environment data units.
    alpha: float = 0.5  # Line opacity.
    zorder: int = 0  # Matplotlib layer order.
    keep_length: int = 0  # Recent states retained; 0 keeps all.


@dataclass(frozen=True, slots=True)
class TextStyle:
    """Style and offset for object and goal labels."""

    color: Any = "k"  # Matplotlib-compatible text color.
    size: float = 10.0  # Font size in points.
    position: tuple[float, float] = (0.0, 0.0)  # Label offset.
    alpha: float = 1.0  # Text opacity.
    zorder: int = 2  # Matplotlib layer order.


@dataclass(frozen=True, slots=True)
class TrailStyle:
    """Style, frequency, and retention policy for object trails."""

    shape: str = "circle"  # Geometry used for each snapshot.
    edge_color: Any = "k"  # Matplotlib-compatible boundary color.
    line_width: float = 0.8  # Boundary width passed to Matplotlib.
    alpha: float = 0.7  # Snapshot opacity.
    fill: bool = False  # Fill the snapshot interior when true.
    color: Any = "k"  # Matplotlib-compatible interior color.
    zorder: int = 0  # Matplotlib layer order.
    frequency: int = 2  # Simulation steps between snapshots.
    keep_length: int = 0  # Maximum snapshots; 0 keeps all.


@dataclass(frozen=True, slots=True)
class ArrowStyle:
    """Style and dimensions for the velocity arrow."""

    color: Any = "gold"  # Matplotlib-compatible arrow color.
    alpha: float = 1.0  # Arrow opacity.
    zorder: int = 3  # Matplotlib layer order.
    length: float = 0.4  # Length in environment data units.
    width: float = 0.6  # Arrow-head width in data units.


@dataclass(frozen=True, slots=True)
class FovStyle:
    """Style for the field-of-view patch."""

    color: Any = "lightblue"  # Matplotlib-compatible interior color.
    edge_color: Any = "blue"  # Matplotlib-compatible boundary color.
    alpha: float = 0.5  # Region opacity.
    zorder: int = 1  # Matplotlib layer order.


@dataclass(frozen=True, slots=True)
class ObjectPlotOptions:
    """Complete immutable plotting configuration for one object."""

    visibility: VisibilityOptions = field(default_factory=VisibilityOptions)
    object: PatchStyle = field(default_factory=PatchStyle)
    image: ImageStyle = field(default_factory=ImageStyle)
    goal: PatchStyle = field(default_factory=lambda: PatchStyle(alpha=0.5, zorder=1))
    trajectory: LineStyle = field(default_factory=LineStyle)
    text: TextStyle = field(default_factory=TextStyle)
    trail: TrailStyle = field(default_factory=TrailStyle)
    arrow: ArrowStyle = field(default_factory=ArrowStyle)
    fov: FovStyle = field(default_factory=FovStyle)

    @classmethod
    def defaults(
        cls,
        *,
        color: Any,
        width: float,
        radius: float,
        shape: str,
        object_zorder: int,
    ) -> ObjectPlotOptions:
        """Create owner-dependent plotting defaults."""
        return cls(
            object=PatchStyle(color=color, zorder=object_zorder),
            goal=PatchStyle(color=color, alpha=0.5, zorder=1),
            trajectory=LineStyle(color=color, width=width),
            text=TextStyle(position=(-radius - 0.1, radius + 0.1)),
            trail=TrailStyle(
                shape=shape,
                edge_color=color,
                color=color,
            ),
        )

    def with_overrides(self, values: Mapping[str, Any]) -> ObjectPlotOptions:
        """Return a new options value with legacy keyword overrides applied."""
        visibility = _replace_fields(
            self.visibility,
            values,
            {
                "show_goal": "goal",
                "show_goal_text": "goal_text",
                "show_goals": "goals",
                "show_text": "text",
                "show_arrow": "arrow",
                "show_trajectory": "trajectory",
                "show_trail": "trail",
                "show_sensor": "sensor",
                "show_fov": "fov",
            },
        )
        object_style = _replace_fields(
            self.object,
            values,
            {
                "obj_color": "color",
                "obj_alpha": "alpha",
                "obj_zorder": "zorder",
                "obj_linestyle": "linestyle",
                "obj_linewidth": "line_width",
            },
        )
        image = _replace_fields(
            self.image,
            values,
            {"obj_zorder": "zorder"},
        )
        goal = _replace_fields(
            self.goal,
            values,
            {
                "goal_color": "color",
                "goal_alpha": "alpha",
                "goal_zorder": "zorder",
            },
        )
        trajectory = _replace_fields(
            self.trajectory,
            values,
            {
                "traj_color": "color",
                "traj_style": "style",
                "traj_width": "width",
                "traj_alpha": "alpha",
                "traj_zorder": "zorder",
                "keep_traj_length": "keep_length",
            },
        )
        text = _replace_fields(
            self.text,
            values,
            {
                "text_color": "color",
                "text_size": "size",
                "text_alpha": "alpha",
                "text_zorder": "zorder",
            },
        )
        if "text_position" in values:
            text = replace(text, position=tuple(values["text_position"]))

        trail = _replace_fields(
            self.trail,
            values,
            {
                "trail_type": "shape",
                "trail_edgecolor": "edge_color",
                "trail_linewidth": "line_width",
                "trail_alpha": "alpha",
                "trail_fill": "fill",
                "trail_color": "color",
                "trail_zorder": "zorder",
                "trail_freq": "frequency",
                "keep_trail_length": "keep_length",
            },
        )
        arrow = _replace_fields(
            self.arrow,
            values,
            {
                "arrow_color": "color",
                "arrow_alpha": "alpha",
                "arrow_zorder": "zorder",
                "arrow_length": "length",
                "arrow_width": "width",
            },
        )
        fov = _replace_fields(
            self.fov,
            values,
            {
                "fov_color": "color",
                "fov_edge_color": "edge_color",
                "fov_alpha": "alpha",
                "fov_zorder": "zorder",
            },
        )

        return replace(
            self,
            visibility=visibility,
            object=object_style,
            image=image,
            goal=goal,
            trajectory=trajectory,
            text=text,
            trail=trail,
            arrow=arrow,
            fov=fov,
        )


def _replace_fields(
    value: Any,
    options: Mapping[str, Any],
    field_names: Mapping[str, str],
) -> Any:
    """Replace dataclass fields whose legacy option keys are present."""
    changes = {
        field_name: options[option_name]
        for option_name, field_name in field_names.items()
        if option_name in options
    }
    return replace(value, **changes) if changes else value


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

    Simulation and artist state is accessed explicitly through ``owner``.
    Artist attributes intentionally remain on ``ObjectBase`` to preserve the
    existing plotting API while moving rendering behavior out of the model.
    """

    def __init__(self, owner: ObjectBase) -> None:
        self._owner: ObjectBase = owner
        self.options = self._resolve_options()

    @property
    def owner(self) -> ObjectBase:
        """Return the owner through the explicit renderer interface."""
        return self._owner

    @owner.setter
    def owner(self, value: ObjectBase) -> None:
        """Keep explicit owner assignment synchronized with legacy ``_owner``."""
        self._owner = value

    def __getattr__(self, name: str) -> Any:
        """Delegate legacy renderer attribute reads to the owning object."""
        owner = object.__getattribute__(self, "_owner")
        return getattr(owner, name)

    def _resolve_options(
        self, overrides: Mapping[str, Any] | None = None
    ) -> ObjectPlotOptions:
        """Resolve live owner defaults, stored configuration, and call overrides."""
        owner = self.owner
        options = ObjectPlotOptions.defaults(
            color=owner.color,
            width=owner.width,
            radius=owner.radius,
            shape=owner.shape,
            object_zorder=3 if owner.role == "robot" else 1,
        ).with_overrides(owner.plot_kwargs)
        return options.with_overrides(overrides) if overrides else options

    def plot(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        **kwargs: Any,
    ) -> None:
        """Plot the object on ``ax`` using optional state and vertices."""
        state = self.owner.state if state is None else state
        vertices = self.owner.vertices if vertices is None else vertices
        self.draw(ax, state, vertices, **kwargs)

    def init(self, ax: Any, **kwargs: Any) -> list[str]:
        """Create the object's artists at its original state."""
        if (
            self.owner.kf is not None
            and not self.owner.static
            and "show_arrow" not in self.owner.plot_kwargs
            and "show_arrow" not in kwargs
        ):
            kwargs.setdefault("show_arrow", self.owner.kf.show_arrow)

        return self.draw(
            ax,
            self.owner.original_state,
            self.owner.original_vertices,
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
        owner = self.owner
        owner.plot_kwargs.update(kwargs)
        options = self._resolve_options()
        self.options = options

        owner.plot_attr_list = list(_PLOT_ATTRIBUTES)
        owner.ax = ax

        visibility = options.visibility
        owner.show_goal = visibility.goal
        owner.show_goal_text = visibility.goal_text
        owner.show_goals = visibility.goals
        owner.show_trail = visibility.trail
        owner.show_sensor = visibility.sensor
        owner.trail_freq = options.trail.frequency

        if owner.shape != "map":
            self.plot_object(ax, state, vertices, _options=options)

        if visibility.goal:
            goal_state = state if initial else owner.goal
            goal_vertices = vertices if initial else owner.goal_vertices
            self.plot_goal(ax, goal_state, goal_vertices, _options=options)

        if visibility.text:
            self.plot_text(ax, state, _options=options)

        if visibility.arrow:
            current_velocity = owner.velocity_xy if np.any(state) else np.zeros((2, 1))
            arrow_theta = 0.0 if initial else owner.heading
            self.plot_arrow(
                ax,
                state,
                current_velocity,
                arrow_theta,
                _options=options,
            )

        if visibility.trajectory:
            trajectory_data = owner.trajectory if np.any(state) else []
            self.plot_trajectory(ax, trajectory_data, _options=options)

        if (
            visibility.trail
            and owner._world_param.count % options.trail.frequency == 0
            and owner._world_param.count > 0
        ):
            self.plot_trail(ax, state, owner.vertices, _options=options)

        if visibility.sensor:
            sensor_options = {**owner.plot_kwargs, **kwargs}
            for sensor in owner.sensors:
                sensor.plot(ax, state, **sensor_options)

        if visibility.fov:
            self.plot_fov(ax, _options=options)

        return owner.plot_attr_list

    def step(self, **kwargs: Any) -> None:
        """Update existing artists from the owner's current state."""
        owner = self.owner
        options = self._resolve_options(kwargs)
        self.options = options
        state = owner.state
        x = float(state[0, 0])
        y = float(state[1, 0])
        heading = float(state[2, 0])

        self._update_object_patch(options.object)
        self._update_object_line(x, y, heading, options.object)
        self._update_object_image(heading)
        self._update_goal_patch(options.goal)
        self._update_arrow_patch(x, y, options.arrow)
        self._update_trajectory(options.trajectory)
        self._update_fov_patch(x, y, heading, options.fov)
        self._update_text(x, y, options.text)
        self._update_goal_text(options.text)

        if (
            options.visibility.trail
            and owner._world_param.count % options.trail.frequency == 0
        ):
            self.plot_trail(owner.ax, state, owner.original_vertices, _options=options)

        if options.visibility.sensor:
            for sensor in owner.sensors:
                sensor.step_plot()

    def _artist(self, name: str) -> Any | None:
        """Return an artist stored on the owner, if it has been created."""
        return getattr(self.owner, name, None)

    def _update_object_patch(self, style: PatchStyle) -> None:
        element = self._artist("object_patch")
        if element is None:
            return

        set_patch_property(
            element,
            self.owner.ax,
            state=self.owner.state,
            color=style.color,
            alpha=style.alpha,
            zorder=style.zorder,
            linestyle=style.linestyle,
            linewidth=style.line_width,
        )

    def _update_object_line(
        self,
        x: float,
        y: float,
        heading: float,
        style: PatchStyle,
    ) -> None:
        element = self._artist("object_line")
        if element is None:
            return

        cos_heading = np.cos(heading)
        sin_heading = np.sin(heading)
        rotation = np.array([[cos_heading, -sin_heading], [sin_heading, cos_heading]])
        vertices = rotation @ self.owner.vertices + np.array([[x], [y]])
        element.set_data(vertices[0, :], vertices[1, :])

        element.set_linestyle(style.linestyle)
        element.set_color(style.color)
        element.set_alpha(style.alpha)
        element.set_zorder(style.zorder)
        element.set_linewidth(style.line_width)

    def _update_object_image(self, heading: float) -> None:
        element = self._artist("object_img")
        if element is None or isinstance(self.owner.ax, Axes3D):
            return

        start_x = float(self.owner.vertices[0, 0])
        start_y = float(self.owner.vertices[1, 0])
        element.set_extent(
            [
                start_x,
                start_x + self.owner.length,
                start_y,
                start_y + self.owner.width,
            ]
        )
        transform = (
            mtransforms.Affine2D().rotate_around(start_x, start_y, heading)
            + self.owner.ax.transData
        )
        element.set_transform(transform)

    def _update_goal_patch(self, style: PatchStyle) -> None:
        element = self._artist("goal_patch")
        if element is None:
            return

        if self.owner.goal is None:
            element.set_visible(False)
            return

        element.set_visible(True)
        goal_state = (
            self.owner.goal
            if self.owner.goal.shape[0] > 2
            else np.pad(self.owner.goal, (0, 1), "constant", constant_values=0)
        )
        set_patch_property(
            element,
            self.owner.ax,
            state=goal_state,
            color=style.color,
            alpha=style.alpha,
            zorder=style.zorder,
        )

    def _update_arrow_patch(
        self,
        x: float,
        y: float,
        style: ArrowStyle,
    ) -> None:
        element = self._artist("arrow_patch")
        if not isinstance(element, Arrow):
            return

        arrow_state = np.array([[x], [y], [self.owner.heading]])
        set_patch_property(
            element,
            self.owner.ax,
            state=arrow_state,
            color=style.color,
            alpha=style.alpha,
            zorder=style.zorder,
        )

    def _update_fov_patch(
        self,
        x: float,
        y: float,
        heading: float,
        style: FovStyle,
    ) -> None:
        element = self._artist("fov_patch")
        if not isinstance(element, Wedge | Circle):
            return

        direction = heading if self.owner.state_dim >= 3 else 0.0
        set_patch_property(
            element,
            self.owner.ax,
            state=np.array([[x], [y], [direction]]),
            facecolor=style.color,
            edgecolor=style.edge_color,
            alpha=style.alpha,
            zorder=style.zorder,
        )

    def _update_trajectory(self, style: LineStyle) -> None:
        """Update a trajectory line and its runtime style properties."""
        element = self._artist("trajectory_line")
        if not isinstance(element, list) or not element:
            return

        line = element[0]
        trajectory = self.owner.trajectory[-style.keep_length :]
        x_list = [state[0, 0] for state in trajectory]
        y_list = [state[1, 0] for state in trajectory]

        if isinstance(self.owner.ax, Axes3D):
            line.set_data_3d(x_list, y_list, [0] * len(x_list))
        else:
            line.set_data(x_list, y_list)

        ax = line.axes
        if ax is not None:
            line.set_linewidth(linewidth_from_data_units(style.width, ax, "y"))

        line.set_color(style.color)
        line.set_linestyle(style.style)
        line.set_alpha(style.alpha)
        line.set_zorder(style.zorder)

    def _update_text(self, x: float, y: float, style: TextStyle) -> None:
        """Update the object label when it exists."""
        owner = self.owner
        if not hasattr(owner, "_text"):
            return

        text = owner._text
        text.set_position((x + style.position[0], y + style.position[1]))
        text.set_text(owner._get_text())
        self._set_text_properties(text, style)

    def _update_goal_text(self, style: TextStyle) -> None:
        """Update the goal label when the object has a goal."""
        owner = self.owner
        if owner.goal is None or not hasattr(owner, "_goal_text"):
            return

        goal_text = owner._goal_text
        goal_text.set_position(
            (
                owner.goal[0, 0] + style.position[0],
                owner.goal[1, 0] + style.position[1],
            )
        )
        goal_text.set_text(owner._get_goal_text())
        self._set_text_properties(goal_text, style)

    @staticmethod
    def _set_text_properties(text: Any, style: TextStyle) -> None:
        """Apply resolved properties to a Matplotlib text artist."""
        text.set_color(style.color)
        text.set_fontsize(style.size)
        text.set_alpha(style.alpha)
        text.set_zorder(style.zorder)

    def plot_object(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        _options: ObjectPlotOptions | None = None,
        **kwargs: Any,
    ) -> None:
        """Draw the object's geometry or configured description image."""
        options = self._resolve_options(kwargs) if _options is None else _options
        state = self.owner.state if state is None else state
        vertices = self.owner.vertices if vertices is None else vertices

        if self.owner.description is None or isinstance(ax, Axes3D):
            try:
                if self.owner.shape != "map":
                    self.owner.object_patch = draw_patch(
                        ax,
                        shape=self.owner.shape,
                        state=state,
                        radius=self.owner.radius,
                        center=(
                            self.owner.original_centroid
                            if self.owner.shape == "circle"
                            else None
                        ),
                        vertices=vertices,
                        color=options.object.color,
                        alpha=options.object.alpha,
                        linestyle=options.object.linestyle,
                        zorder=options.object.zorder,
                    )
                    self.owner.object_patch.set_linewidth(options.object.line_width)
                    self.owner.plot_patch_list.append(self.owner.object_patch)
            except Exception as exc:
                self.owner.logger.error(
                    f"Error occurred while plotting object: {exc!s}"
                )
                raise
        else:
            self.plot_object_image(
                ax,
                state,
                vertices,
                self.owner.description,
                **kwargs,
            )

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
        image_data = image.imread(image_path)
        options = self._resolve_options(kwargs)

        object_image = ax.imshow(
            image_data,
            extent=[
                start_x,
                start_x + self.owner.length,
                start_y,
                start_y + self.owner.width,
            ],
            zorder=options.image.zorder,
        )
        transform = (
            mtransforms.Affine2D().rotate_deg_around(start_x, start_y, angle_degrees)
            + ax.transData
        )
        object_image.set_transform(transform)

        self.owner.plot_patch_list.append(object_image)
        self.owner.object_img = object_image

    def plot_trajectory(
        self,
        ax: Any,
        trajectory: list[Any] | None = None,
        keep_traj_length: int = 0,
        _options: ObjectPlotOptions | None = None,
        **kwargs: Any,
    ) -> None:
        """Draw the object's trajectory."""
        if _options is None:
            options = self._resolve_options(
                {**kwargs, "keep_traj_length": keep_traj_length}
            )
        else:
            options = _options
        trajectory = self.owner.trajectory if trajectory is None else trajectory
        self.owner.keep_traj_length = options.trajectory.keep_length

        kept_trajectory = trajectory[-options.trajectory.keep_length :]
        x_list = [state[0, 0] for state in kept_trajectory]
        y_list = [state[1, 0] for state in kept_trajectory]

        linewidth = linewidth_from_data_units(options.trajectory.width, ax, "y")
        if isinstance(ax, Axes3D):
            linewidth = options.trajectory.width * 10

        solid_capstyle = "round" if self.owner.shape == "circle" else "butt"
        self.owner.trajectory_line = ax.plot(
            x_list,
            y_list,
            color=options.trajectory.color,
            linestyle=options.trajectory.style,
            linewidth=linewidth,
            solid_joinstyle="round",
            solid_capstyle=solid_capstyle,
            alpha=options.trajectory.alpha,
            zorder=options.trajectory.zorder,
        )
        self.owner.plot_line_list.append(self.owner.trajectory_line)

    def plot_goal(
        self,
        ax: Any,
        goal_state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        goal_color: str | None = None,
        goal_zorder: int | None = 1,
        goal_alpha: float | None = 0.5,
        _options: ObjectPlotOptions | None = None,
        **kwargs: Any,
    ) -> None:
        """Draw the object's goal geometry."""
        if goal_state is None:
            return

        if _options is None:
            overrides = {
                **kwargs,
                "goal_zorder": goal_zorder,
                "goal_alpha": goal_alpha,
            }
            if goal_color is not None:
                overrides["goal_color"] = goal_color
            options = self._resolve_options(overrides)
        else:
            options = _options

        self.owner.goal_patch = draw_patch(
            ax,
            shape=self.owner.shape,
            state=goal_state,
            radius=self.owner.radius,
            vertices=vertices,
            color=options.goal.color,
            alpha=options.goal.alpha,
            zorder=options.goal.zorder,
        )
        self.owner.plot_patch_list.append(self.owner.goal_patch)

    def plot_text(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        _options: ObjectPlotOptions | None = None,
        **kwargs: Any,
    ) -> None:
        """Draw the object label and optional goal label."""
        options = self._resolve_options(kwargs) if _options is None else _options
        state = self.owner.state if state is None else state
        x, y = state[0, 0], state[1, 0]

        if isinstance(ax, Axes3D):
            self.owner._text = ax.text(
                x + options.text.position[0],
                y + options.text.position[1],
                self.owner.z,
                self.owner._get_text(),
                fontsize=options.text.size,
                color=options.text.color,
                zorder=options.text.zorder,
                alpha=options.text.alpha,
            )
        else:
            self.owner._text = ax.text(
                x + options.text.position[0],
                y + options.text.position[1],
                self.owner._get_text(),
                fontsize=options.text.size,
                color=options.text.color,
                zorder=options.text.zorder,
                alpha=options.text.alpha,
            )
        self.owner.plot_text_list.append(self.owner._text)

        if (
            getattr(self.owner, "show_goal", options.visibility.goal)
            and getattr(self.owner, "show_goal_text", options.visibility.goal_text)
            and self.owner.goal is not None
        ):
            self._plot_goal_text(ax, options.text)

    def _plot_goal_text(
        self,
        ax: Any,
        style: TextStyle,
    ) -> None:
        """Create the goal label for ``plot_text``."""
        goal_x, goal_y = self.owner.goal[0, 0], self.owner.goal[1, 0]
        args = [
            goal_x + style.position[0],
            goal_y + style.position[1],
        ]
        if isinstance(ax, Axes3D):
            args.append(self.owner.z)

        self.owner._goal_text = ax.text(
            *args,
            self.owner._get_goal_text(),
            fontsize=style.size,
            color=style.color,
            zorder=style.zorder,
            alpha=style.alpha,
        )
        self.owner.plot_text_list.append(self.owner._goal_text)

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
        _options: ObjectPlotOptions | None = None,
        **kwargs: Any,
    ) -> None:
        """Draw an arrow indicating the object's velocity orientation."""
        if _options is None:
            overrides = {
                **kwargs,
                "arrow_length": arrow_length,
                "arrow_width": arrow_width,
                "arrow_zorder": arrow_zorder,
            }
            if arrow_color is not None:
                overrides["arrow_color"] = arrow_color
            options = self._resolve_options(overrides)
        else:
            options = _options
        state = self.owner.state if state is None else state

        self.owner.arrow_patch = draw_patch(
            ax,
            shape="arrow",
            state=state,
            color=options.arrow.color,
            alpha=options.arrow.alpha,
            zorder=options.arrow.zorder,
            arrow_length=options.arrow.length,
            arrow_width=options.arrow.width,
            theta=arrow_theta,
        )
        self.owner.plot_patch_list.append(self.owner.arrow_patch)

    def plot_trail(
        self,
        ax: Any,
        state: np.ndarray | None = None,
        vertices: np.ndarray | None = None,
        keep_trail_length: int = 0,
        _options: ObjectPlotOptions | None = None,
        **kwargs: Any,
    ) -> None:
        """Draw a historical outline of the object."""
        if _options is None:
            options = self._resolve_options(
                {**kwargs, "keep_trail_length": keep_trail_length}
            )
        else:
            options = _options
        vertices = self.owner.original_vertices if vertices is None else vertices

        trail = draw_patch(
            ax,
            shape=options.trail.shape,
            state=state,
            vertices=vertices,
            radius=self.owner.radius,
            center=(
                self.owner.original_centroid
                if options.trail.shape == "circle"
                else None
            ),
            width=self.owner.length,
            height=self.owner.width,
            edgecolor=options.trail.edge_color,
            facecolor=options.trail.color,
            fill=options.trail.fill,
            alpha=options.trail.alpha,
            linewidth=options.trail.line_width,
            zorder=options.trail.zorder,
        )
        self.owner.plot_trail_list.append(trail)

        if (
            len(self.owner.plot_trail_list) > options.trail.keep_length
            and options.trail.keep_length > 0
        ):
            self.owner.plot_trail_list.pop(0).remove()

    def plot_fov(
        self,
        ax: Any,
        _options: ObjectPlotOptions | None = None,
        **kwargs: Any,
    ) -> None:
        """Draw the object's configured field of view."""
        if self.owner.fov is None or self.owner.fov_radius is None:
            return

        options = self._resolve_options(kwargs) if _options is None else _options
        start_degree = -180 * self.owner.fov / (2 * pi)
        end_degree = 180 * self.owner.fov / (2 * pi)
        shape = "circle" if abs(self.owner.fov - 2 * pi) < 0.01 else "wedge"

        self.owner.fov_patch = draw_patch(
            ax,
            shape=shape,
            state=np.zeros((3, 1)),
            radius=self.owner.fov_radius,
            theta1=start_degree,
            theta2=end_degree,
            facecolor=options.fov.color,
            edgecolor=options.fov.edge_color,
            alpha=options.fov.alpha,
            zorder=options.fov.zorder,
        )
        self.owner.plot_patch_list.append(self.owner.fov_patch)

    def plot_uncertainty(self, ax: Any, **kwargs: Any) -> None:
        """Reserved for future uncertainty rendering."""

    def clear(self, all: bool = False) -> None:
        """Remove this object's artists, optionally including its trails."""
        for patch in self.owner.plot_patch_list:
            patch.remove()
        for line in self.owner.plot_line_list:
            line.pop(0).remove()
        for text in self.owner.plot_text_list:
            text.remove()

        if all:
            for trail in self.owner.plot_trail_list:
                trail.remove()
            self.owner.plot_trail_list = []

        self.owner.plot_patch_list = []
        self.owner.plot_line_list = []
        self.owner.plot_text_list = []

        for sensor in self.owner.sensors:
            sensor.plot_clear()
