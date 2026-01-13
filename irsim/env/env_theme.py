"""
PlotStyle dataclass hierarchy for organizing object visualization properties.

This module provides a structured way to configure and modify plotting properties
for objects in the simulator. It supports both the legacy flat kwargs format
and a new nested format for cleaner configuration.

Author: Ruihua Han
"""

import os
from dataclasses import asdict, dataclass, field
from typing import Any, Optional

import yaml


@dataclass
class ObjStyle:
    """Style properties for the object itself."""

    color: Optional[str] = None  # Falls back to object.color
    edgecolor: Optional[str] = None  # Edge color, set to "none" to hide edges
    alpha: float = 1.0
    linestyle: str = "-"
    zorder: Optional[int] = None  # Falls back to 3 for robots, 1 for obstacles

    def to_kwargs(self) -> dict[str, Any]:
        """Convert to kwargs with 'obj_' prefix for backward compatibility."""
        return {
            "obj_color": self.color,
            "obj_edgecolor": self.edgecolor,
            "obj_alpha": self.alpha,
            "obj_linestyle": self.linestyle,
            "obj_zorder": self.zorder,
        }


@dataclass
class GoalStyle:
    """Style properties for goal visualization."""

    show: bool = False
    show_text: bool = False
    show_all: bool = False  # show_goals - show multiple goals
    color: Optional[str] = None  # Falls back to object color
    edgecolor: Optional[str] = None  # Edge color, set to "none" to hide edges
    alpha: float = 0.5
    zorder: int = 1

    def to_kwargs(self) -> dict[str, Any]:
        """Convert to kwargs for backward compatibility."""
        return {
            "show_goal": self.show,
            "show_goal_text": self.show_text,
            "show_goals": self.show_all,
            "goal_color": self.color,
            "goal_edgecolor": self.edgecolor,
            "goal_alpha": self.alpha,
            "goal_zorder": self.zorder,
        }


@dataclass
class TextStyle:
    """Style properties for text labels."""

    show: bool = False
    color: str = "k"
    size: int = 10
    position: list[float] = field(default_factory=lambda: [-0.1, 0.1])
    zorder: int = 2
    alpha: float = 1.0

    def to_kwargs(self) -> dict[str, Any]:
        """Convert to kwargs for backward compatibility."""
        return {
            "show_text": self.show,
            "text_color": self.color,
            "text_size": self.size,
            "text_position": self.position,
            "text_zorder": self.zorder,
            "text_alpha": self.alpha,
        }


@dataclass
class ArrowStyle:
    """Style properties for velocity arrow."""

    show: bool = False
    color: str = "gold"
    length: float = 0.4
    width: float = 0.6
    zorder: int = 4
    alpha: float = 1.0

    def to_kwargs(self) -> dict[str, Any]:
        """Convert to kwargs for backward compatibility."""
        return {
            "show_arrow": self.show,
            "arrow_color": self.color,
            "arrow_length": self.length,
            "arrow_width": self.width,
            "arrow_zorder": self.zorder,
            "arrow_alpha": self.alpha,
        }


@dataclass
class TrajectoryStyle:
    """Style properties for trajectory line."""

    show: bool = False
    color: Optional[str] = None  # Falls back to object color
    style: str = "-"
    width: Optional[float] = None  # Falls back to object width
    alpha: float = 0.5
    zorder: int = 0
    keep_length: int = 0  # Number of points to keep (0 = all)

    def to_kwargs(self) -> dict[str, Any]:
        """Convert to kwargs for backward compatibility."""
        return {
            "show_trajectory": self.show,
            "traj_color": self.color,
            "traj_style": self.style,
            "traj_width": self.width,
            "traj_alpha": self.alpha,
            "traj_zorder": self.zorder,
            "keep_traj_length": self.keep_length,
        }


@dataclass
class TrailStyle:
    """Style properties for object trail."""

    show: bool = False
    freq: int = 2  # Show trail every N steps
    color: Optional[str] = None  # Falls back to object color (fill)
    edgecolor: Optional[str] = None  # Falls back to object color
    linewidth: float = 0.8
    alpha: float = 0.7
    fill: bool = False
    zorder: int = 0
    keep_length: int = 0  # Number of trail elements to keep (0 = unlimited)
    type: Optional[str] = None  # Trail shape type (None = use object shape)

    def to_kwargs(self) -> dict[str, Any]:
        """Convert to kwargs for backward compatibility."""
        return {
            "show_trail": self.show,
            "trail_freq": self.freq,
            "trail_color": self.color,
            "trail_edgecolor": self.edgecolor,
            "trail_linewidth": self.linewidth,
            "trail_alpha": self.alpha,
            "trail_fill": self.fill,
            "trail_zorder": self.zorder,
            "keep_trail_length": self.keep_length,
            "trail_type": self.type,
        }


@dataclass
class FovStyle:
    """Style properties for field of view visualization."""

    show: bool = False
    color: str = "lightblue"
    edge_color: str = "blue"
    alpha: float = 0.5
    zorder: int = 1

    def to_kwargs(self) -> dict[str, Any]:
        """Convert to kwargs for backward compatibility."""
        return {
            "show_fov": self.show,
            "fov_color": self.color,
            "fov_edge_color": self.edge_color,
            "fov_alpha": self.alpha,
            "fov_zorder": self.zorder,
        }


@dataclass
class SensorStyle:
    """Style properties for sensor visualization."""

    show: bool = True

    def to_kwargs(self) -> dict[str, Any]:
        """Convert to kwargs for backward compatibility."""
        return {"show_sensor": self.show}


@dataclass
class PlotStyle:
    """
    Main container for all plot style properties.

    This class organizes all visualization properties for an object into
    logical groups. It supports both programmatic configuration and
    creation from YAML configuration dictionaries.

    Example:
        # Programmatic usage
        style = PlotStyle()
        style.goal.show = True
        style.goal.color = "red"
        style.arrow.show = True

        # From YAML dict (old flat format)
        style = PlotStyle.from_dict({"show_goal": True, "goal_color": "red"})

        # From YAML dict (new nested format)
        style = PlotStyle.from_dict({"goal": {"show": True, "color": "red"}})
    """

    obj: ObjStyle = field(default_factory=ObjStyle)
    goal: GoalStyle = field(default_factory=GoalStyle)
    text: TextStyle = field(default_factory=TextStyle)
    arrow: ArrowStyle = field(default_factory=ArrowStyle)
    trajectory: TrajectoryStyle = field(default_factory=TrajectoryStyle)
    trail: TrailStyle = field(default_factory=TrailStyle)
    fov: FovStyle = field(default_factory=FovStyle)
    sensor: SensorStyle = field(default_factory=SensorStyle)

    def to_kwargs(self) -> dict[str, Any]:
        """Convert entire PlotStyle to flat kwargs dict for backward compatibility."""
        result = {}
        result.update(self.obj.to_kwargs())
        result.update(self.goal.to_kwargs())
        result.update(self.text.to_kwargs())
        result.update(self.arrow.to_kwargs())
        result.update(self.trajectory.to_kwargs())
        result.update(self.trail.to_kwargs())
        result.update(self.fov.to_kwargs())
        result.update(self.sensor.to_kwargs())
        return result

    def to_dict(self) -> dict[str, Any]:
        """Convert to nested dictionary representation."""
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "PlotStyle":
        """
        Create PlotStyle from a kwargs dict (YAML compatibility).

        Supports both old flat format:
            {"show_goal": True, "goal_color": "red", "show_arrow": True}

        And new nested format:
            {"goal": {"show": True, "color": "red"}, "arrow": {"show": True}}

        Args:
            d: Dictionary with plot configuration.

        Returns:
            PlotStyle instance.
        """
        # Check if using new nested format
        nested_keys = ("obj", "goal", "text", "arrow", "trajectory", "trail", "fov", "sensor")
        if any(key in d for key in nested_keys):
            return cls(
                obj=ObjStyle(**{k: v for k, v in d.get("obj", {}).items() if k in ObjStyle.__dataclass_fields__}),
                goal=GoalStyle(**{k: v for k, v in d.get("goal", {}).items() if k in GoalStyle.__dataclass_fields__}),
                text=TextStyle(**{k: v for k, v in d.get("text", {}).items() if k in TextStyle.__dataclass_fields__}),
                arrow=ArrowStyle(**{k: v for k, v in d.get("arrow", {}).items() if k in ArrowStyle.__dataclass_fields__}),
                trajectory=TrajectoryStyle(**{k: v for k, v in d.get("trajectory", {}).items() if k in TrajectoryStyle.__dataclass_fields__}),
                trail=TrailStyle(**{k: v for k, v in d.get("trail", {}).items() if k in TrailStyle.__dataclass_fields__}),
                fov=FovStyle(**{k: v for k, v in d.get("fov", {}).items() if k in FovStyle.__dataclass_fields__}),
                sensor=SensorStyle(**{k: v for k, v in d.get("sensor", {}).items() if k in SensorStyle.__dataclass_fields__}),
            )

        # Parse flat format (backward compatibility)
        return cls(
            obj=ObjStyle(
                color=d.get("obj_color"),
                edgecolor=d.get("obj_edgecolor"),
                alpha=d.get("obj_alpha", 1.0),
                linestyle=d.get("obj_linestyle", "-"),
                zorder=d.get("obj_zorder"),
            ),
            goal=GoalStyle(
                show=d.get("show_goal", False),
                show_text=d.get("show_goal_text", False),
                show_all=d.get("show_goals", False),
                color=d.get("goal_color"),
                edgecolor=d.get("goal_edgecolor"),
                alpha=d.get("goal_alpha", 0.5),
                zorder=d.get("goal_zorder", 1),
            ),
            text=TextStyle(
                show=d.get("show_text", False),
                color=d.get("text_color", "k"),
                size=d.get("text_size", 10),
                position=d.get("text_position", [-0.1, 0.1]),
                zorder=d.get("text_zorder", 2),
                alpha=d.get("text_alpha", 1.0),
            ),
            arrow=ArrowStyle(
                show=d.get("show_arrow", False),
                color=d.get("arrow_color", "gold"),
                length=d.get("arrow_length", 0.4),
                width=d.get("arrow_width", 0.6),
                zorder=d.get("arrow_zorder", 4),
                alpha=d.get("arrow_alpha", 1.0),
            ),
            trajectory=TrajectoryStyle(
                show=d.get("show_trajectory", False),
                color=d.get("traj_color"),
                style=d.get("traj_style", "-"),
                width=d.get("traj_width"),
                alpha=d.get("traj_alpha", 0.5),
                zorder=d.get("traj_zorder", 0),
                keep_length=d.get("keep_traj_length", 0),
            ),
            trail=TrailStyle(
                show=d.get("show_trail", False),
                freq=d.get("trail_freq", 2),
                color=d.get("trail_color"),
                edgecolor=d.get("trail_edgecolor"),
                linewidth=d.get("trail_linewidth", 0.8),
                alpha=d.get("trail_alpha", 0.7),
                fill=d.get("trail_fill", False),
                zorder=d.get("trail_zorder", 0),
                keep_length=d.get("keep_trail_length", 0),
                type=d.get("trail_type"),
            ),
            fov=FovStyle(
                show=d.get("show_fov", False),
                color=d.get("fov_color", "lightblue"),
                edge_color=d.get("fov_edge_color", "blue"),
                alpha=d.get("fov_alpha", 0.5),
                zorder=d.get("fov_zorder", 1),
            ),
            sensor=SensorStyle(
                show=d.get("show_sensor", True),
            ),
        )

    def update(self, **kwargs: Any) -> "PlotStyle":
        """
        Update style with flat kwargs, returning a new PlotStyle.

        This method supports the same flat kwargs format used in the legacy API,
        making it easy to apply runtime overrides.

        Args:
            **kwargs: Flat kwargs to update (e.g., show_goal=True, goal_color="red").

        Returns:
            New PlotStyle with updates applied.
        """
        # Convert current to dict, update, then recreate
        current = self.to_kwargs()
        current.update(kwargs)
        return PlotStyle.from_dict(current)

    def merge_kwargs(self, **kwargs: Any) -> dict[str, Any]:
        """
        Merge current style with kwargs, returning a combined kwargs dict.

        This is useful for getting the effective kwargs when both style
        properties and runtime kwargs need to be considered.

        Args:
            **kwargs: Additional kwargs to merge.

        Returns:
            Combined kwargs dictionary.
        """
        result = self.to_kwargs()
        result.update(kwargs)
        return result


# Path to built-in themes directory
_THEMES_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "themes")


@dataclass
class EnvTheme:
    """
    Global environment theme for controlling all visualization styles.

    This class provides centralized theme configuration that applies default
    styles to all objects in the environment. Individual object styles can
    still override these defaults.

    Attributes:
        background_color: Background color of the figure (hex or named color).
        grid_cmap: Colormap for grid map visualization.
        grid_alpha: Alpha (transparency) for grid map.
        grid_zorder: Z-order (drawing layer) for grid map.
        robot: Default PlotStyle for robot objects.
        obstacle: Default PlotStyle for obstacle objects.

    Example:
        # Load built-in theme
        theme = EnvTheme.load("dark")

        # Load custom theme from file
        theme = EnvTheme.load("./my_theme.yaml")

        # Create programmatically
        theme = EnvTheme(background_color="#1a1a2e", grid_cmap="viridis")
    """

    # Environment-level styles
    background_color: str = "#ffffff"
    grid_cmap: str = "viridis"
    grid_alpha: float = 1.0
    grid_zorder: int = 0

    # Default styles for object types
    robot: PlotStyle = field(default_factory=PlotStyle)
    obstacle: PlotStyle = field(default_factory=PlotStyle)

    def to_dict(self) -> dict[str, Any]:
        """Convert theme to nested dictionary representation."""
        return {
            "background_color": self.background_color,
            "grid_cmap": self.grid_cmap,
            "grid_alpha": self.grid_alpha,
            "grid_zorder": self.grid_zorder,
            "robot": self.robot.to_dict(),
            "obstacle": self.obstacle.to_dict(),
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "EnvTheme":
        """
        Create EnvTheme from a dictionary.

        Args:
            d: Dictionary with theme configuration.

        Returns:
            EnvTheme instance.
        """
        robot_dict = d.get("robot", {})
        obstacle_dict = d.get("obstacle", {})

        return cls(
            background_color=d.get("background_color", "#ffffff"),
            grid_cmap=d.get("grid_cmap", "viridis"),
            grid_alpha=d.get("grid_alpha", 1.0),
            grid_zorder=d.get("grid_zorder", 0),
            robot=PlotStyle.from_dict(robot_dict) if robot_dict else PlotStyle(),
            obstacle=PlotStyle.from_dict(obstacle_dict) if obstacle_dict else PlotStyle(),
        )

    @classmethod
    def load(cls, theme_name_or_path: str) -> "EnvTheme":
        """
        Load theme from a built-in name or file path.

        Args:
            theme_name_or_path: Either a built-in theme name (e.g., "default", "dark")
                or a path to a custom YAML theme file.

        Returns:
            EnvTheme instance.

        Raises:
            FileNotFoundError: If the theme file cannot be found.
        """
        # Check if it's a file path
        if os.path.isfile(theme_name_or_path):
            theme_path = theme_name_or_path
        else:
            # Try built-in themes
            theme_path = os.path.join(_THEMES_DIR, f"{theme_name_or_path}.yaml")
            if not os.path.isfile(theme_path):
                raise FileNotFoundError(
                    f"Theme '{theme_name_or_path}' not found. "
                    f"Available built-in themes: {cls.list_themes()}"
                )

        with open(theme_path) as f:
            theme_dict = yaml.safe_load(f) or {}

        return cls.from_dict(theme_dict)

    @classmethod
    def list_themes(cls) -> list[str]:
        """
        List available built-in theme names.

        Returns:
            List of theme names (without .yaml extension).
        """
        if not os.path.isdir(_THEMES_DIR):
            return []
        return [
            f[:-5] for f in os.listdir(_THEMES_DIR)
            if f.endswith(".yaml")
        ]

    def get_robot_style(self) -> PlotStyle:
        """Get default PlotStyle for robots."""
        return self.robot

    def get_obstacle_style(self) -> PlotStyle:
        """Get default PlotStyle for obstacles."""
        return self.obstacle
