from collections.abc import Callable
from typing import Any

# Behavior registries
behaviors_map: dict[tuple[str, str], Callable[..., Any]] = {}
behaviors_class_map: dict[tuple[str, str], Callable[..., Any]] = {}

# Keep capabilities next to registrations, not on the callable: one callable
# may serve several registrations, and plugins may replace map entries directly.
behavior_neighbor_requirements: dict[tuple[str, str], tuple[Any, bool]] = {}
behavior_class_neighbor_requirements: dict[tuple[str, str], tuple[Any, bool]] = {}

group_behaviors_map: dict[tuple[str, str], Callable[..., Any]] = {}
group_behaviors_class_map: dict[tuple[str, str], Callable[..., Any]] = {}


def _make_register(
    target_map: dict[tuple[str, str], Any], duplicate_fmt: str
) -> Callable[[str, str], Callable[[Any], Any]]:
    """
    Create a decorator that registers objects (functions or classes) into a map
    keyed by (kinematics, action).
    """

    def register(kinematics: str, action: str):
        def decorator(obj: Any) -> Any:
            key = (kinematics, action)
            if key in target_map:
                raise ValueError(
                    duplicate_fmt.format(kinematics=kinematics, action=action)
                )
            target_map[key] = obj
            return obj

        return decorator

    return register


def _with_neighbor_requirement(
    register: Callable[[str, str], Callable[[Any], Any]],
    requirements: dict[tuple[str, str], tuple[Any, bool]],
) -> Callable[..., Callable[[Any], Any]]:
    """Let individual behaviors opt out of the otherwise unchanged neighbour list."""

    def register_with_neighbors(
        kinematics: str, action: str, *, requires_neighbors: bool = True
    ):
        register_object = register(kinematics, action)

        def decorator(obj: Any) -> Any:
            registered = register_object(obj)
            requirements[kinematics, action] = (registered, requires_neighbors)
            return registered

        return decorator

    return register_with_neighbors


# Per-step function behaviors
register_behavior = _with_neighbor_requirement(
    _make_register(
        behaviors_map,
        "Method for category '{kinematics}' and action '{action}' is already registered.",
    ),
    behavior_neighbor_requirements,
)
register_group_behavior = _make_register(
    group_behaviors_map,
    "Method for category '{kinematics}' and action '{action}' is already registered.",
)

# Class-based handlers (initialized once, then __call__ per step)
register_behavior_class = _with_neighbor_requirement(
    _make_register(
        behaviors_class_map,
        "Class for category '{kinematics}' and action '{action}' is already registered.",
    ),
    behavior_class_neighbor_requirements,
)
register_group_behavior_class = _make_register(
    group_behaviors_class_map,
    "Class for category '{kinematics}' and action '{action}' is already registered.",
)
