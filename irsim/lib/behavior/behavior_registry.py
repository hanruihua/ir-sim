from typing import Any, Callable

# Behavior registries
behaviors_map: dict[tuple[str, str], Callable[..., Any]] = {}
behaviors_class_map: dict[tuple[str, str], Callable[..., Any]] = {}

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


# Per-step function behaviors
register_behavior = _make_register(
    behaviors_map,
    "Method for category '{kinematics}' and action '{action}' is already registered.",
)
register_group_behavior = _make_register(
    group_behaviors_map,
    "Method for category '{kinematics}' and action '{action}' is already registered.",
)

# Class-based handlers (initialized once, then __call__ per step)
register_behavior_class = _make_register(
    behaviors_class_map,
    "Class for category '{kinematics}' and action '{action}' is already registered.",
)
register_group_behavior_class = _make_register(
    group_behaviors_class_map,
    "Class for category '{kinematics}' and action '{action}' is already registered.",
)
