from typing import Any, Callable

behaviors_map: dict[tuple[str, str], Callable[..., Any]] = {}


def register_behavior(kinematics: str, action: str):
    """
    decorator to register a method in the behavior registry

    Args:
        kinematics: only support diff, omni, or acker
        action: defined action for the kinematics
    """

    def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
        key = (kinematics, action)
        if key in behaviors_map:
            raise ValueError(
                f"Method for category '{kinematics}' and action '{action}' is already registered."
            )
        behaviors_map[key] = func
        return func

    return decorator
