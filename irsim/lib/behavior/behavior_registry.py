from typing import Any, Callable

behaviors_map: dict[tuple[str, str], Callable[..., Any]] = {}
group_behaviors_map: dict[tuple[str, str], Callable[..., Any]] = {}


def register_behavior(kinematics: str, action: str):
    """
    decorator to register a method in the behavior registry

    Args:
        kinematics: only support diff, omni, or acker
        action: defined action for the kinematics

    Returns:
        Callable: A decorator that registers the target function under the
        (kinematics, action) key and returns the function unchanged.
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


def register_group_behavior(kinematics: str, action: str):
    """
    decorator to register a method in the group behavior registry

    Args:
        kinematics: only support diff, omni, or acker
        action: defined action for the group with the same kinematics

    Returns:
        Callable: A decorator that registers the target function under the
        (kinematics, action) key and returns the function unchanged.
    """

    def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
        key = (kinematics, action)
        if key in group_behaviors_map:
            raise ValueError(
                f"Method for category '{kinematics}' and action '{action}' is already registered."
            )
        group_behaviors_map[key] = func
        return func

    return decorator
