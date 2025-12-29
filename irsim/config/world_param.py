"""
world parameters:
    time: time elapse of the simulation
    control_mode:
        - auto: robot will be controlled automatically
        - keyboard: robot will be controlled by keyboard

    collision_mode:
        - stop (default): All Objects stop when collision,
        - unobstructed: No collision check
        - unobstructed_obstacles: Only allows obstacles to pass through each other without consideration of any collision. The robots will stop when they are in collision with the obstacles.

    step_time: time of the simulation step, default is 0.1
    count: count of the simulation, time = count * step_time

"""

from dataclasses import dataclass


@dataclass
class WorldParam:
    time: float = 0.0
    control_mode: str = "auto"
    collision_mode: str = "stop"
    step_time: float = 0.1
    count: int = 0


# Multi-env storage (default index 0)
_instances: list[WorldParam] = [WorldParam()]
_current = _instances[0]


def bind(instance: WorldParam) -> None:
    """Bind instance to default index 0 and update current alias."""
    global _current
    if _instances:
        _instances[0] = instance
    else:
        _instances.append(instance)
    _current = instance


def __getattr__(name: str):
    return getattr(_current, name)


def __setattr__(name: str, value):
    setattr(_current, name, value)


def __getitem__(index: int) -> WorldParam:
    return _instances[index]


def __setitem__(index: int, instance: WorldParam) -> None:
    """Assign a WorldParam at a specific index. Extends list if needed."""
    global _current
    if index < 0:
        raise IndexError("world_param index must be non-negative")
    if index >= len(_instances):
        _instances.extend(WorldParam() for _ in range(index - len(_instances) + 1))
    _instances[index] = instance
    if index == 0:
        _current = instance
