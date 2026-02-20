"""
Objects: A list of all objects in the environment
Logger: A logger object to log messages
Platform: The operating system platform
"""

import platform
from dataclasses import dataclass, field
from typing import Any

from irsim.world.object_base import ObjectBase


@dataclass
class EnvParam:
    objects: list[ObjectBase] = field(default_factory=list)
    logger: Any | None = None
    GeometryTree: Any | None = None
    platform_name: str = field(default_factory=platform.system)


# Multi-env storage (default index 0)
_instances: list[EnvParam] = [EnvParam()]
_current = _instances[0]


def bind(instance: EnvParam) -> None:
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


def __getitem__(index: int) -> EnvParam:
    return _instances[index]


def __setitem__(index: int, instance: EnvParam) -> None:
    """Assign an EnvParam at a specific index. Extends list if needed."""
    global _current
    if index < 0:
        raise IndexError("env_param index must be non-negative")
    if index >= len(_instances):
        _instances.extend(EnvParam() for _ in range(index - len(_instances) + 1))
    _instances[index] = instance
    if index == 0:
        _current = instance
