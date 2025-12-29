"""
Objects: A list of all objects in the environment
Logger: A logger object to log messages
Platform: The operating system platform
"""

import platform
from dataclasses import dataclass, field
from typing import Any, Optional

from irsim.world.object_base import ObjectBase


@dataclass
class EnvParam:
    objects: list[ObjectBase] = field(default_factory=list)
    logger: Optional[Any] = None
    GeometryTree: Optional[Any] = None
    platform_name: str = field(default_factory=platform.system)


_current = EnvParam()


def bind(instance: EnvParam) -> None:
    global _current
    _current = instance


def __getattr__(name: str):
    return getattr(_current, name)


def __setattr__(name: str, value):
    setattr(_current, name, value)
