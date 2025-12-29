import os
import sys
from dataclasses import dataclass
from typing import cast

import irsim


@dataclass
class PathManager:
    """
    Module for managing the path of the project.
        - root_path: path of the irsim package
        - ani_buffer_path: path of the animation buffer
        - ani_path: path of the animation
        - fig_path: path of the saved figure
    """

    root_path: str = os.path.dirname(cast(str, irsim.__file__))
    ani_buffer_path: str = sys.path[0] + "/animation_buffer"
    ani_path: str = sys.path[0] + "/animation"
    fig_path: str = sys.path[0] + "/figure"


# Multi-env storage (default index 0)
_instances: list[PathManager] = [PathManager()]
_current = _instances[0]


def bind(instance: PathManager) -> None:
    """Bind instance to default index 0 and update current alias."""
    global _current
    if _instances:
        _instances[0] = instance
    else:
        _instances.append(instance)
    _current = instance


class _Proxy:
    def __getattr__(self, name: str):
        return getattr(_current, name)

    def __setattr__(self, name: str, value):
        setattr(_current, name, value)


path_manager = _Proxy()


def __getitem__(index: int) -> PathManager:
    return _instances[index]


def __setitem__(index: int, instance: PathManager) -> None:
    """Assign a PathManager at a specific index. Extends list if needed."""
    global _current
    if index < 0:
        raise IndexError("path_param index must be non-negative")
    if index >= len(_instances):
        _instances.extend(PathManager() for _ in range(index - len(_instances) + 1))
    _instances[index] = instance
    if index == 0:
        _current = instance
