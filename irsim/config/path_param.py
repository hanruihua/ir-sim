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


_current = PathManager()


def bind(instance: PathManager) -> None:
    global _current
    _current = instance


class _Proxy:
    def __getattr__(self, name: str):
        return getattr(_current, name)

    def __setattr__(self, name: str, value):
        setattr(_current, name, value)


path_manager = _Proxy()
