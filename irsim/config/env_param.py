"""
Objects: A list of all objects in the environment
Logger: A logger object to log messages
Platform: The operating system platform
"""

import platform
from typing import Any, Optional

from irsim.world.object_base import ObjectBase

objects: list[ObjectBase] = []
logger: Optional[Any] = None
GeometryTree: Optional[Any] = None
platform_name: str = platform.system()
