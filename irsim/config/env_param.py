"""
Objects: A list of all objects in the environment
Logger: A logger object to log messages
Platform: The operating system platform
"""

import platform
from typing import List, Optional, Any

objects: List[Any] = []
logger: Optional[Any] = None
GeometryTree: Optional[Any] = None
platform_name: str = platform.system()
