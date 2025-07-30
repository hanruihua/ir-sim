"""
Global parameters for IR-SIM simulation environment.

This package contains configuration parameters for:
- env_param: Environment parameters
- world_param: World parameters
- path_param: Path parameters
"""

from . import env_param
from . import world_param
from . import path_param

__all__ = ["env_param", "world_param", "path_param"]
