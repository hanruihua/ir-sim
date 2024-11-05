from .version import __version__
from irsim.env import EnvBase
import os
import sys


def make(world_name=None, **kwargs) -> EnvBase:

    world_name = world_name or os.path.basename(sys.argv[0]).split(".")[0] + ".yaml"

    return EnvBase(world_name, **kwargs)
