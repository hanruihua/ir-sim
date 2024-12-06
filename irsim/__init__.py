from .version import __version__
from irsim.env import EnvBase, EnvBase3D
import os
import sys


def make(world_name=None, projection=None, **kwargs) -> EnvBase:

    world_name = world_name or os.path.basename(sys.argv[0]).split(".")[0] + ".yaml"

    if projection == "3d":
        return EnvBase3D(world_name, **kwargs)
    else:
        return EnvBase(world_name, **kwargs)
