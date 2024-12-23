from .version import __version__
from irsim.env import EnvBase, EnvBase3D
import os
import sys


def make(world_name=None, projection=None, **kwargs) -> EnvBase:

    '''
    Create an environment by the given world file and projection.

    Args:
        world_name (str): The name of the world file. If not specified, the default name of the `python script` will be used.
        projection (str): The projection of the environment. Default is "None". If set to "3d", the environment will be a 3D plot environment.
        **kwargs: Additional keyword arguments for 
            :py:class:`.EnvBase` or :py:class:`.EnvBase3D` for more details. 
    Returns:
        The environment object
    '''

    world_name = world_name or os.path.basename(sys.argv[0]).split(".")[0] + ".yaml"

    if projection == "3d":
        return EnvBase3D(world_name, **kwargs)
    else:
        return EnvBase(world_name, **kwargs)
