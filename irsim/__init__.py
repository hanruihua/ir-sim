from .version import __version__
from irsim.env import EnvBase


def make(world_name, **kwargs):
    return EnvBase(world_name, **kwargs)
