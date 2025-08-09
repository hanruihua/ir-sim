import os
import sys
from typing import Any, Optional, Union

from irsim.env import EnvBase, EnvBase3D

from .version import __version__


def make(
    world_name: Optional[str] = None, projection: Optional[str] = None, **kwargs: Any
) -> Union[EnvBase, EnvBase3D]:
    """
    Create an environment by the given world file and projection.

    This function serves as the main entry point for creating simulation environments.
    It automatically selects between 2D and 3D environments based on the projection parameter.

    Args:
        world_name (str, optional): The name of the world YAML configuration file.
            If not specified, the default name of the current Python script with
            '.yaml' extension will be used.
        projection (str, optional): The projection type of the environment.
            Default is None for 2D environment. If set to "3d", creates a 3D
            plot environment.
        **kwargs: Additional keyword arguments passed to :py:class:`.EnvBase`
            or :py:class:`.EnvBase3D`. Common options include:

            - display (bool): Whether to display the environment visualization
            - save_ani (bool): Whether to save animation
            - log_level (str): Logging level for the environment

    Returns:
        EnvBase: The created environment object. Returns :py:class:`.EnvBase3D`
        if projection is "3d", otherwise returns :py:class:`.EnvBase`.

    Example:
        >>> # Create a 2D environment with default world file
        >>> env = make()
        >>>
        >>> # Create a 3D environment with custom world file
        >>> env = make("my_world.yaml", projection="3d")
        >>>
        >>> # Create environment with additional options
        >>> env = make("world.yaml", display=True, save_ani=False)
    """

    world_name = world_name or os.path.basename(sys.argv[0]).split(".")[0] + ".yaml"

    if projection == "3d":
        return EnvBase3D(world_name, **kwargs)
    return EnvBase(world_name, **kwargs)
