import os
import sys
from typing import Any, Callable, Optional

from irsim.env import EnvBase, EnvBase3D

from .version import __version__


class _EnvFactory:
    """
    Internal object-oriented factory that creates IR-SIM environments.

    Create an environment by the given world file and projection.

    Env candidates:
        - EnvBase (2D default)
        - EnvBase3D (3D)
    """

    def __init__(
        self, default_projection: Optional[str] = None, **default_kwargs: Any
    ) -> None:
        self.default_projection = default_projection
        self.default_kwargs = default_kwargs

        self._registry: dict[str, Callable[..., EnvBase]] = {
            "2d": EnvBase,
            "3d": EnvBase3D,
        }

    def _resolve_world_name(self, world_name: Optional[str]) -> str:
        return world_name or os.path.basename(sys.argv[0]).split(".")[0] + ".yaml"

    def register(self, key: str, ctor: Callable[..., EnvBase]) -> None:
        self._registry[key.strip().lower()] = ctor

    def create(
        self,
        world_name: Optional[str] = None,
        projection: Optional[str] = None,
        **kwargs: Any,
    ) -> EnvBase:
        resolved_world = self._resolve_world_name(world_name)
        options: dict[str, Any] = {**self.default_kwargs, **kwargs}
        key = (projection or self.default_projection or "2d").strip().lower()
        try:
            ctor = self._registry[key]
        except KeyError as e:
            raise ValueError(
                f"Unknown projection {projection!r}. Allowed: {', '.join(self._registry)}"
            ) from e
        return ctor(resolved_world, **options)


_env_factory = _EnvFactory()


def make(
    world_name: Optional[str] = None, projection: Optional[str] = None, **kwargs: Any
) -> EnvBase:
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
            - seed (int, optional): Seed for IR-SIM's random number generator
              to make runs reproducible when using IR-SIM randomness.

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
    return _env_factory.create(world_name=world_name, projection=projection, **kwargs)
