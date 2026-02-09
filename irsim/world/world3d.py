from typing import Any

from irsim.world.world import World


class World3D(World):
    def __init__(
        self,
        name: str,
        depth: float = 10.0,
        offset: list[float] | None = None,
        **kwargs: Any,
    ) -> None:
        """Initialize a 3D world extending the 2D world with depth.

        Args:
            name (str): World name or YAML file path.
            depth (float): Z-depth of the world (range in z). Default 10.0.
            offset (list[float] | None): [x, y, z] world offset. If a 2D
                [x, y] is provided, z defaults to 0.
            **kwargs: Forwarded to the base ``World`` constructor.
        """
        super().__init__(name=name, **kwargs)

        self.depth = depth

        if offset is None:
            offset = [0, 0, 0]
        self.offset = offset if len(offset) == 3 else [offset[0], offset[1], 0]

        self.z_range = [self.offset[2], self.offset[2] + self.depth]
