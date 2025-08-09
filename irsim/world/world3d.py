from typing import Any, Optional

from irsim.world.world import World


class World3D(World):
    def __init__(
        self,
        name: str,
        depth: float = 10.0,
        offset: Optional[list[float]] = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(name=name, **kwargs)

        self.depth = depth

        if offset is None:
            offset = [0, 0, 0]
        self.offset = offset if len(offset) == 3 else [offset[0], offset[1], 0]

        self.z_range = [self.offset[2], self.offset[2] + self.depth]
