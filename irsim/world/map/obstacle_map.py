from typing import Any, Optional

from shapely.strtree import STRtree

from irsim.world.object_base import ObjectBase


class ObstacleMap(ObjectBase):
    def __init__(
        self,
        shape: Optional[dict] = None,
        color: str = "k",
        static: bool = True,
        **kwargs: Any,
    ) -> None:
        """Create an obstacle map object from a set of line segments.

        Args:
            shape (dict | None): Map shape configuration with keys like
                ``{"name": "map", "reso": float, "points": array}``.
            color (str): Display color. Default "k".
            static (bool): Whether the object is static. Default True.
            **kwargs: Forwarded to ``ObjectBase`` constructor.
        """
        if shape is None:
            shape = {"name": "map", "reso": "0.1", "points": None}
        super().__init__(
            shape=shape,
            role="obstacle",
            color=color,
            static=static,
            **kwargs,
        )

        self.linestrings = list(self.geometry.geoms)
        self.geometry_tree = STRtree(self.linestrings)
