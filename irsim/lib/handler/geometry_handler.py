from abc import ABC, abstractmethod
from typing import ClassVar

import numpy as np
import shapely as shp
from shapely import (
    LineString,
    MultiLineString,
    Point,
    Polygon,
    bounds,
    envelope,
    is_valid,
    make_valid,
    minimum_bounding_radius,
)
from shapely.ops import transform, unary_union

from irsim.lib import random_generate_polygon
from irsim.util.random import rng
from irsim.util.util import (
    gen_inequal_from_vertex,
    geometry_transform,
    get_transform,
    is_convex_and_ordered,
    log_warning,
)


class geometry_handler(ABC):
    """
    This class is used to handle the geometry of the object. It reads the shape parameters from yaml file and constructs the geometry of the object.
    """

    def __init__(self, name: str, **kwargs):
        self.name = name
        self._original_geometry = self.construct_original_geometry(**kwargs)
        self.geometry = self._original_geometry
        self.wheelbase = kwargs.get("wheelbase")
        self.length, self.width = self.cal_length_width(self._original_geometry)

    @abstractmethod
    def construct_original_geometry(self, **kwargs):
        """
        Construct the original geometry of the object when the state is in the origin of coordinates.

        Args:
            **kwargs: shape parameters

        Returns: Geometry of the object
        """
        pass

    def step(self, state):
        """
        Transform geometry to the new state.

        Args:
            state (np.ndarray): State vector [x, y, theta].

        Returns:
            shapely.geometry.base.BaseGeometry: Transformed geometry.
        """

        self.geometry = geometry_transform(self._original_geometry, state)

        return self.geometry

    def get_init_Gh(self):
        """
        Generate initial G and h for convex object.

        Returns:
            tuple[np.ndarray, np.ndarray, str | None, bool | None]:
                - G matrix: (N, 2)
                - h vector: (N, 1)
                - cone_type (str): "norm2" for circle or "Rpositive" for polygon
                - convex_flag (bool): whether convex constraints are valid
        """

        if self.name == "circle":
            # The circle may be offset from the body origin (center/wheelbase).
            G = np.array([[1, 0], [0, 1], [0, 0]])
            h = np.vstack((self.original_centroid, [[-self.radius]]))
            cone_type = "norm2"
            convex_flag = True

        elif self.name == "polygon" or self.name == "rectangle":
            convex_flag, _ = is_convex_and_ordered(self.original_vertices)

            if convex_flag:
                G, h = gen_inequal_from_vertex(self.original_vertices)
                cone_type = "Rpositive"
            else:
                G, h, cone_type = None, None, None
        elif self.name == "compound":
            G, h, cone_type, convex_flag = None, None, None, False
        else:
            G, h, cone_type, convex_flag = None, None, None, None

        return G, h, cone_type, convex_flag

    def get_Gh(self, **kwargs):
        """Generate convex constraint matrices for the current shape type.

        Args:
            **kwargs: Shape-specific data such as ``vertices``, ``center``, or
                ``radius``. If omitted, callers should use :meth:`get_init_Gh`
                for constraints from the original geometry.

        Returns:
            tuple: ``(G, h, cone_type, convex_flag)`` where unavailable
            constraints are returned as ``None`` values.
        """
        G, h, cone_type, convex_flag = None, None, None, None

        if self.name == "polygon" or self.name == "rectangle":
            G, h, cone_type, convex_flag = self.get_polygon_Gh(kwargs.get("vertices"))

        elif self.name == "circle":
            G, h, cone_type, convex_flag = self.get_circle_Gh(
                kwargs.get("center"), kwargs.get("radius")
            )

        elif self.name == "compound":
            convex_flag = False

        return G, h, cone_type, convex_flag

    def get_polygon_Gh(self, vertices: np.ndarray | None = None):
        """
        Generate G and h for convex polygon.

        Args:
            vertices: (2, N), N: Edge number of the object

        Returns:
            tuple[np.ndarray | None, np.ndarray | None, str | None, bool]:
                - G matrix: (N, 2)
                - h vector: (N, 1)
                - cone_type (str): "Rpositive" for polygon
                - convex_flag (bool): whether convex constraints are valid
        """

        if self.name == "polygon" or self.name == "rectangle":
            if vertices is None:
                return None, None, None, False
            convex_flag, _ = is_convex_and_ordered(vertices)

            if convex_flag:
                G, h = gen_inequal_from_vertex(vertices)
                cone_type = "Rpositive"
            else:
                G, h, cone_type = None, None, None
        else:
            G, h, cone_type = None, None, None

        return G, h, cone_type, convex_flag

    def get_circle_Gh(self, center: np.ndarray, radius: float):
        """
        Generate G and h for circle.

        Args:
            center: (2, 1) array of center
            radius: float of radius

        Returns:
            tuple[np.ndarray, np.ndarray, str, bool]:
                - G matrix: (3, 2)
                - h vector: (3, 1)
                - cone_type (str): "norm2"
                - convex_flag (bool): True
        """

        assert self.name == "circle"

        G = np.array([[1, 0], [0, 1], [0, 0]])
        h = np.vstack((center, -radius * np.ones((1, 1))))
        cone_type = "norm2"
        convex_flag = True

        return G, h, cone_type, convex_flag

    def cal_length_width(self, geometry):
        """Calculate axis-aligned bounding-box length and width.

        Args:
            geometry: Shapely geometry.

        Returns:
            tuple[float, float]: Length in x and width in y.
        """
        min_x, min_y, max_x, max_y = bounds(geometry).tolist()
        length = max_x - min_x
        width = max_y - min_y

        return length, width

    @property
    def vertices(self):
        """Current geometry vertices as a ``(2, N)`` array."""
        if self.name == "linestring":
            x = self.geometry.xy[0]
            y = self.geometry.xy[1]
            return np.c_[x, y].T
        return self.geometry.exterior.coords._coords.T[:, :-1]

    @property
    def init_vertices(self):
        """
        return original_vertices: [[x1, y1], [x2, y2]....    [[x1, y1]]]; [x1, y1] will repeat twice
        """
        assert "this property is renamed to be original_vertices"

    @property
    def original_vertices(self) -> np.ndarray | None:
        """
        Get the original vertices of the geometry.
        """

        if self.name == "linestring":
            x = self._original_geometry.xy[0]
            y = self._original_geometry.xy[1]
            return np.c_[x, y].T

        if self.name == "map":
            return None
        return self._original_geometry.exterior.coords._coords.T[:, :-1]

    @property
    def original_centroid(self) -> np.ndarray:
        """
        Get the original centroid of the geometry.

        Returns:
            np.ndarray: The original centroid of the geometry.
        """
        return np.array(self._original_geometry.centroid.xy)

    @property
    def radius(self):
        """Minimum bounding radius of the original geometry."""
        return minimum_bounding_radius(self._original_geometry)


class CircleGeometry(geometry_handler):
    """Circular geometry handler for ``shape: {name: circle}``."""

    def __init__(self, name: str = "circle", **kwargs):
        super().__init__(name, **kwargs)

    def construct_original_geometry(
        self,
        radius: float = 0.2,
        center: list | None = None,
        random_shape: bool = False,
        radius_range: list | None = None,
        wheelbase: float | None = None,
    ):
        """Construct a circle at the local origin.

        Args:
            radius: Circle radius.
            center: Local center ``[x, y]``.
            random_shape: If True, sample radius from ``radius_range``.
            radius_range: ``[min, max]`` radius range for random shapes.
            wheelbase: Optional Ackermann wheelbase offset for car-like bodies.

        Returns:
            shapely.geometry.Polygon: Buffered point representing the circle.
        """
        if radius_range is None:
            radius_range = [0.1, 1.0]
        if center is None:
            center = [0, 0]
        if random_shape:
            radius = rng.uniform(*radius_range)

        if wheelbase is None:
            return Point(center).buffer(radius)
        return Point([center[0] + wheelbase / 2, center[1]]).buffer(radius)


class PolygonGeometry(geometry_handler):
    """Polygon geometry handler for ``shape: {name: polygon}``."""

    def __init__(self, name: str = "polygon", **kwargs):
        super().__init__(name, **kwargs)

    def construct_original_geometry(
        self,
        vertices=None,
        random_shape: bool = False,
        is_convex: bool = False,
        **kwargs,
    ):
        """
        Construct a polygon geometry.

        Args:
            vertices: [[x1, y1], [x2, y2]..]
            random_shape: whether to generate random shape, default is False
            is_convex: whether to generate convex shape, default is False
            **kwargs: see random_generate_polygon()

        Returns:
            Polygon object
        """

        if random_shape:
            if is_convex:
                kwargs.setdefault("spikeyness_range", [0, 0])
                vertices = random_generate_polygon(**kwargs)
            else:
                vertices = random_generate_polygon(**kwargs)

        elif vertices is None:
            log_warning("No vertices provided for polygon. Using default square.")
            vertices = [
                (-1, -1),
                (1, -1),
                (1, 1),
                (-1, 1),
            ]

        polygon = Polygon(vertices)

        if is_valid(polygon):
            return polygon
        log_warning("Invalid polygon. Making it valid.")
        valid_polygons = make_valid(polygon)

        polygon = envelope(valid_polygons)

        return make_valid(polygon)


class RectangleGeometry(geometry_handler):
    """Rectangle geometry handler for ``shape: {name: rectangle}``."""

    def __init__(self, name: str = "rectangle", **kwargs):
        super().__init__(name, **kwargs)

    def construct_original_geometry(
        self, length: float = 1.0, width: float = 1.0, wheelbase: float | None = None
    ):
        """
        Construct a rectangle around the local origin.

        Args:
            length: Rectangle length along x.
            width: Rectangle width along y.
            wheelbase: Optional Ackermann wheelbase used to offset the rear axle.

        Returns:
            shapely.geometry.Polygon: Rectangle polygon.
        """

        if wheelbase is None:
            vertices = [
                (-length / 2, -width / 2),
                (length / 2, -width / 2),
                (length / 2, width / 2),
                (-length / 2, width / 2),
            ]
        else:
            start_x = -(length - wheelbase) / 2
            start_y = -width / 2

            vertices = [
                (start_x, start_y),
                (start_x + length, start_y),
                (start_x + length, start_y + width),
                (start_x, start_y + width),
            ]

        return Polygon(vertices)


class CompoundGeometry(geometry_handler):
    """Compound geometry assembled from fixed solid parts.

    Each part is defined in the owning object's body frame. Its optional
    ``pose`` is ``[x, y, theta]`` relative to that frame and defaults to the
    identity pose. The exact collision geometry is the union of all transformed
    parts, so overlapping parts do not introduce internal collision boundaries.
    """

    _SUPPORTED_PARTS: ClassVar[set[str]] = {"circle", "polygon", "rectangle"}

    def __init__(self, name: str = "compound", **kwargs):
        super().__init__(name, **kwargs)

    def construct_original_geometry(self, parts: list[dict] | None = None, **kwargs):
        """Construct a compound geometry in the owning object's body frame.

        Args:
            parts: Non-empty list of shape dictionaries. Each dictionary uses
                the existing primitive shape parameters and may include a local
                ``pose`` of ``[x, y, theta]``.
            **kwargs: Reserved for future compound-level options.

        Returns:
            shapely.geometry.Polygon | shapely.geometry.MultiPolygon: Union of
            all locally transformed part geometries.

        Raises:
            ValueError: If parts are missing, unsupported, empty, or have an
                invalid pose.
            TypeError: If a part is not a dictionary.
        """
        self.part_handlers = []
        self.part_poses = []
        self._original_part_geometries = []

        for part_config, pose_array in self._validate_parts(parts):
            handler = GeometryFactory.create_geometry(**part_config)
            local_geometry = geometry_transform(
                handler._original_geometry, pose_array.reshape(3, 1)
            )

            self.part_handlers.append(handler)
            self.part_poses.append(pose_array)
            self._original_part_geometries.append(local_geometry)

        geometry = unary_union(self._original_part_geometries)
        if geometry.is_empty or geometry.geom_type not in {"Polygon", "MultiPolygon"}:
            raise ValueError(
                "compound parts must produce a non-empty Polygon or MultiPolygon"
            )

        self.part_geometries = list(self._original_part_geometries)
        return geometry

    @classmethod
    def _validate_parts(cls, parts: list[dict] | None) -> list[tuple[dict, np.ndarray]]:
        """Validate and normalize compound part configurations."""
        if not isinstance(parts, list) or not parts:
            raise ValueError("compound shape requires a non-empty 'parts' list")

        validated = []
        for index, part in enumerate(parts):
            if not isinstance(part, dict):
                raise TypeError(f"compound parts[{index}] must be a dictionary")

            part_config = part.copy()
            if "color" in part_config:
                raise ValueError(
                    "compound parts do not support individual colors; "
                    "set color on the owning object"
                )

            try:
                pose = np.asarray(part_config.pop("pose", [0, 0, 0]), dtype=float)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"compound parts[{index}].pose must contain three numbers"
                ) from exc
            if pose.shape != (3,) or not np.all(np.isfinite(pose)):
                raise ValueError(
                    f"compound parts[{index}].pose must be finite [x, y, theta]"
                )

            name = part_config.get("name")
            if not isinstance(name, str):
                raise ValueError(f"compound parts[{index}] requires a shape name")
            name = name.lower()
            if name not in cls._SUPPORTED_PARTS:
                supported = ", ".join(sorted(cls._SUPPORTED_PARTS))
                raise ValueError(
                    f"unsupported compound part '{name}' at parts[{index}]; "
                    f"supported parts are: {supported}"
                )

            part_config["name"] = name
            validated.append((part_config, pose))

        return validated

    def step(self, state):
        """Transform the compound and its individual parts to ``state``."""
        self.geometry = geometry_transform(self._original_geometry, state)
        self.part_geometries = [
            geometry_transform(geometry, state)
            for geometry in self._original_part_geometries
        ]
        return self.geometry

    @property
    def vertices(self) -> None:
        """A compound has no single exact vertex matrix."""
        return None

    @property
    def original_vertices(self) -> None:
        """A compound has no single exact original vertex matrix."""
        return None

    @property
    def part_vertices(self) -> list[np.ndarray]:
        """Current exterior vertices for each part as ``(2, N)`` arrays."""
        return [
            geometry.exterior.coords._coords.T[:, :-1]
            for geometry in self.part_geometries
        ]

    @property
    def original_part_vertices(self) -> list[np.ndarray]:
        """Body-frame exterior vertices for each part as ``(2, N)`` arrays."""
        return [
            geometry.exterior.coords._coords.T[:, :-1]
            for geometry in self._original_part_geometries
        ]


class LinestringGeometry(geometry_handler):
    """LineString geometry handler for wall or boundary segments."""

    def __init__(self, name: str = "linestring", **kwargs):
        super().__init__(name, **kwargs)

    def construct_original_geometry(
        self, vertices, random_shape: bool = False, is_convex: bool = True, **kwargs
    ):
        """
        Construct a LineString object.

        Args:
            vertices: [[x1, y1], [x2, y2]..]
            random_shape: whether to generate random shape, default is False
            is_convex: whether to generate convex shape, default is False
            **kwargs: see random_generate_polygon()

        Returns:
            LineString object
        """

        if random_shape:
            if is_convex:
                vertices = random_generate_polygon(spikeyness_range=[0, 0], **kwargs)
            else:
                vertices = random_generate_polygon(**kwargs)

        return LineString(vertices)


class PointsGeometry(geometry_handler):
    """Map-cell geometry handler that converts occupied points to line segments."""

    def __init__(self, name: str = "map", **kwargs):
        super().__init__(name, **kwargs)

    def construct_original_geometry(
        self,
        points: np.ndarray,
        reso: np.ndarray | float = 0.1,
        **kwargs,
    ):
        """
        Construct boundary lines around occupied map cells.

        Args:
            points: (2, N) array of points
            reso: resolution, either a (2, 1) array [x_reso, y_reso]
                or a scalar applied to both axes

        Returns:
            shapely.geometry.MultiLineString: Boundary of occupied map cells.
        """
        reso = np.atleast_2d(reso)
        half_x = float(reso[0, 0]) / 2
        half_y = float(reso[-1, -1]) / 2
        boxes = shp.box(
            points[0] - half_x,
            points[1] - half_y,
            points[0] + half_x,
            points[1] + half_y,
        )
        boundary = unary_union(boxes).boundary
        if hasattr(boundary, "geoms"):
            return boundary
        return MultiLineString([LineString(boundary.coords)])


########################################3D Geometry Handler #############################################################


class geometry_handler3d(ABC):
    """
    This class is used to handle the 3D geometry of the object. It reads the shape parameters from yaml file and constructs the geometry of the object.
    """

    def __init__(self, name: str, **kwargs):
        self.name = name
        self._original_geometry = self.construct_original_geometry(**kwargs)
        self.geometry = self._original_geometry
        self.wheelbase = kwargs.get("wheelbase")
        self.length, self.width, self.depth = self.cal_length_width(
            self._original_geometry
        )

    @abstractmethod
    def construct_original_geometry(self, **kwargs):
        """Construct the local-frame 3D geometry before any state transform."""
        pass

    def step(self, state):
        """
        Transform geometry to the new state.

        Args:
            state (np.ndarray 6*1): [x, y, z, roll, pitch, yaw].

        Returns:
            Transformed geometry.
        """

        def transform_with_state(x, y):
            trans, rot = get_transform(state)
            points = np.array([x, y])
            new_points = rot @ points + trans
            return (new_points[0, :], new_points[1, :])

        new_geometry = transform(transform_with_state, self._original_geometry)
        self.geometry = new_geometry

        return new_geometry


class GeometryFactory:
    """
    Factory class to create geometry handlers.
    """

    @staticmethod
    def create_geometry(name: str = "circle", **kwargs) -> geometry_handler:
        """Create a geometry handler from a YAML ``shape`` name.

        Args:
            name: Shape name. Supported values are ``circle``, ``polygon``,
                ``rectangle``, ``compound``, ``linestring``, and ``map``.
            **kwargs: Shape-specific parameters forwarded to the handler.

        Returns:
            geometry_handler: Concrete geometry handler.

        Raises:
            ValueError: If ``name`` is not supported.
        """
        name = name.lower()

        if name == "circle":
            return CircleGeometry(name, **kwargs)

        if name == "polygon":
            return PolygonGeometry(name, **kwargs)

        if name == "rectangle":
            return RectangleGeometry(name, **kwargs)

        if name == "compound":
            return CompoundGeometry(name, **kwargs)

        if name == "linestring":
            return LinestringGeometry(name, **kwargs)

        if name == "map":
            return PointsGeometry(name, **kwargs)

        # elif name == 'sphere3d':
        #     return Sphere3DGeometry(name, **kwargs)
        # elif name == 'cuboid3d':
        #     return Cuboid3DGeometry(name, **kwargs)

        raise ValueError(f"Invalid geometry name: {name}")
