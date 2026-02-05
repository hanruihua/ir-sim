from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
from shapely import (
    LineString,
    MultiPoint,
    MultiPolygon,
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
            G = np.array([[1, 0], [0, 1], [0, 0]])
            h = np.array([[0], [0], [-self.radius]])
            cone_type = "norm2"
            convex_flag = True

        elif self.name == "polygon" or self.name == "rectangle":
            convex_flag, _ = is_convex_and_ordered(self.original_vertices)

            if convex_flag:
                G, h = gen_inequal_from_vertex(self.original_vertices)
                cone_type = "Rpositive"
            else:
                G, h, cone_type = None, None, None
        else:
            G, h, cone_type, convex_flag = None, None, None, None

        return G, h, cone_type, convex_flag

    def get_Gh(self, **kwargs):
        if self.name == "polygon" or self.name == "rectangle":
            G, h, cone_type, convex_flag = self.get_polygon_Gh(kwargs.get("vertices"))

        elif self.name == "circle":
            G, h, cone_type, convex_flag = self.get_circle_Gh(
                kwargs.get("center"), kwargs.get("radius")
            )

        elif self.name == "multipolygon":
            # MultiPolygon is treated as non-convex by default
            G, h, cone_type, convex_flag = None, None, None, False

        else:
            G, h, cone_type, convex_flag = None, None, None, None

        return G, h, cone_type, convex_flag

    def get_polygon_Gh(self, vertices: Optional[np.ndarray] = None):
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
        min_x, min_y, max_x, max_y = bounds(geometry).tolist()
        length = max_x - min_x
        width = max_y - min_y

        return length, width

    def _get_multipolygon_vertices(self, geometry) -> np.ndarray:
        """
        Extract combined vertices from a MultiPolygon geometry.

        Args:
            geometry: A Shapely MultiPolygon geometry object.

        Returns:
            np.ndarray: Combined vertices array of shape (2, N) where N is total vertex count.
        """
        all_vertices = []
        for geom in geometry.geoms:
            coords = geom.exterior.coords._coords[:-1]  # Exclude repeated last point
            all_vertices.append(coords)
        combined = np.vstack(all_vertices)
        return combined.T

    @property
    def vertices(self):
        if self.name == "linestring":
            x = self.geometry.xy[0]
            y = self.geometry.xy[1]
            return np.c_[x, y].T
        if self.name == "multipolygon":
            return self._get_multipolygon_vertices(self.geometry)
        return self.geometry.exterior.coords._coords.T[:, :-1]

    @property
    def init_vertices(self):
        """
        return original_vertices: [[x1, y1], [x2, y2]....    [[x1, y1]]]; [x1, y1] will repeat twice
        """
        assert "this property is renamed to be original_vertices"

    @property
    def original_vertices(self) -> Optional[np.ndarray]:
        """
        Get the original vertices of the geometry.
        """

        if self.name == "linestring":
            x = self._original_geometry.xy[0]
            y = self._original_geometry.xy[1]
            return np.c_[x, y].T

        if self.name == "map":
            return None

        if self.name == "multipolygon":
            return self._get_multipolygon_vertices(self._original_geometry)

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
        return minimum_bounding_radius(self._original_geometry)


class CircleGeometry(geometry_handler):
    def __init__(self, name: str = "circle", **kwargs):
        super().__init__(name, **kwargs)

    def construct_original_geometry(
        self,
        radius: float = 0.2,
        center: Optional[list] = None,
        random_shape: bool = False,
        radius_range: Optional[list] = None,
        wheelbase: Optional[float] = None,
    ):
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
            print("No vertices provided for polygon. Using default square")
            vertices = [
                (-1, -1),
                (1, -1),
                (1, 1),
                (-1, 1),
            ]

        polygon = Polygon(vertices)

        if is_valid(polygon):
            return polygon
        print("Invalid polygon. Making it valid.")
        valid_polygons = make_valid(polygon)

        polygon = envelope(valid_polygons)

        return make_valid(polygon)


class RectangleGeometry(geometry_handler):
    def __init__(self, name: str = "rectangle", **kwargs):
        super().__init__(name, **kwargs)

    def construct_original_geometry(
        self, length: float = 1.0, width: float = 1.0, wheelbase: Optional[float] = None
    ):
        """
        Args
            length: in x axis
            width: in y axis
            wheelbase: for ackermann robot
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


class MultiPolygonGeometry(geometry_handler):
    """
    Geometry handler for complex shapes comprised of multiple rectangles or polygons.

    This class supports robots with complex shapes that can be defined as a collection
    of multiple rectangles (or arbitrary polygons), each with their own offset position.
    Overlapping components are automatically merged using unary_union for valid geometry.
    """

    def __init__(self, name: str = "multipolygon", **kwargs):
        super().__init__(name, **kwargs)

    def construct_original_geometry(
        self,
        components: Optional[list] = None,
        **kwargs,
    ):
        """
        Construct a MultiPolygon or Polygon geometry from multiple rectangular/polygon components.

        Args:
            components: List of component dictionaries, each defining a rectangle or polygon.
                For rectangles: {"type": "rectangle", "length": float, "width": float,
                                 "offset": [x, y], "rotation": float (optional, in radians)}
                For polygons: {"type": "polygon", "vertices": [[x1,y1], [x2,y2], ...]}

                If "type" is not specified, "rectangle" is assumed for backward compatibility
                with simple rectangle definitions like:
                [{"length": 1.0, "width": 0.5, "offset": [0, 0]}, ...]

        Returns:
            Shapely Polygon or MultiPolygon (merged if components overlap)

        Example:
            # L-shaped robot from two rectangles
            components = [
                {"length": 2.0, "width": 0.5, "offset": [0, 0]},       # horizontal bar
                {"length": 0.5, "width": 1.5, "offset": [-0.75, 0.5]}, # vertical bar
            ]
        """
        if components is None:
            print("No components provided for multipolygon. Using default single rectangle")
            components = [{"length": 1.0, "width": 1.0, "offset": [0, 0]}]

        polygons = []
        for comp in components:
            comp_type = comp.get("type", "rectangle")

            if comp_type == "rectangle":
                length = comp.get("length", 1.0)
                width = comp.get("width", 1.0)
                offset = comp.get("offset", [0, 0])
                rotation = comp.get("rotation", 0.0)

                # Create rectangle vertices centered at offset
                half_l = length / 2
                half_w = width / 2
                vertices = [
                    (-half_l, -half_w),
                    (half_l, -half_w),
                    (half_l, half_w),
                    (-half_l, half_w),
                ]

                # Apply rotation if specified
                if rotation != 0.0:
                    cos_r = np.cos(rotation)
                    sin_r = np.sin(rotation)
                    vertices = [
                        (x * cos_r - y * sin_r, x * sin_r + y * cos_r)
                        for x, y in vertices
                    ]

                # Apply offset
                vertices = [(x + offset[0], y + offset[1]) for x, y in vertices]
                polygons.append(Polygon(vertices))

            elif comp_type == "polygon":
                poly_vertices = comp.get("vertices")
                if poly_vertices is None:
                    raise ValueError("Polygon component requires 'vertices' parameter")
                offset = comp.get("offset", [0, 0])
                # Apply offset to vertices
                shifted_vertices = [
                    (v[0] + offset[0], v[1] + offset[1]) for v in poly_vertices
                ]
                polygons.append(Polygon(shifted_vertices))

            else:
                raise ValueError(f"Unknown component type: {comp_type}")

        # Use unary_union to merge overlapping polygons into valid geometry
        # This handles overlapping components as recommended in the issue
        merged = unary_union(polygons)

        # If the result is a single Polygon (after merging), convert to MultiPolygon
        # to maintain consistent handling
        if isinstance(merged, Polygon):
            return MultiPolygon([merged])

        return merged


class LinestringGeometry(geometry_handler):
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
    def __init__(self, name: str = "map", **kwargs):
        super().__init__(name, **kwargs)

    def construct_original_geometry(self, points: np.ndarray, reso: float = 0.1):
        """
        Args:
            points: (2, N) array of points
            reso: resolution for the buffer
        """

        return MultiPoint(points.T).buffer(reso / 2).boundary


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
        pass

    def step(self, state):
        """
        Transform geometry to the new state.

        Args:
            state (np.ndarray 6*1): [x, y, z, roll, pitch, roll].

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
        name = name.lower()

        if name == "circle":
            return CircleGeometry(name, **kwargs)

        if name == "polygon":
            return PolygonGeometry(name, **kwargs)

        if name == "rectangle":
            return RectangleGeometry(name, **kwargs)

        if name == "multipolygon":
            return MultiPolygonGeometry(name, **kwargs)

        if name == "linestring":
            return LinestringGeometry(name, **kwargs)

        if name == "map":
            return PointsGeometry(name, **kwargs)

        # elif name == 'sphere3d':
        #     return Sphere3DGeometry(name, **kwargs)
        # elif name == 'cuboid3d':
        #     return Cuboid3DGeometry(name, **kwargs)

        raise ValueError(f"Invalid geometry name: {name}")
