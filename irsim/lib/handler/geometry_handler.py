import numpy as np
from abc import ABC, abstractmethod
from irsim.util.util import (
    get_transform,
    gen_inequal_from_vertex,
    is_convex_and_ordered,
    geometry_transform,
)
from shapely.ops import transform
from shapely import (
    Point,
    Polygon,
    LineString,
    minimum_bounding_radius,
    MultiPoint,
    bounds,
    is_valid,
    make_valid,
    envelope,
)
from irsim.lib import random_generate_polygon
from typing import Optional

class geometry_handler(ABC):
    """
    This class is used to handle the geometry of the object. It reads the shape parameters from yaml file and constructs the geometry of the object.
    """

    def __init__(self, name: str, **kwargs):

        self.name = name
        self._original_geometry = self.construct_original_geometry(**kwargs)
        self.geometry = self._original_geometry
        self.wheelbase = kwargs.get("wheelbase", None)
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
            Transformed geometry.
        """
        
        self.geometry = geometry_transform(self._original_geometry, state)

        return self.geometry

    def get_init_Gh(self):
        """
        Generate initial G and h for convex object.

        Returns:
                G matrix: (N, 2)
                h vector: (N, 1)
                cone_type (str): "norm2" for circle or "Rpositive" for polygon
                convex_flag (bool):  for convex or not
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
            G, h, cone_type, convex_flag = self.get_polygon_Gh(kwargs.get("vertices", None))

        elif self.name == "circle":
            G, h, cone_type, convex_flag = self.get_circle_Gh(
                kwargs.get("center", None), kwargs.get("radius", None)
            )

        return G, h, cone_type, convex_flag

    def get_polygon_Gh(self, vertices: Optional[np.ndarray] = None):

        """
        Generate G and h for convex polygon.

        Args:
            vertices: (2, N), N: Edge number of the object

        Returns:
                G matrix: (N, 2)
                h vector: (N, 1)
                cone_type (str): "norm2" for circle or "Rpositive" for polygon
                convex_flag (bool):  for convex or not
        """

        if self.name == "polygon" or self.name == "rectangle":

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
            G matrix: (3, 2)
            h vector: (3, 1)
            cone_type (str): "norm2"
            convex_flag (bool): True
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

    @property
    def vertices(self):
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
    def original_vertices(self):
        """
        Get the original vertices of the object.
        """

        if self.name == "linestring":
            x = self._original_geometry.xy[0]
            y = self._original_geometry.xy[1]
            return np.c_[x, y].T

        elif self.name == "map":
            return None
        else:
            return self._original_geometry.exterior.coords._coords.T[:, :-1]

    @property
    def radius(self):
        return minimum_bounding_radius(self._original_geometry)


class CircleGeometry(geometry_handler):

    def __init__(self, name: str = "circle", **kwargs):
        super().__init__(name, **kwargs)

    def construct_original_geometry(
        self,
        radius: float = 0.2,
        random_shape: bool = False,
        radius_range: list = [0.1, 1.0],
        wheelbase: Optional[float] = None,
    ):

        if random_shape:
            radius = np.random.uniform(*radius_range)

        if wheelbase is None:
            return Point([0, 0]).buffer(radius)
        else:
            return Point([wheelbase / 2, 0]).buffer(radius)


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
        else:
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
        self.wheelbase = kwargs.get("wheelbase", None)
        self.length, self.width, self.depth = self.cal_length_width(self._original_geometry)

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

        elif name == "polygon":
            return PolygonGeometry(name, **kwargs)

        elif name == "rectangle":
            return RectangleGeometry(name, **kwargs)

        elif name == "linestring":
            return LinestringGeometry(name, **kwargs)

        elif name == "map":
            return PointsGeometry(name, **kwargs)

        # elif name == 'sphere3d':
        #     return Sphere3DGeometry(name, **kwargs)
        # elif name == 'cuboid3d':
        #     return Cuboid3DGeometry(name, **kwargs)

        else:
            raise ValueError(f"Invalid geometry name: {name}")
