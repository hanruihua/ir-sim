import numpy as np
from abc import ABC, abstractmethod
from irsim.util.util import get_transform
from shapely.ops import transform
from shapely import Point, Polygon, LineString, minimum_bounding_radius, MultiPoint, bounds
from irsim.lib import random_generate_polygon

class geometry_handler(ABC):

    '''
    This class is used to handle the geometry of the object. It reads the shape parameters from yaml file and constructs the geometry of the object.
    '''

    def __init__(self, name: str, **kwargs):

        self.name = name
        self._init_geometry = self.construct_init_geometry(**kwargs)
        self.geometry = self._init_geometry
        self.wheelbase = kwargs.get('wheelbase', None)
        self.length, self.width = self.cal_length_width(self._init_geometry)

    @abstractmethod
    def construct_init_geometry(self, **kwargs) :
        pass
    

    def step(self, state):

        """
        Transform geometry to the new state.

        Args:
            state (np.ndarray): State vector [x, y, theta].

        Returns:
            Transformed geometry.
        """

        def transform_with_state(x, y):
            trans, rot = get_transform(state)
            points = np.array([x, y])
            new_points = rot @ points + trans
            return (new_points[0, :], new_points[1, :])

        new_geometry = transform(transform_with_state, self._init_geometry)
        self.geometry = new_geometry
        
        return new_geometry

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
        return self.geometry.exterior.coords._coords.T
    
    @property
    def radius(self):
        return minimum_bounding_radius(self._init_geometry)

class CircleGeometry(geometry_handler):

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

    def construct_init_geometry(self, radius: float=0.2, random_shape: bool=False, radius_range: list=[0.1, 1.0], wheelbase: float=None):
        
        if random_shape:
            radius = np.random.uniform(*radius_range)
        
        if wheelbase is None:
            return Point([0, 0]).buffer(radius)
        else:
            return Point([wheelbase/2, 0]).buffer(radius)

class PolygonGeometry(geometry_handler):

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

    def construct_init_geometry(self, vertices=None, random_shape: bool=False, is_convex: bool=True, **kwargs):

        '''
        vertices: [[x1, y1], [x2, y2]..]
        **kwargs: see random_generate_polygon()
        '''

        if random_shape:
            if is_convex:
                vertices = random_generate_polygon(spikeyness_range=[0, 0], **kwargs)
            else:
                vertices = random_generate_polygon(**kwargs)

        elif vertices is None:
            print('No vertices provided for polygon. Using default square')
            vertices = [
                (-1, -1),
                (1, -1),
                (1, 1),
                (-1, 1),
            ]

        return Polygon(vertices) 

class RectangleGeometry(geometry_handler):

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

    def construct_init_geometry(self, length: float=1.0, width: float = 1.0, wheelbase: float=None):

        '''
        Args
            length: in x axis
            width: in y axis
            wheelbase: for ackermann robot
        '''

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
                (start_x, start_y + width),]

        return Polygon(vertices) 


class LinestringGeometry(geometry_handler):

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

    def construct_init_geometry(self, vertices, random_shape: bool=False, is_convex: bool=True, **kwargs):

        '''
        vertices: [[x1, y1], [x2, y2]..]
        **kwargs: see random_generate_polygon()
        '''

        if random_shape:
            if is_convex:
                vertices = random_generate_polygon(spikeyness_range=[0, 0], **kwargs)
            else:
                vertices = random_generate_polygon(**kwargs)

        return LineString(vertices) 
    
class PointsGeometry(geometry_handler):
    
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

    def construct_init_geometry(self, points: np.ndarray, reso: float=0.1):

        '''
        Args:
            points: (2, N) array of points
            reso: resolution for the buffer
        '''

        return MultiPoint(points.T).buffer(reso / 2).boundary


class GeometryFactory:
    """
    Factory class to create geometry handlers.
    """

    @staticmethod
    def create_geometry(
        name: str,
        **kwargs
    ) -> geometry_handler:
        name = name.lower()

        if name == "circle":
            return CircleGeometry(name, **kwargs)

        elif name == "polygon":
            return PolygonGeometry(name, **kwargs)
        
        elif name == 'rectangle':
            return RectangleGeometry(name, **kwargs)
        
        elif name == 'linestring':
            return LinestringGeometry(name, **kwargs)

        elif name == 'points':
            return PointsGeometry(name, **kwargs)
        else:
            raise ValueError(f"Invalid geometry name: {name}")












