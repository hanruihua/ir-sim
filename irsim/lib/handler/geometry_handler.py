import numpy as np
from abc import ABC, abstractmethod

class geometry_handler(ABC):

    '''
    This class is used to handle the geometry of the object. It reads the shape parameters from yaml file and constructs the geometry of the object.
    '''


    def __init__(self, name, **kwargs):
        self._init_geometry = self.construct_init_geometry()


    @abstractmethod
    def construct_init_geometry(self) :
        pass
    
    @abstractmethod
    def step():
        pass


    # @abstractmethod
    # def construct_init_geometry(self, shape):
    #     """
    #     Construct the geometry of the object.

    #     Args:
    #         shape (str): The shape of the object.
    #         shape_tuple: Tuple to initialize the geometry.
    #         reso (float): The resolution of the object.

    #     Returns:
    #         Geometry of the object.
    #     """
    #     if shape == "circle":
    #         geometry = Point([shape_tuple[0], shape_tuple[1]]).buffer(shape_tuple[2])

    #     elif shape == "polygon" or shape == "rectangle":
    #         geometry = Polygon(shape_tuple)

    #     elif shape == "linestring":
    #         geometry = LineString(shape_tuple)

    #     elif shape == "points":
    #         geometry = MultiPoint(shape_tuple.T).buffer(reso / 2).boundary

    #     else:
    #         raise ValueError(
    #             "shape should be one of the following: circle, polygon, linestring, points"
    #         )

        # if shape == "polygon" or shape == "rectangle" or shape == "circle":
        #     self.G, self.h, self.cone_type = self.generate_Gh(shape, shape_tuple)
        # else:
        #     self.G, self.h, self.cone_type = None, None, "Rpositive"

        # return geometry



class CircleGeometry(geometry_handler):
    pass


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
            return CircleGeometry(**kwargs)












