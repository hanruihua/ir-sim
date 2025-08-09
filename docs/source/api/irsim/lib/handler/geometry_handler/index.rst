irsim.lib.handler.geometry_handler
==================================

.. py:module:: irsim.lib.handler.geometry_handler


Classes
-------

.. autoapisummary::

   irsim.lib.handler.geometry_handler.geometry_handler
   irsim.lib.handler.geometry_handler.CircleGeometry
   irsim.lib.handler.geometry_handler.PolygonGeometry
   irsim.lib.handler.geometry_handler.RectangleGeometry
   irsim.lib.handler.geometry_handler.LinestringGeometry
   irsim.lib.handler.geometry_handler.PointsGeometry
   irsim.lib.handler.geometry_handler.geometry_handler3d
   irsim.lib.handler.geometry_handler.GeometryFactory


Module Contents
---------------

.. py:class:: geometry_handler(name: str, **kwargs)

   Bases: :py:obj:`abc.ABC`


   This class is used to handle the geometry of the object. It reads the shape parameters from yaml file and constructs the geometry of the object.


   .. py:attribute:: name


   .. py:attribute:: geometry


   .. py:attribute:: wheelbase


   .. py:method:: construct_original_geometry(**kwargs)
      :abstractmethod:


      Construct the original geometry of the object when the state is in the origin of coordinates.

      :param \*\*kwargs: shape parameters

      Returns: Geometry of the object



   .. py:method:: step(state)

      Transform geometry to the new state.

      :param state: State vector [x, y, theta].
      :type state: np.ndarray

      :returns: Transformed geometry.



   .. py:method:: get_init_Gh()

      Generate initial G and h for convex object.

      :returns: (N, 2)
                h vector: (N, 1)
                cone_type (str): "norm2" for circle or "Rpositive" for polygon
                convex_flag (bool):  for convex or not
      :rtype: G matrix



   .. py:method:: get_Gh(**kwargs)


   .. py:method:: get_polygon_Gh(vertices: Optional[numpy.ndarray] = None)

      Generate G and h for convex polygon.

      :param vertices: (2, N), N: Edge number of the object

      :returns: (N, 2)
                h vector: (N, 1)
                cone_type (str): "norm2" for circle or "Rpositive" for polygon
                convex_flag (bool):  for convex or not
      :rtype: G matrix



   .. py:method:: get_circle_Gh(center: numpy.ndarray, radius: float)

      Generate G and h for circle.

      :param center: (2, 1) array of center
      :param radius: float of radius

      :returns: (3, 2)
                h vector: (3, 1)
                cone_type (str): "norm2"
                convex_flag (bool): True
      :rtype: G matrix



   .. py:method:: cal_length_width(geometry)


   .. py:property:: vertices


   .. py:property:: init_vertices

      [[x1, y1], [x2, y2]....    [[x1, y1]]]; [x1, y1] will repeat twice

      :type: return original_vertices


   .. py:property:: original_vertices
      :type: Optional[numpy.ndarray]


      Get the original vertices of the geometry.


   .. py:property:: original_centroid
      :type: numpy.ndarray


      Get the original centroid of the geometry.

      :returns: The original centroid of the geometry.
      :rtype: np.ndarray


   .. py:property:: radius


.. py:class:: CircleGeometry(name: str = 'circle', **kwargs)

   Bases: :py:obj:`geometry_handler`


   This class is used to handle the geometry of the object. It reads the shape parameters from yaml file and constructs the geometry of the object.


   .. py:method:: construct_original_geometry(radius: float = 0.2, center: Optional[list] = None, random_shape: bool = False, radius_range: Optional[list] = None, wheelbase: Optional[float] = None)

      Construct the original geometry of the object when the state is in the origin of coordinates.

      :param \*\*kwargs: shape parameters

      Returns: Geometry of the object



.. py:class:: PolygonGeometry(name: str = 'polygon', **kwargs)

   Bases: :py:obj:`geometry_handler`


   This class is used to handle the geometry of the object. It reads the shape parameters from yaml file and constructs the geometry of the object.


   .. py:method:: construct_original_geometry(vertices=None, random_shape: bool = False, is_convex: bool = False, **kwargs)

      Construct a polygon geometry.

      :param vertices: [[x1, y1], [x2, y2]..]
      :param random_shape: whether to generate random shape, default is False
      :param is_convex: whether to generate convex shape, default is False
      :param \*\*kwargs: see random_generate_polygon()

      :returns: Polygon object



.. py:class:: RectangleGeometry(name: str = 'rectangle', **kwargs)

   Bases: :py:obj:`geometry_handler`


   This class is used to handle the geometry of the object. It reads the shape parameters from yaml file and constructs the geometry of the object.


   .. py:method:: construct_original_geometry(length: float = 1.0, width: float = 1.0, wheelbase: Optional[float] = None)

      Args
          length: in x axis
          width: in y axis
          wheelbase: for ackermann robot



.. py:class:: LinestringGeometry(name: str = 'linestring', **kwargs)

   Bases: :py:obj:`geometry_handler`


   This class is used to handle the geometry of the object. It reads the shape parameters from yaml file and constructs the geometry of the object.


   .. py:method:: construct_original_geometry(vertices, random_shape: bool = False, is_convex: bool = True, **kwargs)

      Construct a LineString object.

      :param vertices: [[x1, y1], [x2, y2]..]
      :param random_shape: whether to generate random shape, default is False
      :param is_convex: whether to generate convex shape, default is False
      :param \*\*kwargs: see random_generate_polygon()

      :returns: LineString object



.. py:class:: PointsGeometry(name: str = 'map', **kwargs)

   Bases: :py:obj:`geometry_handler`


   This class is used to handle the geometry of the object. It reads the shape parameters from yaml file and constructs the geometry of the object.


   .. py:method:: construct_original_geometry(points: numpy.ndarray, reso: float = 0.1)

      :param points: (2, N) array of points
      :param reso: resolution for the buffer



.. py:class:: geometry_handler3d(name: str, **kwargs)

   Bases: :py:obj:`abc.ABC`


   This class is used to handle the 3D geometry of the object. It reads the shape parameters from yaml file and constructs the geometry of the object.


   .. py:attribute:: name


   .. py:attribute:: geometry


   .. py:attribute:: wheelbase


   .. py:method:: construct_original_geometry(**kwargs)
      :abstractmethod:



   .. py:method:: step(state)

      Transform geometry to the new state.

      :param state: [x, y, z, roll, pitch, roll].
      :type state: np.ndarray 6*1

      :returns: Transformed geometry.



.. py:class:: GeometryFactory

   Factory class to create geometry handlers.


   .. py:method:: create_geometry(name: str = 'circle', **kwargs) -> geometry_handler
      :staticmethod:



