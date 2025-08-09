irsim.lib.algorithm.generation
==============================

.. py:module:: irsim.lib.algorithm.generation

.. autoapi-nested-parse::

   This file is the implementation of the generation of random polygons.

   Author: Ruihua Han



Functions
---------

.. autoapisummary::

   irsim.lib.algorithm.generation.clip
   irsim.lib.algorithm.generation.random_generate_polygon
   irsim.lib.algorithm.generation.generate_polygon
   irsim.lib.algorithm.generation.random_angle_steps


Module Contents
---------------

.. py:function:: clip(value: float, lower: float, upper: float) -> float

   Clip a value to a specified range.

   :param value: Value to be clipped.
   :type value: float
   :param lower: Lower bound of the range.
   :type lower: float
   :param upper: Upper bound of the range.
   :type upper: float

   :returns: Clipped value.
   :rtype: float


.. py:function:: random_generate_polygon(number: int = 1, center_range: Optional[list[float]] = None, avg_radius_range: Optional[list[float]] = None, irregularity_range: Optional[list[float]] = None, spikeyness_range: Optional[list[float]] = None, num_vertices_range: Optional[list[int]] = None, **kwargs: Any) -> Union[numpy.ndarray, list[numpy.ndarray]]

   reference: https://stackoverflow.com/questions/8997099/algorithm-to-generate-random-2d-polygon

   Generate random polygons with specified properties.

   :param number: Number of polygons to generate (default 1).
   :type number: int
   :param center_range: Range for the polygon center [min_x, min_y, max_x, max_y].
   :type center_range: List[float]
   :param avg_radius_range: Range for the average radius of the polygons.
   :type avg_radius_range: List[float]
   :param irregularity_range: Range for the irregularity of the polygons.
   :type irregularity_range: List[float]
   :param spikeyness_range: Range for the spikeyness of the polygons.
   :type spikeyness_range: List[float]
   :param num_vertices_range: Range for the number of vertices of the polygons.
   :type num_vertices_range: List[int]

   :returns: List of vertices for each polygon or a single polygon's vertices if number=1.


.. py:function:: generate_polygon(center: list[float], avg_radius: float, irregularity: float, spikeyness: float, num_vertices: int) -> numpy.ndarray

   Generate a random polygon around a center point.

   :param center: Center of the polygon.
   :type center: Tuple[float, float]
   :param avg_radius: Average radius from the center to vertices.
   :type avg_radius: float
   :param irregularity: Variance of angle spacing between vertices. Range [0, 1]
   :type irregularity: float
   :param spikeyness: Variance of radius from the center. Range [0, 1]
   :type spikeyness: float
   :param num_vertices: Number of vertices for the polygon.
   :type num_vertices: int

   :returns: Vertices of the polygon in CCW order.
   :rtype: numpy.ndarray


.. py:function:: random_angle_steps(steps: int, irregularity: float) -> list[float]

   Generate random angle steps for polygon vertices.

   :param steps: Number of angles to generate.
   :type steps: int
   :param irregularity: Variance of angle spacing.
   :type irregularity: float

   :returns: Random angles in radians.
   :rtype: List[float]


