irsim.env.env_plot
==================

.. py:module:: irsim.env.env_plot

.. autoapi-nested-parse::

   This file is the implementation of the environment plot to visualize the environment objects.

   Author: Ruihua Han



Classes
-------

.. autoapisummary::

   irsim.env.env_plot.EnvPlot


Functions
---------

.. autoapisummary::

   irsim.env.env_plot.linewidth_from_data_units


Module Contents
---------------

.. py:class:: EnvPlot(world: Any, objects: Optional[list[Any]] = None, saved_figure: Optional[dict[str, Any]] = None, figure_pixels: Optional[collections.abc.Sequence[int]] = None, show_title: bool = True, **kwargs: Any)

   EnvPlot class for visualizing the environment.

   :param world: The world object containing environment information including grid_map, x_range, y_range.
   :param objects: List of objects in the environment. Default is [].
   :type objects: list, optional
   :param saved_figure: Keyword arguments for saving the figure.
                        See https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for details.
                        Default is dict().
   :type saved_figure: dict, optional
   :param figure_pixels: Width and height of the figure in pixels. Default is [1180, 1080].
   :type figure_pixels: list, optional
   :param show_title: Whether to show the title. Default is True.
   :type show_title: bool, optional
   :param kwargs: Additional options such as color_map, no_axis, and tight.

   Initialize the EnvPlot instance.

   Sets up the matplotlib figure, configures plotting parameters,
   and initializes the plot with world data and objects.


   .. py:attribute:: world


   .. py:attribute:: x_range


   .. py:attribute:: y_range


   .. py:attribute:: show_title
      :value: True



   .. py:attribute:: title
      :value: None



   .. py:attribute:: saved_figure_kwargs
      :type:  dict[str, Any]


   .. py:attribute:: color_map
      :type:  dict[str, str]


   .. py:attribute:: saved_ani_kwargs
      :type:  dict[str, Any]


   .. py:attribute:: dyna_line_list
      :type:  list[Any]
      :value: []



   .. py:attribute:: dyna_point_list
      :type:  list[Any]
      :value: []



   .. py:attribute:: dyna_quiver_list
      :type:  list[Any]
      :value: []



   .. py:method:: init_plot(grid_map: Optional[Any], objects: list[Any], no_axis: bool = False, tight: bool = True, **kwargs: Any) -> None

      Initialize the plot with the given grid map and objects.

      :param grid_map: The grid map of the environment.
      :type grid_map: optional
      :param objects: List of objects to plot.
      :type objects: list
      :param no_axis: Whether to show the axis. Default is False.
      :type no_axis: bool, optional
      :param tight: Whether to show the axis tightly. Default is True.
      :type tight: bool, optional



   .. py:method:: step(mode: str = 'dynamic', objects: Optional[list[Any]] = None, **kwargs: Any) -> None


   .. py:method:: init_objects_plot(objects: list[Any], **kwargs: Any) -> None


   .. py:method:: step_objects_plot(mode: str = 'dynamic', objects: Optional[list[Any]] = None, **kwargs: Any) -> None

      Update the plot for the objects.



   .. py:method:: draw_components(mode: str = 'all', objects: Optional[list[Any]] = None, **kwargs: Any) -> None

      Draw the components in the environment.

      :param mode: 'static', 'dynamic', or 'all' to specify which objects to draw.
      :type mode: str
      :param objects: List of objects to draw.
      :type objects: list
      :param kwargs: Additional plotting options.



   .. py:method:: clear_components(mode: str = 'all', objects: Optional[list[Any]] = None) -> None

      Clear the components in the environment.

      :param mode: 'static', 'dynamic', or 'all' to specify which objects to clear.
      :type mode: str
      :param objects: List of objects to clear.
      :type objects: list



   .. py:method:: draw_grid_map(grid_map: Optional[Any] = None, **kwargs: Any) -> None

      Draw the grid map on the plot.

      :param grid_map: The grid map to draw.
      :type grid_map: optional



   .. py:method:: draw_trajectory(traj: list[Any] | numpy.ndarray, traj_type: str = 'g-', label: str = 'trajectory', show_direction: bool = False, refresh: bool = False, **kwargs: Any) -> None

      Draw a trajectory on the plot.

      :param traj: List of points or array of points [x, y, theta].
      :type traj: list or np.ndarray
      :param traj_type: Type of trajectory line (e.g., 'g-').
                        See https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.plot.html for details.
      :type traj_type: str
      :param label: Label for the trajectory.
      :type label: str
      :param show_direction: Whether to show the direction of the trajectory.
      :type show_direction: bool
      :param refresh: Whether to refresh the plot.
      :type refresh: bool
      :param kwargs: Additional plotting options for ax.plot()



   .. py:method:: draw_points(points: Optional[list[Any] | numpy.ndarray], s: int = 10, c: str = 'm', refresh: bool = True, **kwargs: Any) -> None

      Draw points on the plot.

      :param points: List of points, each point as [x, y] or (2, 1) array
                     or (np.array): points array: (2, N), NL number of points
      :type points: list
      :param s: Size of the points.
      :type s: int
      :param c: Color of the points.
      :type c: str
      :param refresh: Whether to refresh the plot.
      :type refresh: bool
      :param kwargs: Additional plotting options.



   .. py:method:: draw_quiver(point: Optional[numpy.ndarray], refresh: bool = False, color: str = 'black', **kwargs: Any) -> None

      Draw a quiver plot on the plot.

      :param points: List of points, each point as [x, y, u, v]. u, v are the components of the vector.
      :type points: 4*1 np.ndarray
      :param kwargs: Additional plotting options.



   .. py:method:: draw_quivers(points: collections.abc.Iterable[numpy.ndarray], refresh: bool = False, color: str = 'black', **kwargs: Any) -> None

      Draw a series of quiver plot on the plot.

      :param points: List of points, each point as [x, y, u, v]. u, v are the components of the vector.
      :type points: list or np.ndarray



   .. py:method:: draw_box(vertices: numpy.ndarray, refresh: bool = False, color: str = 'b-') -> None

      Draw a box by the vertices.

      :param vertices: 2xN array of vertices.
      :type vertices: np.ndarray
      :param refresh: Whether to refresh the plot.
      :type refresh: bool
      :param color: Color and line type of the box.
      :type color: str



   .. py:method:: update_title() -> None


   .. py:method:: save_figure(file_name: str = '', file_format: str = 'png', include_index: bool = False, save_gif: bool = False, **kwargs: Any) -> None

      Save the current figure.

      :param file_name: Name of the figure. Default is ''.
      :type file_name: str
      :param file_format: Format of the figure. Default is 'png'.
      :type file_format: str
      :param kwargs: Additional arguments for saving the figure.
                     See https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.savefig.html for details.



   .. py:method:: save_animate(ani_name: str = 'animation', suffix: str = '.gif', last_frame_duration: int = 1, rm_fig_path: bool = True, **kwargs: Any) -> None

      Save the animation.

      :param ani_name: Name of the animation. Default is 'animation'.
      :type ani_name: str
      :param last_frame_duration: Duration of the last frame for the gif. Default is 1 second.
      :type last_frame_duration: int
      :param suffix: Suffix of the animation file. Default is '.gif'.
      :type suffix: str
      :param rm_fig_path: Whether to remove the figure path after saving. Default is True.
      :type rm_fig_path: bool
      :param kwargs: Additional arguments for saving the animation.
                     See `imageio.imwrite <https://imageio.readthedocs.io/en/stable/_autosummary/imageio.v3.imwrite.html#imageio.v3.imwrite>`_ for details.



   .. py:method:: show() -> None

      Display the plot.



   .. py:method:: close() -> None

      Close the plot.



   .. py:property:: logger


.. py:function:: linewidth_from_data_units(linewidth: float, axis: Any, reference: str = 'y') -> float

   Convert a linewidth in data units to linewidth in points.

   :param linewidth: Linewidth in data units of the respective reference-axis
   :type linewidth: float
   :param axis: The axis which is used to extract the relevant transformation
                data (data limits and size must not change afterwards)
   :type axis: matplotlib axis
   :param reference: The axis that is taken as a reference for the data width.
                     Possible values: 'x' and 'y'. Defaults to 'y'.
   :type reference: string

   :returns: **linewidth** -- Linewidth in points
   :rtype: float


