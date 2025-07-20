irsim.env.env_plot3d
====================

.. py:module:: irsim.env.env_plot3d


Classes
-------

.. autoapisummary::

   irsim.env.env_plot3d.EnvPlot3D


Module Contents
---------------

.. py:class:: EnvPlot3D(world, objects=[], saved_figure=dict(), figure_pixels: list = [1180, 1080], show_title: bool = True, **kwargs)

   Bases: :py:obj:`irsim.env.env_plot.EnvPlot`


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


   .. py:attribute:: ax


   .. py:attribute:: z_range


   .. py:method:: draw_points(points, s=10, c='m', refresh=True, **kwargs)

      Draw points on the plot.

      :param points: List of points, each point as [x, y, z].
      :type points: list
      :param s: Size of the points.
      :type s: int
      :param c: Color of the points.
      :type c: str
      :param refresh: Whether to refresh the plot.
      :type refresh: bool
      :param kwargs: Additional plotting options.



   .. py:method:: draw_quiver(point, refresh=False, **kwargs)

      Draw a quiver plot on the plot.

      :param points: List of points, each point as [x, y, z, u, v, w]. u, v, w are the components of the vector.
      :type points: 6*1 np.ndarray
      :param kwargs: Additional plotting options.



   .. py:method:: draw_quivers(points, refresh=False, **kwargs)

      Draw a series of quiver plot on the plot.

      :param points: List of points, each point as [x, y, z, u, v, w]. u, v, w are the components of the vector.
      :type points: list or np.ndarray



   .. py:method:: draw_trajectory(traj, traj_type='g-', label='trajectory', show_direction=False, refresh=False, **kwargs)

      Draw a trajectory on the plot.

      :param traj: List of points or array of points [x, y, z].
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



