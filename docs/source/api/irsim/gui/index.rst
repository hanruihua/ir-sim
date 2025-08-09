irsim.gui
=========

.. py:module:: irsim.gui


Submodules
----------

.. toctree::
   :maxdepth: 1

   /api/irsim/gui/keyboard_control/index
   /api/irsim/gui/mouse_control/index


Classes
-------

.. autoapisummary::

   irsim.gui.MouseControl


Package Contents
----------------

.. py:class:: MouseControl(ax: matplotlib.axes.Axes, zoom_factor: float = 1.1)

   
   Initialize MouseControl with comprehensive mouse interaction functionality.

   Mouse Controls:
   - Mouse Move: Track cursor position and update current axes
   - Middle Click (Wheel Click): Reset zoom to original view
   - Scroll Up: Zoom in (centered on mouse position)
   - Scroll Down: Zoom out (centered on mouse position)

   :param ax: The matplotlib axes to control
   :param zoom_factor: Factor by which to zoom in/out. Default is 1.1.
                       Higher values = more aggressive zooming.
   :type zoom_factor: float

   .. attribute:: mouse_pos

      The current mouse position

   .. attribute:: left_click_pos

      The position of the left click

   .. attribute:: right_click_pos

      The position of the right click


   .. py:attribute:: zoom_factor
      :value: 1.1



   .. py:attribute:: mouse_pos
      :value: None



   .. py:attribute:: left_click_pos
      :value: None



   .. py:attribute:: right_click_pos
      :value: None



   .. py:attribute:: current_axes


   .. py:attribute:: init_xlim


   .. py:attribute:: init_ylim


   .. py:method:: on_move(event: Any) -> None

      Handle mouse movement events.



   .. py:method:: on_click(event: Any) -> None

      Handle mouse click events.



   .. py:method:: on_release(event: Any) -> None

      Handle mouse release events.



   .. py:method:: on_scroll(event: Any) -> None

      Handle mouse scroll events for zooming.

      :param event: Matplotlib scroll event containing scroll direction and position.



   .. py:method:: reset_zoom(ax: Optional[matplotlib.axes.Axes] = None) -> None

      Reset zoom to original view.

      :param ax: Matplotlib axes to reset. If None, uses current axes.



   .. py:method:: set_zoom_factor(factor: float) -> None

      Set the zoom factor.

      :param factor: New zoom factor (>1 for more aggressive zooming).
      :type factor: float



