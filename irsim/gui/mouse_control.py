from matplotlib.backend_bases import MouseButton
import matplotlib.pyplot as plt
import numpy as np
from typing import Optional, Tuple, Any
from matplotlib.axes import Axes


class MouseControl:
    def __init__(self, ax: Axes, zoom_factor: float = 1.1) -> None:
        """
        Initialize MouseControl with comprehensive mouse interaction functionality.

        Mouse Controls:
        - Mouse Move: Track cursor position and update current axes
        - Middle Click (Wheel Click): Reset zoom to original view
        - Scroll Up: Zoom in (centered on mouse position)
        - Scroll Down: Zoom out (centered on mouse position)

        Args:
            ax: The matplotlib axes to control
            zoom_factor (float): Factor by which to zoom in/out. Default is 1.1.
                                Higher values = more aggressive zooming.


        Attributes:
            mouse_pos: The current mouse position
            left_click_pos: The position of the left click
            right_click_pos: The position of the right click
        """
        self.zoom_factor = zoom_factor
        self.mouse_pos = None
        self.left_click_pos = None
        self.right_click_pos = None
        self.current_axes = ax

        self.init_xlim = ax.get_xlim()
        self.init_ylim = ax.get_ylim()

        # Connect event handlers
        binding_id = plt.connect("motion_notify_event", self.on_move)
        plt.connect("button_press_event", self.on_click)
        plt.connect("button_release_event", self.on_release)
        plt.connect("scroll_event", self.on_scroll)

    def on_move(self, event: Any) -> None:
        """Handle mouse movement events."""
        if event.inaxes:
            self.mouse_pos = (event.xdata, event.ydata)
            # self.current_axes = event.inaxes

        else:
            self.mouse_pos = None
            self.current_axes = None

    def on_click(self, event: Any) -> None:
        """Handle mouse click events."""
        if event.button is MouseButton.LEFT:
            self.left_click_pos = (
                np.round((event.xdata, event.ydata), 2)
                if event.inaxes is not None
                else None
            )

        elif event.button is MouseButton.RIGHT:
            self.right_click_pos = (
                np.round((event.xdata, event.ydata), 2)
                if event.inaxes is not None
                else None
            )

        elif event.button is MouseButton.MIDDLE:
            # Middle mouse button (wheel click) resets zoom
            self.reset_zoom(event.inaxes)

    def on_release(self, event: Any) -> None:
        """Handle mouse release events."""
        if event.button is MouseButton.LEFT:
            self.left_click_pos = None

        elif event.button is MouseButton.RIGHT:
            self.right_click_pos = None

    def on_scroll(self, event: Any) -> None:
        """
        Handle mouse scroll events for zooming.

        Args:
            event: Matplotlib scroll event containing scroll direction and position.
        """
        if event.inaxes is None:
            return

        # Get current axis limits
        ax = event.inaxes
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()

        # Get mouse position in data coordinates
        xdata, ydata = event.xdata, event.ydata

        # Calculate zoom direction (scroll up = zoom in, scroll down = zoom out)
        if event.step > 0:
            # Zoom in
            scale_factor = 1 / self.zoom_factor
        else:
            # Zoom out
            scale_factor = self.zoom_factor

        # Calculate new limits centered on mouse position
        x_range = xlim[1] - xlim[0]
        y_range = ylim[1] - ylim[0]

        new_x_range = x_range * scale_factor
        new_y_range = y_range * scale_factor

        # Calculate new limits keeping mouse position as zoom center
        x_ratio = (xdata - xlim[0]) / x_range if x_range != 0 else 0.5
        y_ratio = (ydata - ylim[0]) / y_range if y_range != 0 else 0.5

        new_xlim = [xdata - new_x_range * x_ratio, xdata + new_x_range * (1 - x_ratio)]
        new_ylim = [ydata - new_y_range * y_ratio, ydata + new_y_range * (1 - y_ratio)]

        # Apply new limits
        ax.set_xlim(new_xlim)
        ax.set_ylim(new_ylim)

        # Redraw the plot
        ax.figure.canvas.draw()

    def reset_zoom(self, ax: Optional[Axes] = None) -> None:
        """
        Reset zoom to original view.

        Args:
            ax: Matplotlib axes to reset. If None, uses current axes.
        """
        if ax is None:
            ax = self.current_axes

        if ax is not None:
            ax.set_xlim(self.init_xlim)
            ax.set_ylim(self.init_ylim)
            ax.figure.canvas.draw()

    def set_zoom_factor(self, factor: float) -> None:
        """
        Set the zoom factor.

        Args:
            factor (float): New zoom factor (>1 for more aggressive zooming).
        """
        self.zoom_factor = max(1.1, float(factor))  # Minimum factor of 1.1
