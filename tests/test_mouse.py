"""
Tests for mouse control functionality.

Covers mouse movement, clicks, scrolling, and zoom operations.
"""

from unittest.mock import Mock

import pytest
from matplotlib.backend_bases import MouseButton

from irsim.gui.mouse_control import MouseControl


class TestMouseControl:
    """Tests for MouseControl class."""

    @pytest.fixture
    def mouse_setup(self, env_factory):
        """Setup mouse control with environment."""
        env = env_factory("test_multi_objects_world.yaml")
        ax = env._env_plot.ax
        mouse_control = MouseControl(ax, zoom_factor=1.2)
        return env, ax, mouse_control

    def test_initial_state(self, mouse_setup):
        """Test initial state of mouse control."""
        _, _, mouse_control = mouse_setup
        assert mouse_control.mouse_pos is None
        assert mouse_control.left_click_pos is None
        assert mouse_control.right_click_pos is None
        assert mouse_control.zoom_factor == 1.2

    def test_mouse_movement(self, mouse_setup):
        """Test mouse movement tracking."""
        _, ax, mouse_control = mouse_setup
        mock_event = Mock()
        mock_event.inaxes = ax
        mock_event.xdata = 5.0
        mock_event.ydata = 5.0
        mouse_control.on_move(mock_event)
        assert mouse_control.mouse_pos == (5.0, 5.0)

    def test_mouse_movement_outside_axes(self, mouse_setup):
        """Test mouse movement outside axes clears position."""
        _, ax, mouse_control = mouse_setup
        # First move inside
        mock_event = Mock()
        mock_event.inaxes = ax
        mock_event.xdata = 5.0
        mock_event.ydata = 5.0
        mouse_control.on_move(mock_event)
        assert mouse_control.mouse_pos == (5.0, 5.0)

        # Then move outside
        mock_event.inaxes = None
        mouse_control.on_move(mock_event)
        assert mouse_control.mouse_pos is None

    def test_left_click(self, mouse_setup):
        """Test left mouse button click."""
        _, ax, mouse_control = mouse_setup
        mock_event = Mock()
        mock_event.button = MouseButton.LEFT
        mock_event.inaxes = ax
        mock_event.xdata = 3.0
        mock_event.ydata = 3.0
        mouse_control.on_click(mock_event)
        assert mouse_control.left_click_pos is not None
        assert mouse_control.left_click_pos[0] == 3.0
        assert mouse_control.left_click_pos[1] == 3.0

    def test_right_click(self, mouse_setup):
        """Test right mouse button click."""
        _, ax, mouse_control = mouse_setup
        mock_event = Mock()
        mock_event.button = MouseButton.RIGHT
        mock_event.inaxes = ax
        mock_event.xdata = 7.0
        mock_event.ydata = 7.0
        mouse_control.on_click(mock_event)
        assert mouse_control.right_click_pos is not None
        assert mouse_control.right_click_pos[0] == 7.0
        assert mouse_control.right_click_pos[1] == 7.0

    def test_scroll_zoom(self, mouse_setup):
        """Test scroll wheel zoom."""
        _, ax, mouse_control = mouse_setup
        initial_xlim = ax.get_xlim()
        initial_ylim = ax.get_ylim()

        mock_event = Mock()
        mock_event.inaxes = ax
        mock_event.xdata = 5.0
        mock_event.ydata = 5.0
        mock_event.step = 1  # Scroll up (zoom in)
        mouse_control.on_scroll(mock_event)

        assert ax.get_xlim() != initial_xlim
        assert ax.get_ylim() != initial_ylim

    def test_middle_click_reset(self, mouse_setup):
        """Test middle click resets zoom."""
        _, ax, mouse_control = mouse_setup
        initial_xlim = ax.get_xlim()
        initial_ylim = ax.get_ylim()

        # First zoom in
        mock_scroll = Mock()
        mock_scroll.inaxes = ax
        mock_scroll.xdata = 5.0
        mock_scroll.ydata = 5.0
        mock_scroll.step = 1
        mouse_control.on_scroll(mock_scroll)

        # Verify zoom changed
        assert ax.get_xlim() != initial_xlim
        assert ax.get_ylim() != initial_ylim

        # Reset with middle click
        mock_middle = Mock()
        mock_middle.button = MouseButton.MIDDLE
        mock_middle.inaxes = ax
        mouse_control.on_click(mock_middle)

        # Verify reset
        assert ax.get_xlim() == initial_xlim
        assert ax.get_ylim() == initial_ylim

    def test_set_zoom_factor(self, mouse_setup):
        """Test setting zoom factor."""
        _, _, mouse_control = mouse_setup
        mouse_control.set_zoom_factor(1.5)
        assert mouse_control.zoom_factor == 1.5

    def test_set_zoom_factor_minimum(self, mouse_setup):
        """Test zoom factor enforces minimum."""
        _, _, mouse_control = mouse_setup
        mouse_control.set_zoom_factor(1.0)  # Below minimum
        assert mouse_control.zoom_factor == 1.1  # Should be set to minimum

    def test_left_release(self, mouse_setup):
        """Test left mouse button release."""
        _, ax, mouse_control = mouse_setup
        # First click
        mock_click = Mock()
        mock_click.button = MouseButton.LEFT
        mock_click.inaxes = ax
        mock_click.xdata = 3.0
        mock_click.ydata = 3.0
        mouse_control.on_click(mock_click)
        assert mouse_control.left_click_pos is not None

        # Then release
        mock_release = Mock()
        mock_release.button = MouseButton.LEFT
        mouse_control.on_release(mock_release)
        assert mouse_control.left_click_pos is None

    def test_right_release(self, mouse_setup):
        """Test right mouse button release."""
        _, ax, mouse_control = mouse_setup
        # First click
        mock_click = Mock()
        mock_click.button = MouseButton.RIGHT
        mock_click.inaxes = ax
        mock_click.xdata = 7.0
        mock_click.ydata = 7.0
        mouse_control.on_click(mock_click)
        assert mouse_control.right_click_pos is not None

        # Then release
        mock_release = Mock()
        mock_release.button = MouseButton.RIGHT
        mouse_control.on_release(mock_release)
        assert mouse_control.right_click_pos is None
