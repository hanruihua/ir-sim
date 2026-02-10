"""
Tests for plotting functionality (EnvPlot, EnvPlot3D, draw_patch).

Covers 2D and 3D plotting, trajectory drawing, quiver drawing, and shape patches.
"""

import os
from unittest.mock import Mock

import matplotlib.pyplot as plt
import numpy as np
import pytest

from irsim.env.env_plot import EnvPlot, draw_patch
from irsim.env.env_plot3d import EnvPlot3D


class TestEnvPlot2D:
    """Tests for 2D plotting functionality."""

    @pytest.fixture
    def plot_2d(self, dummy_world_2d, dummy_logger):
        """Create a 2D plot for testing."""
        plot = EnvPlot(
            dummy_world_2d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=True,
        )
        yield plot
        plt.close("all")

    def test_init_plot_branches(self, dummy_world_2d, dummy_logger):
        """Test init_plot with different parameters."""
        plot = EnvPlot(
            dummy_world_2d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=True,
        )
        plot._init_plot(dummy_world_2d, [], no_axis=True, tight=True)

    def test_draw_trajectory_list(self, plot_2d):
        """Test draw_trajectory with list input."""
        traj_list = [
            np.array([[1.0], [2.0], [0.0]]),
            np.array([[2.0], [3.0], [0.5]]),
        ]
        plot_2d.draw_trajectory(traj_list, show_direction=True, refresh=True)

    def test_draw_points_list(self, plot_2d):
        """Test draw_points with list of points."""
        plot_2d.draw_points([[1.0, 2.0], [3.0, 4.0]], s=5, c="r", refresh=True)

    def test_draw_points_ndarray(self, plot_2d):
        """Test draw_points with numpy array."""
        pts = np.array([[1.0, 2.0], [3.0, 4.0]])
        plot_2d.draw_points(pts, s=5, c="b", refresh=True)

    def test_draw_points_column(self, plot_2d):
        """Test draw_points with column vector."""
        col = np.array([[1.0], [2.0]])
        plot_2d.draw_points(col, s=5, c="g", refresh=True)

    def test_draw_quiver_none(self, plot_2d):
        """Test draw_quiver with None input."""
        plot_2d.draw_quiver(None)

    def test_draw_quiver_array(self, plot_2d):
        """Test draw_quiver with array input."""
        plot_2d.draw_quiver(np.array([1.0, 2.0, 0.5, 0.25]), refresh=True)

    def test_draw_quivers(self, plot_2d):
        """Test draw_quivers with list of arrays."""
        plot_2d.draw_quivers([np.array([1.0, 2.0, 0.3, 0.1])], refresh=True)

    def test_clear_components(self, plot_2d):
        """Test clearing all dynamic components."""
        plot_2d.clear_components("all", [])

    def test_set_ax_viewpoint_none_objects(self, dummy_world_2d, dummy_logger):
        """Test set_ax_viewpoint with None objects."""
        plot = EnvPlot(
            dummy_world_2d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=True,
        )
        plot.viewpoint = "some_name"
        plot.set_ax_viewpoint(objects=None)


class TestEnvPlot3D:
    """Tests for 3D plotting functionality."""

    @pytest.fixture
    def plot_3d(self, dummy_world_3d, dummy_logger):
        """Create a 3D plot for testing."""
        plot = EnvPlot3D(
            dummy_world_3d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=True,
        )
        yield plot
        plt.close("all")

    def test_draw_points_list_3d(self, plot_3d):
        """Test draw_points with list of 3D points."""
        plot_3d.draw_points(
            [[1.0, 2.0, 1.0], [2.0, 3.0, 1.5]], s=5, c="m", refresh=True
        )

    def test_draw_points_ndarray_3d(self, plot_3d):
        """Test draw_points with 3D numpy array."""
        pts3 = np.array([[1.0, 2.0], [2.0, 3.0], [0.5, 0.7]])
        plot_3d.draw_points(pts3, s=5, c="c", refresh=True)

    def test_draw_points_column_3d(self, plot_3d):
        """Test draw_points with 3D column vector."""
        col3 = np.array([[1.0], [2.0], [0.5]])
        plot_3d.draw_points(col3, s=5, c="y", refresh=True)

    def test_draw_quiver_none_3d(self, plot_3d):
        """Test draw_quiver with None input in 3D."""
        plot_3d.draw_quiver(None)

    def test_draw_quiver_array_3d(self, plot_3d):
        """Test draw_quiver with array input in 3D."""
        plot_3d.draw_quiver(np.array([1.0, 2.0, 0.5, 0.1, 0.2, 0.3]), refresh=True)

    def test_draw_quivers_3d(self, plot_3d):
        """Test draw_quivers with list of 3D arrays."""
        plot_3d.draw_quivers([np.array([1.0, 2.0, 0.5, 0.1, 0.2, 0.3])], refresh=True)


class TestDrawPatch:
    """Tests for draw_patch function with various shapes."""

    @pytest.fixture
    def ax_2d(self, dummy_world_2d, dummy_logger):
        """Create a 2D axes for testing."""
        plot = EnvPlot(
            dummy_world_2d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=False,
        )
        yield plot.ax
        plt.close("all")

    @pytest.fixture
    def ax_3d(self, dummy_world_3d, dummy_logger):
        """Create a 3D axes for testing."""
        plot = EnvPlot3D(
            dummy_world_3d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=True,
        )
        yield plot.ax
        plt.close("all")

    def test_draw_circle(self, ax_2d):
        """Test drawing circle patch."""
        state = np.array([[0.0], [0.0], [0.0]])
        circ = draw_patch(
            ax_2d, "circle", state=state, radius=1.0, color="k", alpha=0.5, zorder=1
        )
        assert circ is not None

    def test_draw_rectangle_vertices(self, ax_2d):
        """Test drawing rectangle from vertices."""
        state = np.array([[0.0], [0.0], [0.0]])
        rect_vertices = np.array([[0.0, 1.0, 1.0, 0.0], [0.0, 0.0, 0.5, 0.5]])
        rect = draw_patch(
            ax_2d, "rectangle", state=state, vertices=rect_vertices, color="r"
        )
        assert rect is not None

    def test_draw_rectangle_width_height(self, ax_2d):
        """Test drawing rectangle from width/height."""
        state = np.array([[0.0], [0.0], [0.0]])
        rect = draw_patch(
            ax_2d,
            "rectangle",
            state=state,
            vertices=np.array([[0.0, 0.0]]).T,
            width=1.0,
            height=0.5,
            color="g",
        )
        assert rect is not None

    def test_draw_rectangle_missing_params_raises(self, ax_2d):
        """Test rectangle without vertices or width/height raises."""
        state = np.array([[0.0], [0.0], [0.0]])
        with pytest.raises(
            ValueError, match="rectangle requires either vertices or width/height"
        ):
            draw_patch(ax_2d, "rectangle", state=state)

    def test_draw_rectangle_partial_params_raises(self, ax_2d):
        """Test rectangle with width/height but no vertices raises."""
        state = np.array([[0.0], [0.0], [0.0]])
        with pytest.raises(TypeError):
            draw_patch(ax_2d, "rectangle", state=state, width=1.0, height=0.5)

    def test_draw_polygon(self, ax_2d):
        """Test drawing polygon patch."""
        state = np.array([[0.0], [0.0], [0.0]])
        poly_vertices = np.array([[0.0, 0.5, 0.0], [0.0, 0.5, 0.5]])
        poly = draw_patch(
            ax_2d, "polygon", state=state, vertices=poly_vertices, color="b"
        )
        assert poly is not None

    def test_draw_ellipse(self, ax_2d):
        """Test drawing ellipse patch."""
        state = np.array([[0.0], [0.0], [0.0]])
        ell = draw_patch(
            ax_2d, "ellipse", state=state, width=0.5, height=0.2, color="c"
        )
        assert ell is not None

    def test_draw_wedge_theta(self, ax_2d):
        """Test drawing wedge with theta1/theta2."""
        state = np.array([[0.0], [0.0], [0.0]])
        wedge = draw_patch(
            ax_2d,
            "wedge",
            state=state,
            radius=1.0,
            theta1=-45.0,
            theta2=45.0,
            color="m",
        )
        assert wedge is not None

    def test_draw_wedge_fov(self, ax_2d):
        """Test drawing wedge with field of view."""
        state = np.array([[0.0], [0.0], [0.0]])
        wedge = draw_patch(
            ax_2d, "wedge", state=state, radius=1.0, fov=np.pi / 2, color="y"
        )
        assert wedge is not None

    def test_draw_arrow(self, ax_2d):
        """Test drawing arrow patch."""
        state = np.array([[0.0], [0.0], [0.0]])
        arr = draw_patch(
            ax_2d, "arrow", state=state, arrow_length=0.3, arrow_width=0.4, color="k"
        )
        assert arr is not None

    def test_draw_line(self, ax_2d):
        """Test drawing line."""
        line_vertices = np.array([[0.0, 1.0], [0.0, 1.0]])
        line = draw_patch(ax_2d, "line", vertices=line_vertices, color="k")
        assert line is not None

    def test_draw_linestring(self, ax_2d):
        """Test drawing linestring (alias for line)."""
        line_vertices = np.array([[0.0, 1.0], [0.0, 1.0]])
        ls = draw_patch(ax_2d, "linestring", vertices=line_vertices, color="k")
        assert ls is not None

    def test_draw_unknown_shape_raises(self, ax_2d):
        """Test unknown shape type raises ValueError."""
        state = np.array([[0.0], [0.0], [0.0]])
        with pytest.raises(ValueError, match="Unsupported shape type"):
            draw_patch(ax_2d, "unknown-shape", state=state)

    def test_draw_line_3d(self, ax_3d):
        """Test drawing line on 3D axes."""
        line_vertices = np.array([[0.0, 1.0], [0.0, 1.0]])
        line3d = draw_patch(
            ax_3d, "line", vertices=line_vertices, color="k", alpha=0.8, zorder=2
        )
        assert line3d is not None

    def test_draw_circle_3d(self, ax_3d):
        """Test drawing circle on 3D axes (patch_2d_to_3d conversion)."""
        state = np.array([[0.0], [0.0], [0.0]])
        circ3d = draw_patch(ax_3d, "circle", state=state, radius=0.2, color="r")
        assert circ3d is not None


class TestGoalText:
    """Tests for goal abbreviation text rendering."""

    def test_goal_abbr_text_update(self, env_factory):
        """Test goal abbreviation text position and property updates."""
        env = env_factory("test_all_objects.yaml")
        robot = env.robot
        robot.set_goal([8, 8, 0])

        mock_text = Mock()
        robot.goal_abbr_text = mock_text

        robot._step_plot(text_color="red", text_size=12, text_alpha=0.8, text_zorder=10)

        mock_text.set_position.assert_called()
        mock_text.set_color.assert_called_with("red")
        mock_text.set_fontsize.assert_called_with(12)
        mock_text.set_alpha.assert_called_with(0.8)
        mock_text.set_zorder.assert_called_with(10)

    def test_goal_abbr_text_creation(self, env_factory):
        """Test goal abbreviation text is created during plot_object."""
        env = env_factory("test_all_objects.yaml")
        robot = env.robot
        robot.set_goal([8, 8, 0])
        robot.show_goal = True
        robot.show_goal_text = True
        robot.show_text = True

        fig, ax = plt.subplots()
        robot.plot_object(
            ax, text_color="blue", text_size=10, text_alpha=0.5, text_zorder=5
        )

        assert hasattr(robot, "goal_abbr_text")
        assert robot.goal_abbr_text is not None
        plt.close(fig)


class TestSaveFigure:
    """Tests for figure saving functionality."""

    def test_save_figure_no_extension(self, env_factory):
        """Test save_figure with filename without extension."""
        env = env_factory("test_collision_world.yaml")
        env.render()
        env.save_figure(save_name="test_no_ext")
        if os.path.exists("test_no_ext.png"):
            os.remove("test_no_ext.png")

    def test_save_figure_multiple_dots(self, env_factory):
        """Test save_figure with filename containing multiple dots."""
        env = env_factory("test_collision_world.yaml")
        env.render()
        env.save_figure(save_name="test.file.name.png")
        if os.path.exists("test.file.name.png"):
            os.remove("test.file.name.png")


class TestRenderProperties:
    """Tests for render method with various property arguments."""

    @pytest.mark.parametrize("projection", ["2d", "3d"])
    def test_render_with_properties(self, env_factory, projection):
        """Test render with various object property arguments."""
        env = env_factory(
            "test_collision_avoidance.yaml", full=True, projection=projection
        )

        for i in range(8):
            env.step()
            if i % 4 == 0:
                env.render(0.01, obj_color="red", obj_alpha=0.7, obj_zorder=5)
            elif i % 4 == 1:
                env.render(
                    0.01,
                    obj_linestyle="--",
                    traj_color="blue",
                    traj_alpha=0.8,
                    traj_width=0.3,
                )
            elif i % 4 == 2:
                env.render(
                    0.01,
                    goal_color="green",
                    goal_alpha=0.6,
                    goal_zorder=3,
                    arrow_color="orange",
                    arrow_alpha=0.9,
                    arrow_zorder=4,
                )
            else:
                env.render(
                    0.01,
                    text_color="purple",
                    text_size=14,
                    fov_color="cyan",
                    fov_alpha=0.4,
                    traj_style="--",
                    traj_zorder=3,
                    text_alpha=0.5,
                    text_zorder=4,
                )

            env.draw_trajectory(env.robot.trajectory, show_direction=True)
            if env.done():
                break


class TestDrawMethods:
    """Tests for environment draw methods."""

    def test_draw_quiver_env(self, env_factory):
        """Test environment draw_quiver method."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env.draw_quiver(np.array([1, 2, 2, 3]))
        env.draw_quiver(np.array([1, 2, 2, 3]), refresh=True)

    def test_draw_quivers_env(self, env_factory):
        """Test environment draw_quivers method."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        points = [np.array([1, 3, 2, 3]), np.array([1, 2, 2, 5])]
        env.draw_quivers(points)
        env.draw_quivers(points, refresh=True)

    def test_set_title(self, env_factory):
        """Test setting plot title."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env.set_title(f"Simulation time: {env.time:.2f}s")


# ---------------------------------------------------------------------------
# Coverage-targeted tests for env_plot.py
# ---------------------------------------------------------------------------


class TestSetPatchProperty:
    """Tests for set_patch_property kwargs (lines 831, 841, 905-908)."""

    def test_set_patch_property_fill_and_linewidth(self, dummy_world_2d, dummy_logger):
        """set_patch_property with fill and linewidth kwargs."""
        from irsim.env.env_plot import draw_patch, set_patch_property

        plot = EnvPlot(
            dummy_world_2d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=False,
        )
        state = np.array([[0.0], [0.0], [0.0]])
        circ = draw_patch(plot.ax, "circle", state=state, radius=1.0, color="k")
        set_patch_property(
            circ,
            plot.ax,
            state=state,
            fill=True,
            linewidth=2.0,
            linestyle="--",
            alpha=0.5,
            zorder=3,
        )
        plt.close("all")


class TestStepObjectsPlotInvalidMode:
    """Tests for step_objects_plot and draw_components with invalid mode (lines 183, 187, 209)."""

    def test_step_objects_plot_invalid_mode(self, dummy_world_2d, dummy_logger):
        """step_objects_plot with invalid mode logs error (line 187)."""
        plot = EnvPlot(
            dummy_world_2d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=False,
        )
        plot.step_objects_plot(mode="invalid_mode", objects=[])
        plt.close("all")

    def test_draw_components_invalid_mode(self, dummy_world_2d, dummy_logger):
        """draw_components with invalid mode logs error (line 209)."""
        plot = EnvPlot(
            dummy_world_2d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=False,
        )
        plot.draw_components(mode="invalid_mode", objects=[])
        plt.close("all")


class TestDrawPatchLineWithAlphaFill:
    """Tests for draw_patch line shape with fill and alpha kwargs (lines 831, 841)."""

    def test_draw_line_with_fill_and_alpha(self, dummy_world_2d, dummy_logger):
        """draw_patch line with alpha (line 841)."""
        plot = EnvPlot(
            dummy_world_2d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=False,
        )
        line_vertices = np.array([[0.0, 1.0], [0.0, 1.0]])
        line = draw_patch(
            plot.ax,
            "line",
            vertices=line_vertices,
            color="k",
            alpha=0.5,
            linestyle="--",
        )
        assert line is not None
        plt.close("all")
