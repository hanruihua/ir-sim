"""
Tests for plotting functionality (EnvPlot, EnvPlot3D, draw_patch).

Covers 2D and 3D plotting, trajectory drawing, quiver drawing, and shape patches.
"""

import contextlib
import os
from dataclasses import FrozenInstanceError
from pathlib import Path
from unittest.mock import Mock

import matplotlib.pyplot as plt
import numpy as np
import pytest
import yaml
from matplotlib.colors import to_rgba

import irsim
import irsim.env.env_plot as env_plot_module
from irsim.env.env_plot import EnvPlot, draw_patch
from irsim.env.env_plot3d import EnvPlot3D
from irsim.world.object_base import ObjectBase
from irsim.world.object_plot import ObjectPlotOptions


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

    def test_draw_points_none(self, plot_2d):
        """draw_points(None) is a no-op."""
        assert plot_2d.draw_points(None) is None

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

    def test_draw_grid_map_3d_without_logger(self):
        """draw_grid_map on a 3D axis must not crash when no env logger is set.

        self.logger resolves to env_param.logger, which defaults to None, so the
        "Map will not show in 3D plot" warning must be guarded.
        """
        from types import SimpleNamespace

        fig = plt.figure()
        ax3d = fig.add_subplot(111, projection="3d")
        # No env logger configured -> logger is None.
        fake = SimpleNamespace(
            ax=ax3d, x_range=[0, 10], y_range=[0, 10], logger=None, _grid_im=None
        )
        EnvPlot.draw_grid_map(fake, np.zeros((10, 10)))  # must not raise
        plt.close("all")

    def test_draw_grid_map_3d_warns_with_logger(self):
        """When a logger is present the 3D grid-map warning is still emitted."""
        from types import SimpleNamespace

        fig = plt.figure()
        ax3d = fig.add_subplot(111, projection="3d")
        logger = Mock()
        fake = SimpleNamespace(
            ax=ax3d, x_range=[0, 10], y_range=[0, 10], logger=logger, _grid_im=None
        )
        EnvPlot.draw_grid_map(fake, np.zeros((10, 10)))
        logger.warning.assert_called_once()
        plt.close("all")

    def test_draw_grid_map_none_drops_existing(self):
        """draw_grid_map(None) removes a previously drawn grid image (e.g. on a
        reload into a world with no obstacle map)."""
        from types import SimpleNamespace

        grid_im = Mock()
        fake = SimpleNamespace(
            ax=None, x_range=[0, 10], y_range=[0, 10], logger=None, _grid_im=grid_im
        )
        EnvPlot.draw_grid_map(fake, None)
        grid_im.remove.assert_called_once()
        assert fake._grid_im is None


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

    def test__goal_text_update(self, env_factory):
        """Test goal abbreviation text position and property updates."""
        env = env_factory("test_all_objects.yaml")
        robot = env.robot
        robot.set_goal([8, 8, 0])

        mock_text = Mock()
        robot._goal_text = mock_text

        robot._step_plot(text_color="red", text_size=12, text_alpha=0.8, text_zorder=10)

        mock_text.set_position.assert_called()
        mock_text.set_color.assert_called_with("red")
        mock_text.set_fontsize.assert_called_with(12)
        mock_text.set_alpha.assert_called_with(0.8)
        mock_text.set_zorder.assert_called_with(10)

    def test__goal_text_creation(self, env_factory):
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

        assert hasattr(robot, "_goal_text")
        assert robot._goal_text is not None
        plt.close(fig)


class TestObjectPlot:
    """Tests for the object-owned renderer boundary."""

    def test_artist_state_stays_on_owner(self, env_factory):
        env = env_factory("test_all_objects.yaml")
        robot = env.robot

        assert robot._object_plot.owner is robot
        assert robot._object_plot._owner is robot
        assert "object_patch" in vars(robot)
        assert "object_patch" not in vars(robot._object_plot)

    def test_legacy_renderer_attributes_delegate_to_owner(self):
        obj = ObjectBase(
            shape={"name": "circle", "radius": 0.2},
            color="navy",
            plot={"show_goal": True},
        )

        assert obj._object_plot.color == "navy"
        assert obj._object_plot.plot_kwargs == {"show_goal": True}

    def test_owner_aliases_stay_synchronized(self):
        first = ObjectBase(shape={"name": "circle", "radius": 0.2})
        second = ObjectBase(shape={"name": "circle", "radius": 0.3})

        first._object_plot.owner = second

        assert first._object_plot.owner is second
        assert first._object_plot._owner is second
        assert first._object_plot.radius == second.radius

    def test_goal_text_is_optional_without_goal(self):
        obj = ObjectBase(shape={"name": "circle", "radius": 0.2}, goal=None)
        obj.show_goal = True
        obj.show_goal_text = True

        fig, ax = plt.subplots()
        obj.plot_text(ax)

        assert hasattr(obj, "_text")
        assert not hasattr(obj, "_goal_text")
        plt.close(fig)

    def test_initial_object_style_uses_plot_overrides(self):
        obj = ObjectBase(shape={"name": "circle", "radius": 0.2})

        fig, ax = plt.subplots()
        obj.plot_object(
            ax,
            obj_color="magenta",
            obj_alpha=0.25,
            obj_linewidth=2.5,
        )

        np.testing.assert_allclose(
            obj.object_patch.get_facecolor(), to_rgba("magenta", alpha=0.25)
        )
        assert obj.object_patch.get_linewidth() == 2.5
        plt.close(fig)

    @pytest.mark.parametrize(
        ("plot", "render_options", "expected_zorder"),
        [
            ({}, {}, 2),
            ({"obj_zorder": 7}, {}, 7),
            ({}, {"obj_zorder": 9}, 9),
        ],
    )
    def test_object_image_preserves_legacy_zorder(
        self, plot, render_options, expected_zorder
    ):
        obj = ObjectBase(
            shape={"name": "rectangle", "length": 0.5, "width": 0.2},
            role="robot",
            description="diff_robot0.png",
            plot=plot,
        )

        fig, ax = plt.subplots()
        obj.plot_object(ax, **render_options)

        assert obj.object_img.get_zorder() == expected_zorder
        assert (
            obj._object_plot._resolve_options(render_options).image.zorder
            == expected_zorder
        )
        plt.close(fig)

    def test_plot_options_are_grouped_immutable_values(self):
        obj = ObjectBase(
            shape={"name": "circle", "radius": 0.2},
            color="navy",
            plot={
                "show_goal": True,
                "obj_alpha": 0.3,
                "keep_traj_length": 12,
                "text_position": [0.4, 0.6],
                "keep_trail_length": 8,
                "arrow_length": 0.9,
                "fov_color": "cyan",
            },
        )

        options = obj._object_plot.options

        assert isinstance(options, ObjectPlotOptions)
        assert options.visibility.goal
        assert options.object.alpha == 0.3
        assert options.image.zorder == 2
        assert options.goal.color == "navy"
        assert options.trajectory.keep_length == 12
        assert options.text.position == (0.4, 0.6)
        assert options.trail.keep_length == 8
        assert options.arrow.length == 0.9
        assert options.fov.color == "cyan"
        assert not hasattr(options, "__dict__")

        with pytest.raises(FrozenInstanceError):
            options.object.alpha = 0.8

        overridden = options.with_overrides({"obj_alpha": 0.8})
        assert options.object.alpha == 0.3
        assert overridden.object.alpha == 0.8

    def test_plot_dataclasses_have_fully_resolved_defaults(self):
        options = ObjectPlotOptions()

        assert options.object.alpha == 1.0
        assert options.object.zorder == 1
        assert options.object.linestyle == "-"
        assert options.object.line_width == 1.0
        assert options.image.zorder == 2
        assert options.goal.alpha == 0.5
        assert options.trajectory.alpha == 0.5
        assert options.text.alpha == 1.0
        assert options.trail.alpha == 0.7
        assert options.arrow.alpha == 1.0
        assert options.fov.alpha == 0.5

    def test_initial_overrides_update_live_plot_config(self):
        stored_options = {"obj_color": "blue"}
        obj = ObjectBase(
            shape={"name": "circle", "radius": 0.2},
            plot=stored_options,
        )

        fig, ax = plt.subplots()
        obj._init_plot(ax, obj_color="magenta")

        assert stored_options["obj_color"] == "magenta"
        assert obj.plot_kwargs["obj_color"] == "magenta"
        assert obj._object_plot.options.object.color == "magenta"

        obj._step_plot(obj_color="red")
        np.testing.assert_allclose(obj.object_patch.get_facecolor(), to_rgba("red"))

        obj._step_plot()
        np.testing.assert_allclose(obj.object_patch.get_facecolor(), to_rgba("magenta"))
        plt.close(fig)

    def test_owner_defaults_are_resolved_at_render_time(self):
        obj = ObjectBase(
            shape={"name": "circle", "radius": 0.2},
            color="blue",
        )
        obj.color = "red"

        fig, ax = plt.subplots()
        obj._init_plot(ax)

        np.testing.assert_allclose(obj.object_patch.get_facecolor(), to_rgba("red"))
        assert obj._object_plot.options.object.color == "red"

        obj.color = "orange"
        obj._step_plot()
        np.testing.assert_allclose(obj.object_patch.get_facecolor(), to_rgba("orange"))
        plt.close(fig)

    def test_plot_config_changes_are_resolved_at_render_time(self):
        obj = ObjectBase(
            shape={"name": "circle", "radius": 0.2},
            plot={"obj_color": "blue"},
        )
        obj.plot_kwargs["obj_color"] = "green"

        fig, ax = plt.subplots()
        obj._init_plot(ax)

        np.testing.assert_allclose(obj.object_patch.get_facecolor(), to_rgba("green"))
        assert obj._object_plot.options.object.color == "green"

        obj.plot_kwargs["obj_color"] = "purple"
        obj._step_plot()
        np.testing.assert_allclose(obj.object_patch.get_facecolor(), to_rgba("purple"))
        plt.close(fig)

    def test_legacy_plot_helpers_resolve_direct_call_options(self):
        obj = ObjectBase(
            shape={"name": "circle", "radius": 0.2},
            state=[0.0, 0.0, 0.0],
            goal=[1.0, 1.0, 0.0],
            color="navy",
        )
        trajectory = [obj.state.copy(), np.array([[0.5], [0.25], [0.0]])]

        fig, ax = plt.subplots()
        obj._plot(ax, obj.state, obj.vertices)

        obj.plot_trajectory(
            ax,
            trajectory,
            keep_traj_length=1,
            traj_color="red",
            traj_style="--",
            traj_alpha=0.4,
            traj_zorder=6,
        )
        trajectory_line = obj.trajectory_line[0]
        assert obj.keep_traj_length == 1
        assert trajectory_line.get_color() == "red"
        assert trajectory_line.get_linestyle() == "--"
        assert trajectory_line.get_alpha() == 0.4
        assert trajectory_line.get_zorder() == 6

        obj.plot_goal(ax)
        obj.plot_goal(
            ax,
            obj.goal,
            obj.goal_vertices,
            goal_color="green",
            goal_zorder=7,
            goal_alpha=0.3,
        )
        np.testing.assert_allclose(
            obj.goal_patch.get_facecolor(), to_rgba("green", alpha=0.3)
        )
        assert obj.goal_patch.get_zorder() == 7

        obj.plot_arrow(
            ax,
            obj.state,
            arrow_theta=0.0,
            arrow_length=0.7,
            arrow_width=0.2,
            arrow_color="orange",
            arrow_zorder=8,
            arrow_alpha=0.6,
        )
        np.testing.assert_allclose(
            obj.arrow_patch.get_facecolor(), to_rgba("orange", alpha=0.6)
        )
        assert obj.arrow_patch.get_zorder() == 8

        obj.plot_trail(
            ax,
            obj.state,
            obj.original_vertices,
            keep_trail_length=1,
            trail_color="yellow",
        )
        first_trail = obj.plot_trail_list[0]
        obj.plot_trail(
            ax,
            obj.state,
            obj.original_vertices,
            keep_trail_length=1,
            trail_color="cyan",
        )
        assert len(obj.plot_trail_list) == 1
        assert obj.plot_trail_list[0] is not first_trail
        plt.close(fig)

    def test_legacy_plot_helper_guard_paths(self):
        obj = ObjectBase(shape={"name": "circle", "radius": 0.2})

        fig, ax = plt.subplots()
        obj.plot_object_image(ax)
        obj.plot_object_image(
            ax,
            obj.state,
            obj.vertices,
            description="__missing_plot_image__.png",
        )
        obj.plot_fov(ax)
        obj.plot_uncertainty(ax)

        assert not hasattr(obj, "object_img")
        assert not hasattr(obj, "fov_patch")
        plt.close(fig)

    def test_object_line_update_applies_geometry_and_style(self):
        obj = ObjectBase(
            shape={"name": "linestring", "vertices": [[-1.0, 0.0], [1.0, 0.0]]}
        )
        obj.object_line = Mock()
        style = obj._object_plot._resolve_options(
            {
                "obj_color": "purple",
                "obj_alpha": 0.4,
                "obj_zorder": 9,
                "obj_linestyle": "--",
                "obj_linewidth": 2.5,
            }
        ).object

        obj._object_plot._update_object_line(2.0, 3.0, np.pi / 2, style)

        rotation = np.array([[0.0, -1.0], [1.0, 0.0]])
        expected_vertices = rotation @ obj.vertices + np.array([[2.0], [3.0]])
        x_data, y_data = obj.object_line.set_data.call_args.args
        np.testing.assert_allclose(x_data, expected_vertices[0, :], atol=1e-12)
        np.testing.assert_allclose(y_data, expected_vertices[1, :], atol=1e-12)
        obj.object_line.set_linestyle.assert_called_once_with("--")
        obj.object_line.set_color.assert_called_once_with("purple")
        obj.object_line.set_alpha.assert_called_once_with(0.4)
        obj.object_line.set_zorder.assert_called_once_with(9)
        obj.object_line.set_linewidth.assert_called_once_with(2.5)


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


class TestSaveAnimate:
    """Tests for animation saving behavior (GIF/MP4 and cleanup)."""

    @staticmethod
    def _create_dummy_frames(buffer_dir, n_frames=3):
        buffer_dir.mkdir(parents=True, exist_ok=True)
        for idx in range(1, n_frames + 1):
            (buffer_dir / f"frame_{idx:04d}.png").write_bytes(b"png")

    def test_save_animate_gif(self, dummy_world_2d, dummy_logger, tmp_path):
        """Test saving GIF animation from buffered frames."""
        plot = EnvPlot(
            dummy_world_2d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=False,
        )

        ani_dir = tmp_path / "animations"
        buffer_dir = tmp_path / "buffer"
        self._create_dummy_frames(buffer_dir, n_frames=3)

        writer = Mock()
        context_manager = Mock()
        context_manager.__enter__ = Mock(return_value=writer)
        context_manager.__exit__ = Mock(return_value=False)

        get_writer_mock = Mock(return_value=context_manager)
        imread_mock = Mock(return_value=np.zeros((4, 4, 3), dtype=np.uint8))
        original_ani_path = env_plot_module.pm.ani_path
        original_ani_buffer_path = env_plot_module.pm.ani_buffer_path
        original_get_writer = env_plot_module.imageio.get_writer
        original_imread = env_plot_module.imageio.imread

        try:
            env_plot_module.pm.ani_path = str(ani_dir)
            env_plot_module.pm.ani_buffer_path = str(buffer_dir)
            env_plot_module.imageio.get_writer = get_writer_mock
            env_plot_module.imageio.imread = imread_mock

            plot.save_animate(
                ani_name="gif_test",
                suffix=".gif",
                last_frame_duration=2,
                rm_fig_path=False,
            )
        finally:
            env_plot_module.pm.ani_path = original_ani_path
            env_plot_module.pm.ani_buffer_path = original_ani_buffer_path
            env_plot_module.imageio.get_writer = original_get_writer
            env_plot_module.imageio.imread = original_imread

        assert get_writer_mock.call_count == 1
        assert writer.append_data.call_count == 3
        assert buffer_dir.exists()

    def test_save_animate_mp4(self, dummy_world_2d, dummy_logger, tmp_path):
        """Test saving MP4 animation from buffered frames."""
        plot = EnvPlot(
            dummy_world_2d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=False,
        )

        ani_dir = tmp_path / "animations"
        buffer_dir = tmp_path / "buffer"
        self._create_dummy_frames(buffer_dir, n_frames=2)

        writer = Mock()
        context_manager = Mock()
        context_manager.__enter__ = Mock(return_value=writer)
        context_manager.__exit__ = Mock(return_value=False)

        get_writer_mock = Mock(return_value=context_manager)
        imread_mock = Mock(return_value=np.zeros((4, 4, 3), dtype=np.uint8))
        original_ani_path = env_plot_module.pm.ani_path
        original_ani_buffer_path = env_plot_module.pm.ani_buffer_path
        original_get_writer = env_plot_module.imageio.get_writer
        original_imread = env_plot_module.imageio.imread

        try:
            env_plot_module.pm.ani_path = str(ani_dir)
            env_plot_module.pm.ani_buffer_path = str(buffer_dir)
            env_plot_module.imageio.get_writer = get_writer_mock
            env_plot_module.imageio.imread = imread_mock

            plot.save_animate(
                ani_name="mp4_test",
                suffix=".mp4",
                fps=7,
                rm_fig_path=False,
            )
        finally:
            env_plot_module.pm.ani_path = original_ani_path
            env_plot_module.pm.ani_buffer_path = original_ani_buffer_path
            env_plot_module.imageio.get_writer = original_get_writer
            env_plot_module.imageio.imread = original_imread

        assert get_writer_mock.call_count == 1
        assert writer.append_data.call_count == 2

    def test_save_animate_empty_buffer(self, dummy_world_2d, dummy_logger, tmp_path):
        """Test save_animate with an empty frame buffer."""
        plot = EnvPlot(
            dummy_world_2d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=False,
        )

        ani_dir = tmp_path / "animations"
        buffer_dir = tmp_path / "buffer"
        buffer_dir.mkdir(parents=True, exist_ok=True)

        get_writer_mock = Mock()
        original_ani_path = env_plot_module.pm.ani_path
        original_ani_buffer_path = env_plot_module.pm.ani_buffer_path
        original_get_writer = env_plot_module.imageio.get_writer

        try:
            env_plot_module.pm.ani_path = str(ani_dir)
            env_plot_module.pm.ani_buffer_path = str(buffer_dir)
            env_plot_module.imageio.get_writer = get_writer_mock

            plot.save_animate(ani_name="no_images", suffix=".gif")
        finally:
            env_plot_module.pm.ani_path = original_ani_path
            env_plot_module.pm.ani_buffer_path = original_ani_buffer_path
            env_plot_module.imageio.get_writer = original_get_writer

        get_writer_mock.assert_not_called()

    def test_save_animate_remove_buffer(self, dummy_world_2d, dummy_logger, tmp_path):
        """Test frame buffer cleanup after animation save."""
        plot = EnvPlot(
            dummy_world_2d,
            objects=[],
            saved_figure={},
            figure_pixels=[200, 150],
            show_title=False,
        )

        ani_dir = tmp_path / "animations"
        buffer_dir = tmp_path / "buffer"
        self._create_dummy_frames(buffer_dir, n_frames=1)

        writer = Mock()
        context_manager = Mock()
        context_manager.__enter__ = Mock(return_value=writer)
        context_manager.__exit__ = Mock(return_value=False)
        get_writer_mock = Mock(return_value=context_manager)
        imread_mock = Mock(return_value=np.zeros((4, 4, 3), dtype=np.uint8))
        original_ani_path = env_plot_module.pm.ani_path
        original_ani_buffer_path = env_plot_module.pm.ani_buffer_path
        original_get_writer = env_plot_module.imageio.get_writer
        original_imread = env_plot_module.imageio.imread

        try:
            env_plot_module.pm.ani_path = str(ani_dir)
            env_plot_module.pm.ani_buffer_path = str(buffer_dir)
            env_plot_module.imageio.get_writer = get_writer_mock
            env_plot_module.imageio.imread = imread_mock

            plot.save_animate(ani_name="cleanup_test", suffix=".gif", rm_fig_path=True)
        finally:
            env_plot_module.pm.ani_path = original_ani_path
            env_plot_module.pm.ani_buffer_path = original_ani_buffer_path
            env_plot_module.imageio.get_writer = original_get_writer
            env_plot_module.imageio.imread = original_imread

        assert not buffer_dir.exists()


class TestCircleCenterRender:
    """Circles with a body-frame center offset must render on their collision
    geometry, for both translated and rotated object states."""

    @staticmethod
    def _patch_center_in_data_coords(patch, ax):
        """Map a Circle patch's center to data coords.

        ``Circle.get_transform()`` maps unit-circle space to display space and
        already includes the patch's own center, so the center is the image of
        the unit-space origin.
        """
        display = patch.get_transform().transform((0.0, 0.0))
        return ax.transData.inverted().transform(display)

    def test_circle_center_offset_render_matches_collision(self, circle_center_world):
        env = irsim.make(circle_center_world, display=False)
        env.render()

        ax = env._env_plot.ax
        for obj in env.obstacle_list:
            rendered = self._patch_center_in_data_coords(obj.object_patch, ax)
            collision = np.array(obj.geometry.centroid.coords[0])
            np.testing.assert_allclose(rendered, collision, atol=1e-6)

        # Translated only: state (5, 5, 0) with center [1, 1] -> world (6, 6).
        first = np.array(env.obstacle_list[0].geometry.centroid.coords[0])
        np.testing.assert_allclose(first, [6.0, 6.0], atol=1e-6)

        # Rotated: state (5, 5, 1.57) with center [1, 0] -> (5 + cos, 5 + sin).
        second = np.array(env.obstacle_list[1].geometry.centroid.coords[0])
        expected = [5.0 + np.cos(1.57), 5.0 + np.sin(1.57)]
        np.testing.assert_allclose(second, expected, atol=1e-6)

        env.end()


# ---------------------------------------------------------------------------
# Default `show_arrow` behavior across robots and obstacles.
#
# Regression (#237, v2.9.2): YAML obstacles without a `kinematics:` block were
# unintentionally inheriting `show_arrow=True` because the kinematics factory
# falls back to a DifferentialKinematics handler. The handler-derived default
# applies only when the object is dynamic: `kf is not None` and not static.
# ---------------------------------------------------------------------------


def _write_yaml(path: Path, data: dict) -> Path:
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return path


def _has_arrow(obj) -> bool:
    """Return True if the object actually drew an arrow patch."""
    return getattr(obj, "arrow_patch", None) is not None


@pytest.fixture
def scenario_factory(tmp_path):
    """Build a minimal irsim env from an inline scenario dict."""
    created = []

    def _make(scenario: dict) -> object:
        yaml_path = _write_yaml(tmp_path / "scenario.yaml", scenario)
        env = irsim.make(str(yaml_path), save_ani=False, display=False)
        created.append(env)
        return env

    yield _make

    for env in created:
        with contextlib.suppress(Exception):
            env.end()


BASE_WORLD = {
    "height": 10,
    "width": 10,
    "step_time": 0.1,
    "sample_time": 0.1,
    "offset": [0, 0],
}

ROBOT_DIFF = {
    "kinematics": {"name": "diff"},
    "shape": {"name": "circle", "radius": 0.2},
    "state": [1, 1, 0],
    "goal": [9, 9, 0],
}


def test_robot_diff_has_arrow_by_default(scenario_factory):
    """A diff-drive robot should show an arrow by default."""
    env = scenario_factory(
        {
            "world": BASE_WORLD,
            "robot": [ROBOT_DIFF],
        }
    )
    env.render()
    assert _has_arrow(env.robot), "diff robot should draw arrow by default"


def test_static_obstacle_has_no_arrow(scenario_factory):
    """A static YAML obstacle (no `kinematics:` block) should not draw an
    arrow, even though the kinematics factory secretly attaches a fallback
    DifferentialKinematics handler."""
    env = scenario_factory(
        {
            "world": BASE_WORLD,
            "robot": [ROBOT_DIFF],
            "obstacle": [
                {
                    "shape": {"name": "circle", "radius": 1.0},
                    "state": [5, 5, 0],
                },
                {
                    "shape": {"name": "rectangle", "length": 1.5, "width": 1.2},
                    "state": [6, 5, 1],
                },
            ],
        }
    )
    env.render()
    for obs in env.obstacle_list:
        assert obs.static, f"YAML static obstacle {obs.name} should be flagged static"
        assert not _has_arrow(obs), (
            f"static obstacle {obs.name} should not draw an arrow by default"
        )


def test_dynamic_diff_obstacle_has_arrow_by_default(scenario_factory):
    """A genuinely dynamic obstacle (diff kinematics declared in YAML) should
    draw an arrow by default — it has a real kinematics handler with
    `show_arrow=True` and is not flagged static."""
    env = scenario_factory(
        {
            "world": BASE_WORLD,
            "robot": [ROBOT_DIFF],
            "obstacle": [
                {
                    "kinematics": {"name": "diff"},
                    "shape": {"name": "circle", "radius": 0.3},
                    "state": [3, 3, 0],
                },
            ],
        }
    )
    env.render()
    obs = env.obstacle_list[0]
    assert not obs.static, "diff obstacle should not be flagged static"
    assert _has_arrow(obs), "dynamic diff obstacle should draw arrow by default"


def test_dynamic_omni_obstacle_has_no_arrow_by_default(scenario_factory):
    """Omni kinematics defaults `show_arrow=False` at the handler level, so
    even a dynamic omni obstacle should not draw an arrow."""
    env = scenario_factory(
        {
            "world": BASE_WORLD,
            "robot": [ROBOT_DIFF],
            "obstacle": [
                {
                    "kinematics": {"name": "omni"},
                    "shape": {"name": "circle", "radius": 0.3},
                    "state": [3, 7, 0],
                },
            ],
        }
    )
    env.render()
    obs = env.obstacle_list[0]
    assert not obs.static
    assert not _has_arrow(obs), (
        "dynamic omni obstacle should not draw arrow (handler default False)"
    )


def test_obstacle_arrow_explicit_false(scenario_factory):
    """`plot.show_arrow: False` on a dynamic diff obstacle suppresses arrow."""
    env = scenario_factory(
        {
            "world": BASE_WORLD,
            "robot": [ROBOT_DIFF],
            "obstacle": [
                {
                    "kinematics": {"name": "diff"},
                    "shape": {"name": "circle", "radius": 0.3},
                    "state": [3, 3, 0],
                    "plot": {"show_arrow": False},
                }
            ],
        }
    )
    env.render()
    obs = env.obstacle_list[0]
    assert not _has_arrow(obs), (
        "obstacle with plot.show_arrow=False should not draw arrow"
    )


def test_static_obstacle_arrow_explicit_true(scenario_factory):
    """Users can still opt static obstacles into arrows via `plot.show_arrow`."""
    env = scenario_factory(
        {
            "world": BASE_WORLD,
            "robot": [ROBOT_DIFF],
            "obstacle": [
                {
                    "shape": {"name": "circle", "radius": 0.5},
                    "state": [5, 5, 0],
                    "plot": {"show_arrow": True},
                }
            ],
        }
    )
    env.render()
    obs = env.obstacle_list[0]
    assert _has_arrow(obs), (
        "static obstacle with plot.show_arrow=True should draw arrow"
    )


def test_robot_arrow_explicit_false(scenario_factory):
    """Users can suppress the robot arrow via `plot.show_arrow: False`."""
    robot = dict(ROBOT_DIFF)
    robot["plot"] = {"show_arrow": False}
    env = scenario_factory(
        {
            "world": BASE_WORLD,
            "robot": [robot],
        }
    )
    env.render()
    assert not _has_arrow(env.robot), (
        "robot with plot.show_arrow=False should not draw arrow"
    )


def test_kf_none_robot_has_no_arrow():
    """A robot constructed with `kinematics=None` (i.e. `kf is None`) gets
    `self.static = True` from ObjectBase, so the dynamic-only handler
    default does not apply and no arrow is drawn."""
    obj = ObjectBase(
        kinematics=None,
        shape={"name": "circle", "radius": 0.2},
        state=[1, 1, 0],
        role="robot",
    )
    assert obj.kf is None
    assert obj.static is True

    fig, ax = plt.subplots()
    try:
        obj._init_plot(ax)
        assert not _has_arrow(obj), (
            "robot with kf=None should not draw arrow by default"
        )
    finally:
        plt.close(fig)


class TestObjectText:
    """Custom object and goal text overrides."""

    def test_set_text(self, env_factory):
        """set_text overrides the default abbreviation and can be reset."""
        robot = env_factory("test_all_objects.yaml").robot

        assert robot._get_text() == robot.abbr
        robot.set_text("hello")
        assert robot._get_text() == "hello"
        robot.set_text(None)
        assert robot._get_text() == robot.abbr

    def test_set_text_updates_artist(self, env_factory):
        """set_text pushes the new text to an existing matplotlib artist."""
        robot = env_factory("test_all_objects.yaml").robot
        mock_text = Mock()
        robot._text = mock_text

        robot.set_text("custom")
        mock_text.set_text.assert_called_with("custom")
        robot.set_text(None)
        mock_text.set_text.assert_called_with(robot.abbr)

    def test_set_goal_text(self, env_factory):
        """set_goal_text overrides the goal abbreviation and can be reset."""
        robot = env_factory("test_all_objects.yaml").robot

        assert robot._get_goal_text() == robot.goal_abbr
        robot.set_goal_text("target")
        assert robot._get_goal_text() == "target"
        robot.set_goal_text(None)
        assert robot._get_goal_text() == robot.goal_abbr

    def test_set_goal_text_updates_artist(self, env_factory):
        """set_goal_text pushes the new text to an existing matplotlib artist."""
        robot = env_factory("test_all_objects.yaml").robot
        mock_text = Mock()
        robot._goal_text = mock_text

        robot.set_goal_text("my goal")
        mock_text.set_text.assert_called_with("my goal")
        robot.set_goal_text(None)
        mock_text.set_text.assert_called_with(robot.goal_abbr)


class TestGoalPatchVisibility:
    """The goal patch hides when the goal is cleared."""

    def test_goal_patch_hidden_when_goal_none(self, scenario_factory):
        env = scenario_factory(
            {
                "world": BASE_WORLD,
                "robot": [{**ROBOT_DIFF, "plot": {"show_goal": True}}],
            }
        )
        env.render(0.01)
        robot = env.robot

        goal_patch = robot._object_plot._artist("goal_patch")
        assert goal_patch is not None
        assert goal_patch.get_visible() is True

        robot.set_goal(None)
        env.render(0.01)
        assert goal_patch.get_visible() is False
