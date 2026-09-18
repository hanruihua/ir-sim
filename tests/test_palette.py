"""
Tests for ``irsim.config.palette_param``: the runtime color defaults, how the
library reads them when objects and plots are created, and ``color: 'cycle'``.
"""

import numpy as np
import pytest
from matplotlib.colors import to_rgba

from irsim.config import palette_param
from irsim.config.palette_param import DEFAULT_CYCLE, PaletteParam, bind
from irsim.lib.handler.kinematics_handler import (
    DifferentialKinematics,
    _kinematics_registry,
    register_kinematics,
)
from irsim.world.object_base import ObjectBase
from irsim.world.object_plot import ArrowStyle, FovStyle


@pytest.fixture
def fresh_palette():
    """Bind a fresh palette for the test and restore the previous one after."""
    saved = palette_param[0]
    bind(PaletteParam())
    yield palette_param
    bind(saved)


def _yaml(tmp_path, text):
    path = tmp_path / "palette_world.yaml"
    path.write_text(text)
    return str(path)


ROBOT_WORLD = """
world: {height: 10, width: 10}
robot:
  - kinematics: {name: 'diff'}
    shape: {name: 'circle', radius: 0.2}
    state: [1, 1, 0]
    sensors:
      - name: 'lidar2d'
        number: 20
        range_max: 5
  - kinematics: {name: 'acker'}
    shape: {name: 'rectangle', length: 1.0, width: 0.5, wheelbase: 0.8}
    state: [5, 5, 0, 0]
obstacle:
  - shape: {name: 'circle', radius: 0.3}
    state: [8, 8, 0]
"""


class TestPaletteParam:
    def test_defaults(self):
        assert palette_param.robot == "#009E73"
        assert palette_param.obstacle == "k"
        assert palette_param.cycle == list(DEFAULT_CYCLE)
        assert palette_param.cycle_color(0) == palette_param.robot
        assert palette_param.cycle_color(len(DEFAULT_CYCLE)) == DEFAULT_CYCLE[0]

    def test_module_proxies_the_bound_instance(self, fresh_palette):
        palette_param.robot = "navy"
        assert palette_param[0].robot == "navy"
        assert palette_param.robot == "navy"

        other = PaletteParam(robot="teal")
        palette_param[2] = other
        assert palette_param[2] is other
        assert palette_param.robot == "navy"  # index 0 stays current
        palette_param[0] = other
        assert palette_param.robot == "teal"
        with pytest.raises(IndexError):
            palette_param[-1] = other

    def test_bind_fills_an_empty_registry(self, fresh_palette):
        import irsim.config.palette_param as module

        saved = list(module._instances)
        module._instances.clear()
        try:
            bind(PaletteParam(robot="teal"))
            assert module._instances[0].robot == "teal"
            assert palette_param.robot == "teal"
        finally:
            module._instances[:] = saved
            bind(saved[0])

    def test_non_field_attributes_stay_on_the_module(self, fresh_palette):
        palette_param.scratch_note = "kept on the module"
        try:
            assert palette_param.scratch_note == "kept on the module"
            assert not hasattr(palette_param[0], "scratch_note")
        finally:
            delattr(palette_param, "scratch_note")

    def test_fresh_fixture_restores(self, fresh_palette):
        palette_param.arrow = "red"
        assert ArrowStyle().color == "red"

    def test_defaults_are_restored_between_tests(self):
        assert palette_param.arrow == "#F0E442"


class TestPaletteConsumers:
    def test_objects_take_the_palette_at_creation(
        self, fresh_palette, env_factory, tmp_path
    ):
        palette_param.robot = "navy"
        palette_param.robot_acker = "olive"
        palette_param.obstacle = "gray"
        palette_param.arrow = "red"
        palette_param.fov = "pink"
        palette_param.fov_edge = "purple"
        palette_param.lidar = "blue"

        env = env_factory(_yaml(tmp_path, ROBOT_WORLD))
        diff, acker = env.robot_list
        assert diff.color == "navy"
        assert acker.color == "olive"
        assert env.obstacle_list[0].color == "gray"
        assert diff.lidar.color == "blue"
        assert diff._object_plot.options.arrow.color == "red"
        assert diff._object_plot.options.fov.color == "pink"
        assert diff._object_plot.options.fov.edge_color == "purple"
        assert FovStyle().edge_color == "purple"

    def test_explicit_colors_still_win(self, fresh_palette):
        palette_param.obstacle = "gray"
        assert ObjectBase(color="red").color == "red"
        assert ObjectBase().color == "gray"

    def test_handler_class_color_overrides_the_palette(self, fresh_palette):
        @register_kinematics("palette_test_diff")
        class YellowDiff(DifferentialKinematics):
            color = "y"

        try:
            assert YellowDiff.default_color() == "y"
            assert YellowDiff.default_color("obstacle") == palette_param.obstacle
            palette_param.robot = "navy"
            assert YellowDiff.default_color() == "y"
            assert DifferentialKinematics.default_color() == "navy"
        finally:
            _kinematics_registry.pop("palette_test_diff", None)

    def test_handler_obstacle_color_overrides_the_palette(self, fresh_palette):
        @register_kinematics("palette_test_gray_obstacle")
        class GrayObstacleDiff(DifferentialKinematics):
            obstacle_color = "gray"

        try:
            palette_param.obstacle = "k"
            assert GrayObstacleDiff.default_color("obstacle") == "gray"
            assert GrayObstacleDiff.default_color() == palette_param.robot
        finally:
            _kinematics_registry.pop("palette_test_gray_obstacle", None)

    def test_3d_points_use_the_marker_color(self, fresh_palette, env_factory, tmp_path):
        from unittest.mock import patch

        palette_param.marker = "orange"
        env = env_factory(_yaml(tmp_path, ROBOT_WORLD), projection="3d")
        ax = env._env_plot.ax
        with patch.object(ax, "scatter", wraps=ax.scatter) as scatter:
            env.draw_points([[1.0, 1.0, 1.0]])
        assert scatter.call_args.args[5] == "orange"

    def test_draw_helpers_and_laser_highlight(
        self, fresh_palette, env_factory, tmp_path
    ):
        palette_param.marker = "orange"
        palette_param.path = "brown"
        palette_param.laser_highlight = "magenta"
        env = env_factory(_yaml(tmp_path, ROBOT_WORLD))
        env.render(0.001)
        ax = env._env_plot.ax

        env.draw_points([[2.0, 2.0]])
        assert np.allclose(ax.collections[-1].get_facecolor()[0], to_rgba("orange"))
        env.draw_trajectory(
            [np.array([[0.0], [0.0], [0.0]]), np.array([[1.0], [1.0], [0.0]])]
        )
        assert to_rgba(ax.lines[-1].get_color()) == to_rgba("brown")

        env.robot.set_laser_color([0, 1])
        colors = env.robot.lidar.laser_LineCollection.get_colors()
        assert np.allclose(colors[0][:3], to_rgba("magenta")[:3])


class TestColorCycle:
    def test_cycle_assigns_palette_colors_in_order(self, env_factory, tmp_path):
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 20, width: 20}\n"
                "robot:\n"
                "  - number: 10\n"
                "    distribution: {name: 'circle', radius: 5, center: [10, 10]}\n"
                "    kinematics: {name: 'diff'}\n"
                "    shape: {name: 'circle', radius: 0.2}\n"
                "    color: 'cycle'\n"
                "obstacle:\n"
                "  - number: 3\n"
                "    distribution: {name: 'manual'}\n"
                "    shape: {name: 'circle', radius: 0.2}\n"
                "    state: [[1, 1, 0], [2, 2, 0], [3, 3, 0]]\n"
                "    color: 'cycle'\n",
            )
        )
        robots = [r.color for r in env.robot_list]
        assert robots == [palette_param.cycle_color(i) for i in range(10)]
        assert robots[8] == robots[0]
        assert [o.color for o in env.obstacle_list] == list(DEFAULT_CYCLE[:3])

    def test_cycle_uses_the_bound_cycle(self, fresh_palette, env_factory, tmp_path):
        palette_param.cycle = ["red", "blue"]
        env = env_factory(
            _yaml(
                tmp_path,
                "world: {height: 10, width: 10}\n"
                "robot:\n"
                "  - number: 3\n"
                "    distribution: {name: 'manual'}\n"
                "    kinematics: {name: 'diff'}\n"
                "    shape: {name: 'circle', radius: 0.2}\n"
                "    state: [[1, 1, 0], [3, 3, 0], [5, 5, 0]]\n"
                "    color: 'cycle'\n",
            )
        )
        assert [r.color for r in env.robot_list] == ["red", "blue", "red"]
