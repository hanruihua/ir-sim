"""Tests for the default `show_arrow` behavior across robots and obstacles.

Regression: after the kinematics-handler registry refactor (#237, v2.9.2),
YAML obstacles without a `kinematics:` block were unintentionally inheriting
`show_arrow=True` — `KinematicsFactory.create_kinematics` falls back to a
`DifferentialKinematics` instance even when no name is given, so the old
`self.kf is not None` guard couldn't tell them apart from real dynamic
obstacles.

Rule the fix codifies: apply the handler-derived `show_arrow` default only
when the object is *dynamic* — `self.kf is not None` AND `self.static is
False`. `self.static` is True for ObjectStatic (used for any YAML object
without kinematics, including kf=None robots), so this naturally suppresses
arrows for purely static things while preserving them for diff/acker
dynamic obstacles.
"""

import contextlib
from pathlib import Path

import matplotlib.pyplot as plt
import pytest
import yaml

import irsim
from irsim.world.object_base import ObjectBase


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
