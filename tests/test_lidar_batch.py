"""The batched lidar caster reproduces the per-sensor scan on whole scenes."""

import copy
import importlib.util
from pathlib import Path

import numpy as np
import pytest
import yaml

import irsim
from irsim.lib.algorithm.lidar_batch import LidarBatchCaster

TESTS = Path(__file__).resolve().parent
USAGE = TESTS.parent / "usage"
HAS_NUMBA = importlib.util.find_spec("numba") is not None

BACKENDS = ["numpy"] + (["numba"] if HAS_NUMBA else [])


def _lidar_configs(node):
    """Yield every lidar2d sensor mapping of a YAML tree."""
    if isinstance(node, dict):
        if node.get("name") == "lidar2d" or node.get("type") == "lidar2d":
            yield node
        for value in node.values():
            yield from _lidar_configs(value)
    elif isinstance(node, list):
        for value in node:
            yield from _lidar_configs(value)


def _load(path, mutate=None):
    cfg = yaml.safe_load(Path(path).read_text())
    for sensor in _lidar_configs(cfg):
        sensor["noise"] = False
    if cfg.get("world", {}).get("obstacle_map"):
        cfg["world"]["obstacle_map"] = str(Path(path).parent / cfg["world"]["obstacle_map"])
    if mutate is not None:
        mutate(cfg)
    return cfg


def _make(cfg, lidar_batch, tmp_path, tag):
    cfg = copy.deepcopy(cfg)
    cfg.setdefault("world", {})["lidar_batch"] = lidar_batch
    path = tmp_path / f"{tag}.yaml"
    path.write_text(yaml.safe_dump(cfg))
    return irsim.make(str(path), display=False, disable_all_plot=True, seed=0, log_level="ERROR")


def _lidars(env):
    return [obj.lidar for obj in env.objects if getattr(obj, "lidar", None) is not None]


def _assert_same_scans(cfg, lidar_batch, tmp_path, steps=25, atol=1e-9):
    ref = _make(cfg, False, tmp_path, "ref")
    env = _make(cfg, lidar_batch, tmp_path, f"batch_{lidar_batch}")
    try:
        assert _lidars(env), "scene has no lidar to compare"
        for _ in range(steps):
            ref.step()
            env.step()
            for a, b in zip(ref.objects, env.objects):
                np.testing.assert_allclose(a.state, b.state, atol=atol)
            for a, b in zip(_lidars(ref), _lidars(env)):
                np.testing.assert_allclose(a.range_data, b.range_data, atol=atol, rtol=0)
                np.testing.assert_allclose(a.lidar_origin, b.lidar_origin, atol=atol, rtol=0)
                if a.has_velocity:
                    np.testing.assert_allclose(a.velocity, b.velocity, atol=atol, rtol=0)
                np.testing.assert_allclose(a.get_points(), b.get_points(), atol=1e-8, rtol=0)
        return ref, env
    finally:
        ref.end(0)
        env.end(0)


SCENES = [
    TESTS / "test_collision_avoidance.yaml",          # many circle robots, rvo, moving circle obstacles
    TESTS / "test_all_objects.yaml",                  # every shape and kinematics
    TESTS / "test_grid_map.yaml",                     # map object
    USAGE / "05lidar_world" / "lidar_world_laser_color.yaml",  # offset circle, unobstructed, linestrings, sensor offset
    USAGE / "12dynamic_obstacle" / "dynamic_obstacle.yaml",    # contact mode, moving obstacle
]


def _ensure_lidar(cfg):
    """Give every robot entry a 360-degree lidar when the scene has none."""
    if any(True for _ in _lidar_configs(cfg)):
        return
    for robot in cfg.get("robot", []):
        robot["sensors"] = [{"name": "lidar2d", "range_max": 6, "angle_range": 6.28, "number": 80, "noise": False}]


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("scene", SCENES, ids=lambda p: p.stem)
def test_batched_scan_matches_per_sensor_scan(scene, backend, tmp_path):
    ref, env = _assert_same_scans(_load(scene, _ensure_lidar), backend, tmp_path)
    assert env._lidar_batch.backend == backend


@pytest.mark.parametrize("backend", BACKENDS)
def test_hit_velocities_are_tracked(backend, tmp_path):
    def with_velocity(cfg):
        for sensor in _lidar_configs(cfg):
            sensor["has_velocity"] = True

    cfg = _load(TESTS / "test_collision_avoidance.yaml", with_velocity)
    ref, env = _assert_same_scans(cfg, backend, tmp_path, steps=15)
    assert any(np.any(lidar.velocity != 0) for lidar in _lidars(env))


@pytest.mark.parametrize("backend", BACKENDS)
def test_polygon_owner_offset_sensor_and_mixed_beam_counts(backend, tmp_path):
    """Rectangle (polygon) robots carrying offset lidars with different beam counts."""
    cfg = {
        "world": {"height": 12, "width": 12, "step_time": 0.1, "collision_mode": "unobstructed"},
        "robot": [
            {"kinematics": {"name": "diff"}, "shape": {"name": "rectangle", "length": 0.8, "width": 0.4},
             "state": [2, 2, 0.3], "goal": [10, 10, 0], "behavior": {"name": "dash"},
             "sensors": [{"name": "lidar2d", "range_max": 6, "angle_range": 6.28, "number": 90,
                          "noise": False, "offset": [0.2, -0.1, 0.4]}]},
            {"kinematics": {"name": "acker"}, "shape": {"name": "rectangle", "length": 1.0, "width": 0.5},
             "state": [9, 3, 2.0], "goal": [2, 9, 0], "behavior": {"name": "dash"},
             "sensors": [{"name": "lidar2d", "range_max": 5, "angle_range": 3.14, "number": 64,
                          "noise": False, "offset": [0.3, 0.0, 0.0]}]},
            {"kinematics": {"name": "omni"}, "shape": {"name": "circle", "radius": 0.3},
             "state": [6, 9, 0], "goal": [6, 2, 0], "behavior": {"name": "dash"},
             "sensors": [{"name": "lidar2d", "range_max": 8, "angle_range": 6.28, "number": 128,
                          "noise": False, "offset": [0, 0, 0]}]},
        ],
        "obstacle": [
            {"shape": {"name": "circle", "radius": 0.8, "center": [0.5, 0.2]}, "state": [6, 6, 0.7]},
            {"shape": {"name": "polygon", "vertices": [[3, 7], [4.5, 7.5], [4, 9], [2.5, 8.5]]}, "state": [0, 0, 0]},
            {"shape": {"name": "rectangle", "length": 2.0, "width": 0.5}, "state": [8, 8, 0.4]},
            {"shape": {"name": "linestring", "vertices": [[1, 5], [4, 4], [5, 1]]}, "state": [0, 0, 0]},
            {"kinematics": {"name": "omni"}, "shape": {"name": "rectangle", "length": 0.6, "width": 0.6},
             "state": [4, 3, 0], "goal": [8, 5, 0], "behavior": {"name": "dash"}},
        ],
    }
    _assert_same_scans(cfg, backend, tmp_path, steps=30)


@pytest.mark.parametrize("backend", BACKENDS)
def test_scan_from_inside_a_circle_polygon(backend, tmp_path):
    """A sensor origin inside another circle body hits its edges from within."""
    cfg = {
        "world": {"height": 10, "width": 10, "step_time": 0.1, "collision_mode": "unobstructed"},
        "robot": [{"kinematics": {"name": "diff"}, "shape": {"name": "circle", "radius": 0.2},
                   "state": [5.2, 5.1, 0.4], "goal": [5.2, 5.1, 0],
                   "sensors": [{"name": "lidar2d", "range_max": 4, "angle_range": 6.28, "number": 72,
                                "noise": False}]}],
        "obstacle": [{"shape": {"name": "circle", "radius": 1.0}, "state": [5, 5, 0]},
                     {"shape": {"name": "circle", "radius": 0.5}, "state": [7, 5, 0]}],
    }
    _assert_same_scans(cfg, backend, tmp_path, steps=3)


def test_analytic_circles_are_close_to_the_polygon_scan(tmp_path):
    cfg = _load(TESTS / "test_collision_avoidance.yaml")
    ref = _make(cfg, False, tmp_path, "ref")
    env = _make(cfg, "analytic", tmp_path, "analytic")
    try:
        for _ in range(10):
            ref.step()
            env.step()
        assert env._lidar_batch.analytic_circles
        diffs = np.concatenate([np.abs(a.range_data - b.range_data) for a, b in zip(_lidars(ref), _lidars(env))])
        assert np.median(diffs) < 1e-3          # polygon-vs-circle sagitta on most beams
        assert np.mean(diffs > 0.05) < 0.02     # grazing beams may switch between hit and miss
    finally:
        ref.end(0)
        env.end(0)


def test_backend_selection_and_errors():
    caster = LidarBatchCaster(backend="numpy")
    assert caster.backend == "numpy" and caster._kernel is None
    with pytest.raises(ValueError):
        LidarBatchCaster(backend="cuda")
    if HAS_NUMBA:
        assert LidarBatchCaster().backend == "numba"
        assert LidarBatchCaster(backend="numba").backend == "numba"
    else:
        assert LidarBatchCaster().backend == "numpy"
        with pytest.raises(ImportError):
            LidarBatchCaster(backend="numba")


def test_other_sensor_types_keep_their_own_step(tmp_path):
    """An FMCW lidar is not a plain Lidar2D and must be stepped by itself."""
    cfg = _load(USAGE / "22fmcw_lidar_world" / "fmcw_lidar_world.yaml") if (
        USAGE / "22fmcw_lidar_world" / "fmcw_lidar_world.yaml").exists() else None
    if cfg is None:
        pytest.skip("fmcw usage scene not found")
    ref = _make(cfg, False, tmp_path, "ref")
    env = _make(cfg, True, tmp_path, "batch")
    try:
        for _ in range(5):
            ref.step()
            env.step()
        for a, b in zip(_lidars(ref), _lidars(env)):
            np.testing.assert_allclose(a.range_data, b.range_data, atol=1e-9, rtol=0)
    finally:
        ref.end(0)
        env.end(0)
