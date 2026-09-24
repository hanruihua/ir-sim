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
        cfg["world"]["obstacle_map"] = str(
            Path(path).parent / cfg["world"]["obstacle_map"]
        )
    if mutate is not None:
        mutate(cfg)
    return cfg


def _make(cfg, lidar_batch, tmp_path, tag):
    cfg = copy.deepcopy(cfg)
    cfg.setdefault("world", {})["lidar_batch"] = lidar_batch
    path = tmp_path / f"{tag}.yaml"
    path.write_text(yaml.safe_dump(cfg))
    return irsim.make(
        str(path), display=False, disable_all_plot=True, seed=0, log_level="ERROR"
    )


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
            for a, b in zip(ref.objects, env.objects, strict=True):
                np.testing.assert_allclose(a.state, b.state, atol=atol, rtol=0)
            for a, b in zip(_lidars(ref), _lidars(env), strict=True):
                np.testing.assert_allclose(
                    a.range_data, b.range_data, atol=atol, rtol=0
                )
                np.testing.assert_allclose(
                    a.lidar_origin, b.lidar_origin, atol=atol, rtol=0
                )
                if a.has_velocity:
                    np.testing.assert_allclose(
                        a.velocity, b.velocity, atol=atol, rtol=0
                    )
                np.testing.assert_allclose(
                    a.get_points(), b.get_points(), atol=1e-8, rtol=0
                )
        return ref, env
    finally:
        ref.end(0)
        env.end(0)


SCENES = [
    TESTS
    / "test_collision_avoidance.yaml",  # many circle robots, rvo, moving circle obstacles
    TESTS / "test_all_objects.yaml",  # every shape and kinematics
    TESTS / "test_grid_map.yaml",  # map object
    USAGE
    / "05lidar_world"
    / "lidar_world_laser_color.yaml",  # offset circle, unobstructed, linestrings, sensor offset
    USAGE
    / "12dynamic_obstacle"
    / "dynamic_obstacle.yaml",  # contact mode, moving obstacle
]


def _ensure_lidar(cfg):
    """Give every robot entry a 360-degree lidar when the scene has none."""
    if any(True for _ in _lidar_configs(cfg)):
        return
    for robot in cfg.get("robot", []):
        robot["sensors"] = [
            {
                "name": "lidar2d",
                "range_max": 6,
                "angle_range": 6.28,
                "number": 80,
                "noise": False,
            }
        ]


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("scene", SCENES, ids=lambda p: p.stem)
def test_batched_scan_matches_per_sensor_scan(scene, backend, tmp_path):
    _, env = _assert_same_scans(_load(scene, _ensure_lidar), backend, tmp_path)
    assert env._lidar_batch.backend == backend


@pytest.mark.parametrize("backend", BACKENDS)
def test_hit_velocities_are_tracked(backend, tmp_path):
    def with_velocity(cfg):
        for sensor in _lidar_configs(cfg):
            sensor["has_velocity"] = True

    cfg = _load(TESTS / "test_collision_avoidance.yaml", with_velocity)
    _, env = _assert_same_scans(cfg, backend, tmp_path, steps=15)
    assert any(np.any(lidar.velocity != 0) for lidar in _lidars(env))


@pytest.mark.parametrize("backend", BACKENDS)
def test_polygon_owner_offset_sensor_and_mixed_beam_counts(backend, tmp_path):
    """Rectangle (polygon) robots carrying offset lidars with different beam counts."""
    cfg = {
        "world": {
            "height": 12,
            "width": 12,
            "step_time": 0.1,
            "collision_mode": "unobstructed",
        },
        "robot": [
            {
                "kinematics": {"name": "diff"},
                "shape": {"name": "rectangle", "length": 0.8, "width": 0.4},
                "state": [2, 2, 0.3],
                "goal": [10, 10, 0],
                "behavior": {"name": "dash"},
                "sensors": [
                    {
                        "name": "lidar2d",
                        "range_max": 6,
                        "angle_range": 6.28,
                        "number": 90,
                        "noise": False,
                        "offset": [0.2, -0.1, 0.4],
                    }
                ],
            },
            {
                "kinematics": {"name": "acker"},
                "shape": {"name": "rectangle", "length": 1.0, "width": 0.5},
                "state": [9, 3, 2.0],
                "goal": [2, 9, 0],
                "behavior": {"name": "dash"},
                "sensors": [
                    {
                        "name": "lidar2d",
                        "range_max": 5,
                        "angle_range": 3.14,
                        "number": 64,
                        "noise": False,
                        "offset": [0.3, 0.0, 0.0],
                    }
                ],
            },
            {
                "kinematics": {"name": "omni"},
                "shape": {"name": "circle", "radius": 0.3},
                "state": [6, 9, 0],
                "goal": [6, 2, 0],
                "behavior": {"name": "dash"},
                "sensors": [
                    {
                        "name": "lidar2d",
                        "range_max": 8,
                        "angle_range": 6.28,
                        "number": 128,
                        "noise": False,
                        "offset": [0, 0, 0],
                    }
                ],
            },
        ],
        "obstacle": [
            {
                "shape": {"name": "circle", "radius": 0.8, "center": [0.5, 0.2]},
                "state": [6, 6, 0.7],
            },
            {
                "shape": {
                    "name": "polygon",
                    "vertices": [[3, 7], [4.5, 7.5], [4, 9], [2.5, 8.5]],
                },
                "state": [0, 0, 0],
            },
            {
                "shape": {"name": "rectangle", "length": 2.0, "width": 0.5},
                "state": [8, 8, 0.4],
            },
            {
                "shape": {"name": "linestring", "vertices": [[1, 5], [4, 4], [5, 1]]},
                "state": [0, 0, 0],
            },
            {
                "kinematics": {"name": "omni"},
                "shape": {"name": "rectangle", "length": 0.6, "width": 0.6},
                "state": [4, 3, 0],
                "goal": [8, 5, 0],
                "behavior": {"name": "dash"},
            },
        ],
    }
    _assert_same_scans(cfg, backend, tmp_path, steps=30)


@pytest.mark.parametrize("backend", BACKENDS)
def test_scan_from_inside_a_circle_polygon(backend, tmp_path):
    """A sensor origin inside another circle body hits its edges from within."""
    cfg = {
        "world": {
            "height": 10,
            "width": 10,
            "step_time": 0.1,
            "collision_mode": "unobstructed",
        },
        "robot": [
            {
                "kinematics": {"name": "diff"},
                "shape": {"name": "circle", "radius": 0.2},
                "state": [5.2, 5.1, 0.4],
                "goal": [5.2, 5.1, 0],
                "sensors": [
                    {
                        "name": "lidar2d",
                        "range_max": 4,
                        "angle_range": 6.28,
                        "number": 72,
                        "noise": False,
                    }
                ],
            }
        ],
        "obstacle": [
            {"shape": {"name": "circle", "radius": 1.0}, "state": [5, 5, 0]},
            {"shape": {"name": "circle", "radius": 0.5}, "state": [7, 5, 0]},
        ],
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
        diffs = np.concatenate(
            [
                np.abs(a.range_data - b.range_data)
                for a, b in zip(_lidars(ref), _lidars(env), strict=True)
            ]
        )
        assert np.median(diffs) < 1e-3  # polygon-vs-circle sagitta on most beams
        assert (
            np.mean(diffs > 0.05) < 0.02
        )  # grazing beams may switch between hit and miss
    finally:
        ref.end(0)
        env.end(0)


def test_backend_selection_and_errors():
    caster = LidarBatchCaster(backend="numpy")
    assert caster.backend == "numpy"
    assert caster._kernel is None
    with pytest.raises(ValueError, match="lidar_batch backend"):
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
    cfg = (
        _load(USAGE / "22fmcw_lidar_world" / "fmcw_lidar_world.yaml")
        if (USAGE / "22fmcw_lidar_world" / "fmcw_lidar_world.yaml").exists()
        else None
    )
    if cfg is None:
        pytest.skip("fmcw usage scene not found")
    ref = _make(cfg, False, tmp_path, "ref")
    env = _make(cfg, True, tmp_path, "batch")
    try:
        for _ in range(5):
            ref.step()
            env.step()
        for a, b in zip(_lidars(ref), _lidars(env), strict=True):
            np.testing.assert_allclose(a.range_data, b.range_data, atol=1e-9, rtol=0)
    finally:
        ref.end(0)
        env.end(0)


def _accuracy_scene(sensors=None, obstacles=None):
    return {
        "world": {"collision_mode": "unobstructed"},
        "robot": [
            {
                "kinematics": {"name": "omni"},
                "state": [0, 0, 0],
                "sensors": sensors
                or [
                    {
                        "type": "lidar2d",
                        "number": 1,
                        "range_max": 10,
                        "has_velocity": True,
                    }
                ],
            }
        ],
        "obstacle": obstacles
        or [
            {
                "shape": {"name": "linestring", "vertices": [[3, -2], [3, 2]]},
                "state": [0, 0, 0],
            }
        ],
    }


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("counts", [[9, 9, 9, 9], [9, 7, 11, 13]])
def test_mixed_sensor_noise_keeps_seeded_draw_order(backend, counts, tmp_path):
    cfg = _accuracy_scene(
        [
            {
                "type": kind,
                "number": count,
                "range_max": 10,
                "noise": True,
                "velocity_noise_std": 0.1,
            }
            for kind, count in zip(
                ["fmcw_lidar2d", "lidar2d", "fmcw_lidar2d", "lidar2d"],
                counts,
                strict=True,
            )
        ]
    )
    ref = _make(cfg, False, tmp_path, "noise_ref")
    env = _make(cfg, backend, tmp_path, "noise_batch")
    try:
        for _ in range(5):
            ref.step()
            env.step()
            for a, b in zip(ref.robot.sensors, env.robot.sensors, strict=True):
                np.testing.assert_allclose(
                    a.range_data, b.range_data, atol=1e-9, rtol=0
                )
                if hasattr(a, "valid"):
                    np.testing.assert_array_equal(a.valid, b.valid)
                    np.testing.assert_allclose(
                        a.radial_velocity, b.radial_velocity, atol=1e-9, rtol=0
                    )
            assert (
                ref._env_param.rng.bit_generator.state
                == env._env_param.rng.bit_generator.state
            )
    finally:
        ref.end(0)
        env.end(0)


@pytest.mark.parametrize("backend", BACKENDS)
def test_grazing_circle_beams_keep_reference_hit_decisions(backend, tmp_path):
    """Roundoff at a tangent must not turn a hit into a max-range miss."""
    cfg = _accuracy_scene(
        obstacles=[
            {
                "shape": {"name": "circle", "radius": 1},
                "state": [3, 1, 0],
            }
        ]
    )
    env = _make(cfg, backend, tmp_path, "grazing")
    caster = LidarBatchCaster(backend=backend)
    random = np.random.default_rng(2134)
    try:
        for _ in range(150):
            theta = random.uniform(-np.pi, np.pi)
            distance = random.uniform(1, 9)
            y = 1.0 + random.choice([-1, 0, 1]) * 1e-14
            c, s = np.cos(theta), np.sin(theta)
            env.robot.set_state([0, 0, theta])
            env.obstacle_list[0].set_state(
                [c * distance - s * y, s * distance + c * y, theta]
            )
            env.build_tree()
            sensor = env.robot.lidar
            sensor.step(env.robot.state[:3])
            expected = sensor.range_data.copy()
            caster.step(env.objects)
            np.testing.assert_allclose(sensor.range_data, expected, atol=1e-9, rtol=0)
    finally:
        env.end(0)


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("distance", [3.0, 10.0])
def test_endpoint_and_max_range_hits_keep_target_velocity(backend, distance, tmp_path):
    cfg = _accuracy_scene(
        obstacles=[
            {
                "kinematics": {"name": "omni"},
                "shape": {
                    "name": "linestring",
                    "vertices": [[distance, 0], [distance, 2]],
                },
                "state": [0, 0, 0],
            }
        ]
    )
    env = _make(cfg, backend, tmp_path, "endpoints")
    caster = LidarBatchCaster(backend=backend)
    try:
        for theta in np.linspace(-np.pi, np.pi, 71):
            env.robot.set_state([0, 0, theta])
            env.obstacle_list[0].set_state([0, 0, theta])
            env.obstacle_list[0].set_velocity([0.3, 0.2])
            env.build_tree()
            sensor = env.robot.lidar
            sensor.step(env.robot.state[:3])
            expected = sensor.range_data.copy(), sensor.velocity.copy()
            caster.step(env.objects)
            np.testing.assert_allclose(
                sensor.range_data, expected[0], atol=1e-9, rtol=0
            )
            np.testing.assert_array_equal(sensor.velocity, expected[1])
    finally:
        env.end(0)


@pytest.mark.parametrize("backend", BACKENDS)
def test_coincident_targets_keep_reference_velocity(backend, tmp_path):
    cfg = _accuracy_scene(
        obstacles=[
            {
                "kinematics": {"name": "omni"},
                "shape": {"name": "linestring", "vertices": [[3, -2], [3, 2]]},
                "state": [0, 0, 0],
            }
            for _ in range(2)
        ]
    )
    env = _make(cfg, backend, tmp_path, "ties")
    try:
        for obj, velocity in zip(
            env.obstacle_list, [[0.3, 0.2], [-0.4, 0.1]], strict=True
        ):
            obj.set_velocity(velocity)
        sensor = env.robot.lidar
        sensor.step(env.robot.state[:3])
        expected = sensor.velocity.copy()
        LidarBatchCaster(backend=backend).step(env.objects)
        np.testing.assert_array_equal(sensor.velocity, expected)
    finally:
        env.end(0)


@pytest.mark.parametrize("backend", BACKENDS)
def test_replaced_circle_geometry_preserves_holes_and_parts(backend, tmp_path):
    """A body's shape name cannot override its updated boundary geometry."""
    import shapely

    cfg = _accuracy_scene(
        obstacles=[
            {
                "shape": {"name": "circle", "radius": 2},
                "state": [0, 0, 0],
            }
        ]
    )
    env = _make(cfg, backend, tmp_path, "geometry_changes")
    caster = LidarBatchCaster(backend=backend)
    outer = shapely.Point(0, 0).buffer(2)
    inner = shapely.Point(0, 0).buffer(0.5)
    geometries = [
        outer,
        shapely.Polygon(outer.exterior.coords, [inner.exterior.coords]),
        shapely.MultiPolygon([shapely.box(2, -1, 3, 1), shapely.box(4, -1, 5, 1)]),
        shapely.Polygon(),
        shapely.Point(4, 0).buffer(0.5, quad_segs=4),
    ]
    try:
        for geometry in geometries:
            obstacle = env.obstacle_list[0]
            obstacle.set_original_geometry(geometry)
            obstacle.set_state([0, 0, 0])
            env.build_tree()
            sensor = env.robot.lidar
            sensor.step(env.robot.state[:3])
            expected = sensor.range_data.copy()
            caster.step(env.objects)
            np.testing.assert_allclose(sensor.range_data, expected, atol=1e-9, rtol=0)
    finally:
        env.end(0)


@pytest.mark.parametrize("backend", BACKENDS)
def test_randomized_batch_scans_match_independent_geos_intersections(backend, tmp_path):
    """Compare full scans with GEOS, independent of either numerical kernel."""
    import shapely

    random = np.random.default_rng(91234)
    obstacles = []
    for i in range(12):
        shape = [
            {"name": "circle", "radius": 0.4, "center": [0.1, -0.2]},
            {"name": "rectangle", "length": 0.7, "width": 0.3},
            {"name": "linestring", "vertices": [[-0.4, -0.3], [0.3, 0.4], [0.5, -0.3]]},
        ][i % 3]
        obstacles.append({"shape": shape, "state": [3 + i % 4, -3 + i // 4, 0]})
    cfg = _accuracy_scene(
        [
            {
                "type": "lidar2d",
                "number": 361,
                "range_max": 12,
                "angle_range": 2 * np.pi,
                "offset": [0.2, -0.1, 0.31],
            },
        ],
        obstacles,
    )
    env = _make(cfg, backend, tmp_path, "geos")
    caster = LidarBatchCaster(backend=backend)
    try:
        for _ in range(20):
            env.robot.set_state(
                [
                    random.uniform(-2, 0),
                    random.uniform(-2, 2),
                    random.uniform(-np.pi, np.pi),
                ]
            )
            for obstacle in env.obstacle_list:
                obstacle.set_state(
                    [
                        random.uniform(2, 8),
                        random.uniform(-4, 4),
                        random.uniform(-np.pi, np.pi),
                    ]
                )
            env.build_tree()
            sensor = env.robot.lidar
            world_beams = sensor._world_geometry(env.robot.state[:3])
            boundaries = shapely.union_all(
                [
                    obj.geometry if obj.shape == "linestring" else obj.geometry.boundary
                    for obj in env.obstacle_list
                ]
            )
            intersections = shapely.intersection(
                shapely.get_parts(world_beams), boundaries
            )
            distances = shapely.distance(
                shapely.Point(sensor.lidar_origin[:2, 0]), intersections
            )
            expected = np.where(np.isnan(distances), sensor.range_max, distances)
            caster.step(env.objects)
            np.testing.assert_allclose(sensor.range_data, expected, atol=1e-9, rtol=0)
    finally:
        env.end(0)


def test_default_batch_is_exact_and_auto_falls_back_without_numba(
    monkeypatch, tmp_path
):
    import irsim.lib.algorithm.lidar_batch_numba as compiled

    monkeypatch.setattr(compiled, "AVAILABLE", False)
    cfg = _load(TESTS / "test_collision_avoidance.yaml")
    _, env = _assert_same_scans(cfg, True, tmp_path, steps=3)
    assert env._lidar_batch.backend == "numpy"
    assert not env._lidar_batch.analytic_circles


@pytest.mark.parametrize("backend", BACKENDS)
def test_switching_from_analytic_to_exact_rebuilds_caster(backend, tmp_path):
    cfg = _accuracy_scene(
        obstacles=[
            {
                "shape": {"name": "circle", "radius": 1},
                "state": [3, 0.4, 0],
            }
        ]
    )
    env = _make(cfg, "analytic", tmp_path, "mode_change")
    try:
        env.step()
        approximate = env.robot.lidar.range_data.copy()
        env._world_param.lidar_batch = backend
        env.step()
        exact = env.robot.lidar.range_data.copy()
        assert not env._lidar_batch.analytic_circles
        assert abs(approximate[0] - exact[0]) > 1e-5
        env.robot.lidar.step(env.robot.state[:3])
        np.testing.assert_allclose(exact, env.robot.lidar.range_data, atol=1e-9, rtol=0)
    finally:
        env.end(0)
