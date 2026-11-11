"""The batched lidar caster reproduces the per-sensor scan on whole scenes."""

from pathlib import Path

import numpy as np
import pytest
import yaml

import irsim
import irsim.lib.algorithm.ray_cast2d_batch as ray_cast2d_batch
from irsim.lib.algorithm.ray_cast2d_batch import RayCast2DBatch
from irsim.world.sensors.lidar2d import Lidar2D

TESTS = Path(__file__).resolve().parent
USAGE = TESTS.parent / "usage"

BACKENDS = ["numpy"] + (["numba"] if ray_cast2d_batch.NUMBA_AVAILABLE else [])


@pytest.fixture(params=BACKENDS)
def backend(request, monkeypatch):
    """Cast with the NumPy kernel, and with the compiled one when numba is
    installed, through a fresh process caster."""
    if request.param == "numpy":
        monkeypatch.setattr(ray_cast2d_batch, "NUMBA_AVAILABLE", False)
    monkeypatch.setattr(ray_cast2d_batch, "_caster", RayCast2DBatch())
    return request.param


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


def _make(cfg, tmp_path, tag):
    path = tmp_path / f"{tag}.yaml"
    path.write_text(yaml.safe_dump(cfg))
    return irsim.make(
        str(path), display=False, disable_all_plot=True, seed=0, log_level="ERROR"
    )


def _step_alone(env):
    """Step every sensor of ``env`` without the batch: each lidar casts alone."""
    for obj in env.objects:
        for sensor in obj.sensors:
            if type(sensor) is Lidar2D:
                sensor._cast_alone(obj.state[0:3])
            else:
                sensor.step(obj.state[0:3])


def _make_reference(cfg, tmp_path):
    """An env whose lidars all cast alone (the per-sensor scan)."""
    env = _make(cfg, tmp_path, "ref")
    env._objects_sensor_step = lambda: _step_alone(env)
    return env


def _batch_step(env):
    """Step every sensor of ``env`` as the environment does, in a fresh scene."""
    env.build_tree()
    for obj in env.objects:
        obj.sensor_step()


def _lidars(env):
    return [obj.lidar for obj in env.objects if obj.lidar is not None]


def _assert_same_scans(cfg, tmp_path, steps=25):
    """Batched and per-sensor scans of a scene agree bit for bit, step by step."""
    ref = _make_reference(cfg, tmp_path)
    env = _make(cfg, tmp_path, "batch")
    try:
        assert _lidars(env), "scene has no lidar to compare"
        for _ in range(steps):
            ref.step()
            env.step()
            for a, b in zip(ref.objects, env.objects, strict=True):
                np.testing.assert_array_equal(a.state, b.state)
            for a, b in zip(_lidars(ref), _lidars(env), strict=True):
                np.testing.assert_array_equal(a.state, b.state)
                np.testing.assert_array_equal(a.valid, b.valid)
                np.testing.assert_array_equal(a.range_data, b.range_data)
                np.testing.assert_array_equal(a.lidar_origin, b.lidar_origin)
                np.testing.assert_array_equal(a.velocity, b.velocity)
                np.testing.assert_array_equal(a.get_points(), b.get_points())
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


@pytest.mark.parametrize("scene", SCENES, ids=lambda p: p.stem)
def test_batched_scan_matches_per_sensor_scan(scene, backend, tmp_path):
    _assert_same_scans(_load(scene, _ensure_lidar), tmp_path)
    assert ray_cast2d_batch._caster.backend == backend


def test_hit_velocities_are_tracked(backend, tmp_path):
    def with_velocity(cfg):
        for sensor in _lidar_configs(cfg):
            sensor["has_velocity"] = True

    cfg = _load(TESTS / "test_collision_avoidance.yaml", with_velocity)
    _, env = _assert_same_scans(cfg, tmp_path, steps=15)
    assert any(np.any(lidar.velocity != 0) for lidar in _lidars(env))


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
    _assert_same_scans(cfg, tmp_path, steps=30)


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
    _assert_same_scans(cfg, tmp_path, steps=3)


def test_kernel_is_compiled_only_when_numba_is_installed(monkeypatch):
    assert RayCast2DBatch().backend == (
        "numba" if ray_cast2d_batch.NUMBA_AVAILABLE else "numpy"
    )
    monkeypatch.setattr(ray_cast2d_batch, "NUMBA_AVAILABLE", False)
    caster = RayCast2DBatch()
    assert caster.backend == "numpy"
    assert caster._kernel is None


def test_a_kernel_that_fails_to_compile_falls_back_to_numpy(monkeypatch, tmp_path):
    """A numba install that cannot compile the kernel warns and scans with NumPy."""

    def broken_kernel(*args):
        raise RuntimeError("numba could not compile the kernel")

    monkeypatch.setattr(ray_cast2d_batch, "NUMBA_AVAILABLE", True)
    monkeypatch.setattr(ray_cast2d_batch, "cast_kernel", broken_kernel)
    monkeypatch.setattr(ray_cast2d_batch, "_caster", RayCast2DBatch())
    with pytest.warns(RuntimeWarning, match="NumPy"):
        env = _make(_accuracy_scene(), tmp_path, "broken_kernel")
    try:
        sensor = env.robot.lidar
        batched = sensor.range_data.copy()
        sensor._cast_alone(env.robot.state[:3])
        assert ray_cast2d_batch._caster.backend == "numpy"
        np.testing.assert_allclose(batched, [3.0], atol=1e-12)
        np.testing.assert_array_equal(batched, sensor.range_data)
    finally:
        env.end(0)


def test_other_sensor_types_keep_their_own_step(tmp_path):
    """An FMCW lidar is not a plain Lidar2D and must be stepped by itself."""
    cfg = (
        _load(USAGE / "22fmcw_lidar_world" / "fmcw_lidar_world.yaml")
        if (USAGE / "22fmcw_lidar_world" / "fmcw_lidar_world.yaml").exists()
        else None
    )
    if cfg is None:
        pytest.skip("fmcw usage scene not found")
    ref = _make_reference(cfg, tmp_path)
    env = _make(cfg, tmp_path, "batch")
    try:
        for _ in range(5):
            ref.step()
            env.step()
        for a, b in zip(_lidars(ref), _lidars(env), strict=True):
            np.testing.assert_array_equal(a.range_data, b.range_data)
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


@pytest.mark.parametrize("counts", [[9, 9, 9, 9], [9, 7, 11, 13]])
@pytest.mark.parametrize("angle_std", [0.0, 0.05])
def test_mixed_sensor_noise_keeps_seeded_draw_order(
    backend, counts, angle_std, tmp_path
):
    cfg = _accuracy_scene(
        [
            {
                "type": kind,
                "number": count,
                "range_max": 10,
                "noise": True,
                "angle_std": angle_std,
                "velocity_noise_std": 0.1,
            }
            for kind, count in zip(
                ["fmcw_lidar2d", "lidar2d", "fmcw_lidar2d", "lidar2d"],
                counts,
                strict=True,
            )
        ]
    )
    ref = _make_reference(cfg, tmp_path)
    env = _make(cfg, tmp_path, "noise_batch")
    try:
        for _ in range(5):
            ref.step()
            env.step()
            for a, b in zip(ref.robot.sensors, env.robot.sensors, strict=True):
                np.testing.assert_array_equal(a.range_data, b.range_data)
                np.testing.assert_array_equal(a.valid, b.valid)
                np.testing.assert_array_equal(a.cast_angles, b.cast_angles)
                if hasattr(a, "radial_velocity"):
                    np.testing.assert_array_equal(a.radial_velocity, b.radial_velocity)
            assert (
                ref._env_param.rng.bit_generator.state
                == env._env_param.rng.bit_generator.state
            )
    finally:
        ref.end(0)
        env.end(0)


@pytest.mark.parametrize("distance", [0.25, 3.0, 10.0, 11.0])
@pytest.mark.parametrize("std", [0.0, 3.0])
def test_batch_preserves_validity_range_band_and_noise(
    backend, distance, std, tmp_path
):
    """Batch scans retain blind-zone, max-range, miss and noisy-hit semantics."""
    cfg = _accuracy_scene(
        sensors=[
            {
                "type": "lidar2d",
                "number": 1,
                "range_min": 1.0,
                "range_max": 10.0,
                "noise": True,
                "std": std,
            }
        ],
        obstacles=[
            {
                "shape": {
                    "name": "linestring",
                    "vertices": [[distance, -2], [distance, 2]],
                },
                "state": [0, 0, 0],
            }
        ],
    )
    _, env = _assert_same_scans(cfg, tmp_path, steps=5)
    scan = env.robot.lidar
    assert 1.0 <= scan.range_data[0] <= 10.0
    if std == 0:
        assert scan.valid[0] == (1.0 <= distance <= 10.0)
        assert scan.range_data[0] == pytest.approx(np.clip(distance, 1.0, 10.0))


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
    env = _make(cfg, tmp_path, "grazing")
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
            sensor._cast_alone(env.robot.state[:3])
            expected = sensor.range_data.copy()
            _batch_step(env)
            np.testing.assert_array_equal(sensor.range_data, expected)
    finally:
        env.end(0)


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
    env = _make(cfg, tmp_path, "endpoints")
    try:
        for theta in np.linspace(-np.pi, np.pi, 71):
            env.robot.set_state([0, 0, theta])
            env.obstacle_list[0].set_state([0, 0, theta])
            env.obstacle_list[0].set_velocity([0.3, 0.2])
            env.build_tree()
            sensor = env.robot.lidar
            sensor._cast_alone(env.robot.state[:3])
            expected = sensor.range_data.copy(), sensor.velocity.copy()
            _batch_step(env)
            np.testing.assert_array_equal(sensor.range_data, expected[0])
            np.testing.assert_array_equal(sensor.velocity, expected[1])
    finally:
        env.end(0)


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
    env = _make(cfg, tmp_path, "ties")
    try:
        for obj, velocity in zip(
            env.obstacle_list, [[0.3, 0.2], [-0.4, 0.1]], strict=True
        ):
            obj.set_velocity(velocity)
        sensor = env.robot.lidar
        sensor._cast_alone(env.robot.state[:3])
        expected = sensor.velocity.copy()
        _batch_step(env)
        np.testing.assert_array_equal(sensor.velocity, expected)
    finally:
        env.end(0)


def test_mixed_resolution_circles_are_cast_by_the_batch(backend, tmp_path, monkeypatch):
    """Circle bodies with different polygon resolutions are cast exactly in batch.

    The beams hit edges away from vertices, so no scan is ambiguous and the
    compared ranges come from the batched cast, not the reference fallback.
    """
    import shapely

    sensors = [
        {
            "type": "lidar2d",
            "number": 31,
            "range_max": 10,
            "angle_range": 1.6,
            "has_velocity": True,
        }
    ]
    cfg = _accuracy_scene(
        sensors=sensors,
        obstacles=[
            {
                "shape": {"name": "circle", "radius": 1},
                "state": [4, 0.37, 0],
                "kinematics": {"name": "omni"},
            },
            {"shape": {"name": "circle", "radius": 0.6}, "state": [6, -1.9, 0]},
            {  # a wall behind the circles keeps segments in the same cast
                "shape": {"name": "linestring", "vertices": [[8.3, -6], [8.3, 6]]},
                "state": [0, 0, 0],
            },
        ],
    )
    env = _make(cfg, tmp_path, "mixed_resolution")
    try:
        coarse = env.obstacle_list[0]
        coarse.set_original_geometry(shapely.Point(0, 0).buffer(1, quad_segs=4))
        coarse.set_state([4, 0.37, 0])
        coarse._velocity = np.array([[0.5], [-0.25]])
        coarse._velocity_xy_cache = None
        env.build_tree()
        sensor = env.robot.lidar
        sensor._cast_alone(env.robot.state[:3])
        expected = sensor.range_data.copy()
        expected_velocity = sensor.velocity.copy()
        assert (expected < sensor.range_max).sum() >= 10, "beams must hit both bodies"
        assert np.any(expected_velocity != 0), "the coarse circle must be seen moving"

        fallbacks = []
        reference_step = Lidar2D._cast_alone
        monkeypatch.setattr(
            Lidar2D,
            "_cast_alone",
            lambda self, state: fallbacks.append(self) or reference_step(self, state),
        )
        _batch_step(env)

        assert fallbacks == []
        np.testing.assert_array_equal(sensor.range_data, expected)
        np.testing.assert_array_equal(sensor.velocity, expected_velocity)
    finally:
        env.end(0)


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
    env = _make(cfg, tmp_path, "geometry_changes")
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
            sensor._cast_alone(env.robot.state[:3])
            expected = sensor.range_data.copy()
            _batch_step(env)
            np.testing.assert_array_equal(sensor.range_data, expected)
    finally:
        env.end(0)


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
    env = _make(cfg, tmp_path, "geos")
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
            _batch_step(env)
            np.testing.assert_allclose(sensor.range_data, expected, atol=1e-9, rtol=0)
    finally:
        env.end(0)


def test_only_regular_circle_buffers_are_cast_as_circles():
    import shapely

    from irsim.lib.algorithm.ray_cast2d_batch import _as_circle_polygon

    angles, edge_start, edge_end, center, radius, vertices = _as_circle_polygon(
        shapely.Point(1, 2).buffer(0.5, quad_segs=4)
    )
    assert angles.shape == (16,)
    assert vertices.shape == (16, 2)
    assert np.all(np.diff(angles) > 0)
    # Every ring edge once, with its ends in the ring's direction.
    assert set(zip(edge_start.tolist(), edge_end.tolist(), strict=True)) == {
        (i, (i + 1) % 16) for i in range(16)
    }
    np.testing.assert_allclose(center, [1, 2], atol=1e-12)
    assert radius == pytest.approx(0.5)

    disc = shapely.Point(0, 0).buffer(1)
    for geometry in (
        shapely.affinity.scale(disc, 2, 1),  # inscribed radii differ
        shapely.box(0, 0, 2, 1),  # vertices on one circle, unevenly spaced
        shapely.Polygon(
            disc.exterior.coords, [shapely.Point(0, 0).buffer(0.5).exterior.coords]
        ),
        shapely.Polygon(),
        disc.union(shapely.Point(3, 0).buffer(1)),  # MultiPolygon
    ):
        assert _as_circle_polygon(geometry) is None


@pytest.mark.parametrize(
    "vertices",
    [
        [[2, 0], [5, 0]],  # ahead: hit at the near end
        [[5, 0], [2, 0]],  # same segment, reversed
        [[-1, 0], [3, 0]],  # through the sensor
        [[-1, 0], [15, 0]],  # through the sensor and past the range
        [[12, 0], [15, 0]],  # beyond range
        [[-5, 0], [-2, 0]],  # behind
    ],
)
def test_beam_along_a_segment_matches_the_reference_scan(backend, vertices, tmp_path):
    """A beam collinear with a segment reads what the per-sensor scan reads.

    Collinear beams are ambiguous, so the batch rescans them with the
    reference step.
    """
    cfg = _accuracy_scene(
        obstacles=[
            {"shape": {"name": "linestring", "vertices": vertices}, "state": [0, 0, 0]}
        ]
    )
    env = _make(cfg, tmp_path, "collinear")
    try:
        sensor = env.robot.lidar
        sensor._cast_alone(env.robot.state[:3])
        expected = (sensor.range_data.copy(), sensor.valid.copy())
        _batch_step(env)
        np.testing.assert_array_equal(sensor.range_data, expected[0])
        np.testing.assert_array_equal(sensor.valid, expected[1])
    finally:
        env.end(0)


def test_segment_bounding_boxes():
    from irsim.lib.algorithm.ray_cast2d_batch import Segments

    start = np.array([[0.0, 5.0], [3.0, -1.0]])
    end = np.array([[2.0, 1.0], [-4.0, 7.0]])
    segments = Segments.of(start, end, owner=4)
    np.testing.assert_array_equal(segments.box, [[0, 2, 1, 5], [-4, 3, -1, 7]])
    np.testing.assert_array_equal(segments.owner, [4, 4])


def test_collinear_hits_away_from_the_origin_on_several_beams(backend, tmp_path):
    """Two segments on one beam of a sensor away from the world origin.

    Only an axis-aligned beam is exactly collinear in floating point, so both
    segments lie on the beam at heading 0; the nearer end is read.
    """
    cfg = _accuracy_scene(
        obstacles=[
            {"shape": {"name": "linestring", "vertices": v}, "state": [0, 0, 0]}
            for v in ([[7, 2], [9, 2]], [[3, 2], [6, 2]])
        ],
    )
    cfg["robot"][0]["state"] = [1, 2, 0]
    env = _make(cfg, tmp_path, "collinear_off_origin")
    try:
        sensor = env.robot.lidar
        sensor._cast_alone(env.robot.state[:3])
        np.testing.assert_allclose(sensor.range_data, [2.0], atol=1e-12)
        _batch_step(env)
        np.testing.assert_allclose(sensor.range_data, [2.0], atol=1e-9, rtol=0)
    finally:
        env.end(0)


def test_replaced_body_geometry_is_cast_with_the_new_shape(backend, tmp_path):
    """A caster reused across steps follows a body whose shape was replaced."""
    import shapely

    sensors = [{"type": "lidar2d", "number": 21, "range_max": 10, "angle_range": 1.2}]
    cfg = _accuracy_scene(
        sensors=sensors,
        obstacles=[
            {"shape": {"name": "circle", "radius": 1}, "state": [4, 0.2, 0]},
            {
                "shape": {"name": "rectangle", "length": 1, "width": 1},
                "state": [6, -2.5, 0],
                "kinematics": {"name": "omni"},
            },
        ],
    )
    env = _make(cfg, tmp_path, "replaced_shapes")
    sensor = env.robot.lidar
    try:
        for radius, box in ((1.0, 1.0), (0.4, 2.5)):
            circle, rectangle = env.obstacle_list
            circle.set_original_geometry(shapely.Point(0, 0).buffer(radius))
            circle.set_state([4, 0.2, 0])
            rectangle.set_original_geometry(
                shapely.box(-box / 2, -box / 2, box / 2, box / 2)
            )
            rectangle.set_state([6, -2.5, 0])
            env.build_tree()
            sensor._cast_alone(env.robot.state[:3])
            expected = sensor.range_data.copy()
            _batch_step(env)
            np.testing.assert_array_equal(sensor.range_data, expected)
    finally:
        env.end(0)


def test_beam_along_a_circle_polygon_edge_uses_the_reference_scan(
    backend, tmp_path, monkeypatch
):
    """An exact scan whose beam runs along a circle polygon edge is ambiguous.

    The sensor is rescanned by the reference step, so the batch never needs a
    collinear distance of its own for circle polygons.
    """
    import shapely

    cfg = _accuracy_scene(
        obstacles=[{"shape": {"name": "circle", "radius": 1}, "state": [0, 0, 0]}]
    )
    cfg["robot"][0]["state"] = [-3, 1, 0]
    env = _make(cfg, tmp_path, "edge_collinear")
    try:
        body = env.obstacle_list[0]
        body.set_original_geometry(shapely.box(-1, -1, 1, 1))  # a regular 4-gon
        body.set_state([0, 0, 0])
        env.build_tree()
        sensor = env.robot.lidar
        sensor._cast_alone(env.robot.state[:3])
        expected = sensor.range_data.copy()
        np.testing.assert_allclose(expected, [2.0], atol=1e-12)

        fallbacks = []
        reference_step = Lidar2D._cast_alone
        monkeypatch.setattr(
            Lidar2D,
            "_cast_alone",
            lambda self, state: fallbacks.append(self) or reference_step(self, state),
        )
        _batch_step(env)

        assert ray_cast2d_batch._caster._circle_polygon(body) is not None, (
            "cast as a circle polygon"
        )
        assert fallbacks == [sensor]
        np.testing.assert_array_equal(sensor.range_data, expected)
    finally:
        env.end(0)


def test_batched_sensor_heading_is_wrapped_like_the_reference(backend, tmp_path):
    """Owner heading plus sensor offset past pi is stored wrapped to [-pi, pi]."""
    sensors = [
        {
            "type": "lidar2d",
            "number": 8,
            "range_max": 10,
            "offset": [0.1, 0, 0.3],
        }
    ]
    cfg = _accuracy_scene(sensors=sensors)
    cfg["robot"][0]["state"] = [0, 0, 3.0]
    env = _make(cfg, tmp_path, "wrapped_heading")
    try:
        sensor = env.robot.lidar
        sensor._cast_alone(env.robot.state[:3])
        expected = sensor.lidar_origin.copy()
        assert -np.pi <= expected[2, 0] <= np.pi
        _batch_step(env)
        np.testing.assert_array_equal(sensor.lidar_origin, expected)
    finally:
        env.end(0)


_MOVED_OBSTACLES = {
    "wall": {"shape": {"name": "linestring", "vertices": [[0, -2], [0, 2]]}},
    "circle": {"shape": {"name": "circle", "radius": 0.5}},
    "box": {"shape": {"name": "rectangle", "length": 1.0, "width": 1.0}},
}


@pytest.mark.parametrize("moved", ["wall", "circle", "box", "robot"])
@pytest.mark.parametrize("sensor_index", [0, 1], ids=["already_read", "not_yet_read"])
def test_an_object_moved_after_the_cast_is_seen_at_its_new_place(
    backend, moved, sensor_index, tmp_path
):
    """A manual move refreshes every scan, including one not yet read."""
    obstacle = dict(_MOVED_OBSTACLES.get(moved, _MOVED_OBSTACLES["wall"]))
    obstacle["state"] = [3.5, 0.1, 0]  # off the beam axis, so no vertex is hit
    sensors = [{"type": "lidar2d", "number": 1, "range_max": 10}] * 2
    env = _make(
        _accuracy_scene(sensors=sensors, obstacles=[obstacle]),
        tmp_path,
        f"moved_{moved}",
    )
    try:
        first, second = env.robot.sensors
        env.build_tree()
        first.step(env.robot.state[:3])  # second's cached scan is still unread
        before = first.range_data.copy()
        body = env.robot if moved == "robot" else env.obstacle_list[0]
        x, y, theta = body.state[:3, 0]
        body.set_state([x + 1, y, theta])  # one metre along the beam
        lidar = (first, second)[sensor_index]
        lidar.step(env.robot.state[:3])
        after = lidar.range_data.copy()
        lidar._cast_alone(env.robot.state[:3])
        np.testing.assert_array_equal(after, lidar.range_data)
        assert after[0] != before[0]
    finally:
        env.end(0)


def _count_casts(monkeypatch):
    """Record every cast of all lidars made by the process caster."""
    caster = ray_cast2d_batch._caster
    casts = []
    scan_group = caster._scan_group

    def counted(group, objects):
        casts.append(len(group.sensors))
        return scan_group(group, objects)

    monkeypatch.setattr(caster, "_scan_group", counted)
    return casts


def _assert_scans_match_cast_alone(env):
    for robot in env.robot_list:
        scan = robot.lidar.range_data.copy()
        robot.lidar._cast_alone(robot.state[:3])
        np.testing.assert_array_equal(scan, robot.lidar.range_data)


def _robots_before_a_wall(count=3):
    cfg = _accuracy_scene(
        obstacles=[
            {
                "shape": {"name": "linestring", "vertices": [[3, -1], [3, 7]]},
                "state": [0, 0, 0],
            }
        ]
    )
    cfg["robot"] = [
        {**cfg["robot"][0], "state": [0, 2 * i + 0.1, 0]} for i in range(count)
    ]
    return cfg


def test_a_manual_move_then_scan_loop_never_recasts_every_lidar(
    backend, tmp_path, monkeypatch
):
    """Each robot moved by hand scans alone: one move does not recast the
    lidars of every robot, and every scan matches the per-sensor scan."""
    env = _make(_robots_before_a_wall(), tmp_path, "manual_loop")
    try:
        casts = _count_casts(monkeypatch)
        for robot in env.robot_list:
            robot.step([0.5, 0.0])  # moves, then steps its own lidar
        assert casts == []
        _assert_scans_match_cast_alone(env)

        env.step([[0.5, 0.0]] * len(env.robot_list))
        assert len(casts) == 1  # a new step casts every lidar once
        _assert_scans_match_cast_alone(env)
    finally:
        env.end(0)


def test_moves_by_hand_then_scans_cast_every_lidar_once(backend, tmp_path, monkeypatch):
    """After many objects are moved by hand, the first lidar casts alone and
    the next one casts every lidar, whose scans the rest reuse."""
    env = _make(_robots_before_a_wall(), tmp_path, "moves_then_scans")
    try:
        casts = _count_casts(monkeypatch)
        for robot in env.robot_list:
            x, y, theta = robot.state[:3, 0]
            robot.set_state([x + 0.05, y, theta])
        for robot in env.robot_list:
            robot.sensor_step()
        assert len(casts) == 1
        _assert_scans_match_cast_alone(env)
    finally:
        env.end(0)


def test_a_contact_step_still_casts_every_lidar_once(backend, tmp_path, monkeypatch):
    """Contact pushes move objects after the step's tree rebuild; the step's
    lidars are still cast together, once, and match the per-sensor scan."""
    lidar = {"type": "lidar2d", "number": 9, "range_max": 10}
    cfg = {
        "world": {"collision_mode": "contact"},
        "robot": [
            {
                "kinematics": {"name": "omni"},
                "shape": {"name": "circle", "radius": 0.2},
                "state": [0, 2 * i + 0.1, 0],
                "sensors": [lidar],
            }
            for i in range(2)
        ],
        "obstacle": [
            {
                "shape": {"name": "rectangle", "length": 0.4, "width": 0.4},
                "mass": 1.0,
                "state": [0.45, 0.1, 0],
            }
        ],
    }
    env = _make(cfg, tmp_path, "contact_lidars")
    try:
        box = env.obstacle_list[0]
        start = box.state[0, 0]
        casts = _count_casts(monkeypatch)
        for _ in range(10):
            env.step([[1.0, 0.0], [0.0, 0.0]])
        assert box.state[0, 0] > start + 0.1  # the box was pushed
        assert len(casts) == 10
        _assert_scans_match_cast_alone(env)
    finally:
        env.end(0)


def test_a_pose_other_than_the_carriers_scans_from_that_pose(backend, tmp_path):
    """lidar.step(state) casts from ``state`` even when its batched scan is
    waiting to be taken."""
    env = _make(_accuracy_scene(), tmp_path, "other_pose")
    try:
        env.build_tree()  # a new pass: the scan from the carrier's pose waits
        lidar = env.robot.lidar
        lidar.step(np.array([[1.0], [0.0], [0.0]]))
        assert lidar.range_data[0] == pytest.approx(2.0)
        np.testing.assert_array_equal(lidar.state[:2, 0], [1.0, 0.0])
    finally:
        env.end(0)


def test_unchanged_scene_reuses_scans_with_fresh_range_noise(
    backend, tmp_path, monkeypatch
):
    """Repeated sensor steps reuse intersections but draw range noise again."""
    from irsim.util.random import set_seed

    env = _make(_accuracy_scene(), tmp_path, "reused_scan")
    try:
        lidar = env.robot.lidar
        lidar.noise, lidar.std = True, 0.1
        state = env.robot.state[:3]
        set_seed(7)
        expected = []
        for _ in range(3):
            lidar._cast_alone(state)
            expected.append(lidar.range_data.copy())

        def unexpected_cast(*args):
            pytest.fail("An unchanged scene should reuse the cached intersections")

        monkeypatch.setattr(ray_cast2d_batch._caster, "_cast", unexpected_cast)
        monkeypatch.setattr(lidar, "_cast_alone", unexpected_cast)
        set_seed(7)
        for ranges in expected:
            lidar.step(state)
            np.testing.assert_array_equal(lidar.range_data, ranges)
        assert expected[0][0] != expected[1][0]
    finally:
        env.end(0)


@pytest.mark.parametrize("quad_segs", [4, 16], ids=["different_count", "same_count"])
@pytest.mark.parametrize("moving", [False, True], ids=["static", "moving"])
@pytest.mark.parametrize("fresh_caster", [False, True], ids=["cached", "first_scan"])
def test_pending_circle_replacement_keeps_the_placed_shape(
    backend, quad_segs, moving, fresh_caster, tmp_path, monkeypatch
):
    """A replacement may shrink the radius or change the vertex count before
    the next placement; a tree rebuild must still scan the current body."""
    import shapely

    obstacle = {"shape": {"name": "circle", "radius": 1}, "state": [4, 0.8, 0]}
    if moving:
        obstacle["kinematics"] = {"name": "omni"}
    cfg = _accuracy_scene(obstacles=[obstacle])
    cfg["robot"][0]["shape"] = {"name": "rectangle", "length": 0.4, "width": 0.4}
    env = _make(cfg, tmp_path, "pending_circle")
    try:
        body, sensor = env.obstacle_list[0], env.robot.lidar
        sensor._cast_alone(env.robot.state[:3])
        original = sensor.range_data.copy()
        assert original[0] < sensor.range_max

        body.set_original_geometry(shapely.Point(0, 0).buffer(0.5, quad_segs=quad_segs))
        if fresh_caster:
            monkeypatch.setattr(ray_cast2d_batch, "_caster", RayCast2DBatch())
        _batch_step(env)
        np.testing.assert_array_equal(sensor.range_data, original)
        assert ray_cast2d_batch._caster.backend == backend

        body.set_state([4, 0.8, 0])
        env.robot.sensor_step()
        assert sensor.range_data[0] == sensor.range_max  # the smaller body is missed
        sensor._cast_alone(env.robot.state[:3])
        assert sensor.range_data[0] == sensor.range_max
    finally:
        env.end(0)


@pytest.mark.parametrize("kind", ["segments", "circles"])
@pytest.mark.parametrize("counts", [(5, 3), (3, 17), (0, 17), (5, 0)])
def test_numpy_candidates_are_generated_in_bounded_chunks(kind, counts, monkeypatch):
    """Even one sensor with more primitives than the limit stays bounded;
    dense results are yielded on demand, in sensor order."""
    n_sensors, n_items = counts
    rays = ray_cast2d_batch.Rays(
        np.zeros((n_sensors, 2)),
        np.tile([1.0, 0.0], (n_sensors, 1, 1)),
        np.ones(n_sensors),
        np.arange(n_sensors),
    )
    centers = np.tile([0.5, 0.0], (n_items, 1))
    owners = np.arange(n_items)
    if n_items:
        centers[-1, 0] = 10  # out of range
    nonzero = np.nonzero
    tested_sizes = []

    def bounded_nonzero(near):
        assert near.size <= 7
        tested_sizes.append(near.size)
        return nonzero(near)

    monkeypatch.setattr(ray_cast2d_batch, "MAX_BLOCK_ELEMENTS", 7)
    monkeypatch.setattr(np, "nonzero", bounded_nonzero)
    if kind == "segments":
        offset = np.array([0.0, 0.1])
        segments = ray_cast2d_batch.Segments.of(
            centers - offset, centers + offset, owner=0
        )._replace(owner=owners)
        chunks = ray_cast2d_batch._segments_in_range(rays, segments)
    else:
        circles = ray_cast2d_batch._NO_CIRCLES._replace(
            owner=owners, radius=np.full(n_items, 0.1)
        )
        chunks = ray_cast2d_batch._circles_in_range(rays, centers, circles)
    assert tested_sizes == []  # no full candidate list is allocated up front
    found = []
    for sensor, item in chunks:
        assert len(sensor) <= 7
        found.extend(zip(sensor.tolist(), item.tolist(), strict=True))
    assert found == [
        (sensor, item)
        for sensor in range(n_sensors)
        for item in range(n_items - 1)
        if sensor != item
    ]
    assert sum(tested_sizes) == n_sensors * n_items


def test_small_candidate_blocks_preserve_scans(backend, tmp_path, monkeypatch):
    monkeypatch.setattr(ray_cast2d_batch, "MAX_BLOCK_ELEMENTS", 7)
    _assert_same_scans(
        _load(TESTS / "test_collision_avoidance.yaml", _ensure_lidar), tmp_path, steps=3
    )
