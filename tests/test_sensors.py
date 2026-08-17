import matplotlib.pyplot as plt
import numpy as np
import pytest
import shapely
from shapely import STRtree

import irsim.lib.algorithm.ray_casting_2d as ray_casting_2d
from irsim.lib.algorithm.ray_casting_2d import cast_ray_segments, cast_rays
from irsim.lib.handler.geometry_handler import GeometryFactory
from irsim.util.random import rng, set_seed
from irsim.world.object_factory import ObjectFactory
from irsim.world.sensors.fmcw_lidar2d import FMCWLidar2D
from irsim.world.sensors.lidar2d import Lidar2D
from irsim.world.sensors.sensor_factory import SensorFactory


class _DummySensorEnvParam:
    def __init__(self, objects, geometry_tree):
        self.objects = objects
        self.GeometryTree = geometry_tree


class _DummySensorEnv:
    def __init__(self, env_param):
        self._env_param = env_param


class _DummySensorParent:
    def __init__(self, velocity_xy, env_param):
        self._velocity_xy = np.c_[velocity_xy]
        self._env = _DummySensorEnv(env_param)

    @property
    def velocity_xy(self):
        return self._velocity_xy


class _DummyObstacleObject:
    def __init__(
        self,
        obj_id,
        geometry,
        velocity_xy=(0.0, 0.0),
        shape="circle",
        unobstructed=False,
        geometry_valid=True,
    ):
        self._id = obj_id
        self._geometry = geometry
        self._geometry_valid = geometry_valid
        self.shape = shape
        self.unobstructed = unobstructed
        self._velocity_xy = np.c_[velocity_xy]

    @property
    def geometry(self):
        return self._geometry

    @property
    def velocity_xy(self):
        return self._velocity_xy


class _DummyMapObject:
    """Mimics a grid-map object that exposes ``linestrings`` + an STRtree."""

    def __init__(self, obj_id, linestrings):
        self._id = obj_id
        self.linestrings = list(linestrings)
        self._geometry = shapely.MultiLineString(self.linestrings)
        self._geometry_valid = True
        self.shape = "map"
        self.unobstructed = False
        self.geometry_tree = STRtree(self.linestrings)
        self._velocity_xy = np.c_[(0.0, 0.0)]

    @property
    def geometry(self):
        return self._geometry

    @property
    def velocity_xy(self):
        return self._velocity_xy


def _legacy_intersection_points(geometry):
    """Yield the representative points used by the previous FMCW scanner."""
    if geometry.is_empty:
        return
    if geometry.geom_type == "Point":
        yield geometry
    elif geometry.geom_type == "MultiPoint":
        yield from geometry.geoms
    elif geometry.geom_type in {"LineString", "LinearRing"}:
        for coordinate in geometry.coords:
            yield shapely.Point(coordinate)
    elif geometry.geom_type in {"MultiLineString", "GeometryCollection"}:
        for part in geometry.geoms:
            yield from _legacy_intersection_points(part)


def _legacy_fmcw_scan(sensor):
    """Run the pre-vectorization per-beam FMCW measurement algorithm."""
    origin = np.array([sensor.lidar_origin[0, 0], sensor.lidar_origin[1, 0]])
    origin_point = shapely.Point(*origin)
    sensor_theta = float(sensor.lidar_origin[2, 0])
    objects = sensor._env_param.objects
    tree = sensor._env_param.GeometryTree

    ranges = np.full(sensor.number, sensor.range_max, dtype=float)
    velocities = np.zeros(sensor.number)
    valid = np.zeros(sensor.number, dtype=bool)
    if tree is None:
        return ranges, velocities, valid

    for beam, beam_angle in enumerate(sensor.angle_list):
        world_angle = sensor_theta + beam_angle
        direction = np.array([np.cos(world_angle), np.sin(world_angle)])
        ray = shapely.LineString([origin, origin + sensor.range_max * direction])
        best_distance = None
        best_object = None

        for geom_index in tree.query(ray):
            obj = objects[geom_index]
            if obj._id == sensor.obj_id or not obj._geometry_valid or obj.unobstructed:
                continue
            if obj.shape == "map":
                indices = obj.geometry_tree.query(ray, predicate="intersects")
                if len(indices) == 0:
                    continue
                geometry = shapely.MultiLineString(
                    [obj.linestrings[index] for index in indices]
                )
            else:
                if not ray.intersects(obj.geometry):
                    continue
                geometry = obj.geometry

            intersection = ray.intersection(geometry)
            for point in _legacy_intersection_points(intersection):
                distance = origin_point.distance(point)
                if distance <= 1e-9 or distance > sensor.range_max + 1e-9:
                    continue
                if best_distance is None or distance < best_distance:
                    best_distance = distance
                    best_object = obj

        if best_distance is None:
            continue
        if sensor.noise:
            best_distance += rng.normal(0, sensor.std)
        if sensor.range_min <= best_distance <= sensor.range_max:
            ranges[beam] = best_distance
            valid[beam] = True
            velocity = sensor._compute_radial_velocity(best_object, direction)
            if sensor.velocity_noise_std > 0:
                velocity += rng.normal(0, sensor.velocity_noise_std)
            velocities[beam] = velocity

    return ranges, velocities, valid


class TestFMCWLidar2D:
    def test_factory_creates_fmcw_sensor(self):
        """SensorFactory should create the simplified FMCW lidar."""
        factory = SensorFactory()
        sensor = factory.create_sensor(
            np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            type="fmcw_lidar2d",
            number=1,
            angle_range=0.0,
            range_max=5.0,
        )

        assert isinstance(sensor, FMCWLidar2D)
        assert sensor.sensor_type == "fmcw_lidar2d"

    def test_plot_subdict_overrides_and_flat_fallback(self):
        """Visualization params resolve from `plot:` first, then flat keys."""
        factory = SensorFactory()

        # fmcw: plot: wins over a flat key; a flat-only key still applies.
        sensor = factory.create_sensor(
            np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            type="fmcw_lidar2d",
            number=1,
            angle_range=0.0,
            range_max=5.0,
            velocity_marker_size=10,  # flat (should be overridden)
            no_hit_alpha=0.123,  # flat-only (should apply)
            plot={"velocity_marker_size": 50, "velocity_linewidth": 3.5},
        )
        assert sensor.velocity_marker_size == 50
        assert sensor.velocity_linewidth == 3.5
        assert sensor.no_hit_alpha == 0.123

        # lidar2d: alpha/color honored from the plot: sub-dict.
        lidar = factory.create_sensor(
            np.array([[0.0], [0.0], [0.0]]),
            obj_id=2,
            name="lidar2d",
            number=1,
            angle_range=0.0,
            range_max=5.0,
            plot={"alpha": 0.7, "color": "g"},
        )
        assert lidar.alpha == 0.7
        assert lidar.color == "g"

        # lidar2d: flat keys still work without a plot: dict (back-compat).
        lidar_flat = factory.create_sensor(
            np.array([[0.0], [0.0], [0.0]]),
            obj_id=3,
            name="lidar2d",
            number=1,
            angle_range=0.0,
            range_max=5.0,
            color="b",
        )
        assert lidar_flat.color == "b"

    def test_object_base_recognizes_fmcw_as_lidar(self):
        """ObjectBase should expose FMCW lidar via the existing lidar slot."""
        obj = ObjectFactory().create_object(
            obj_type="robot",
            kinematics={"name": "diff"},
            shape={"name": "circle", "radius": 0.2},
            sensors=[
                {
                    "type": "fmcw_lidar2d",
                    "number": 1,
                    "angle_range": 0.0,
                    "range_max": 5.0,
                }
            ],
        )[0]

        assert obj.lidar is not None
        assert obj.lidar.sensor_type == "fmcw_lidar2d"

    def test_radial_velocity_uses_relative_motion_by_default(self):
        """Stationary targets should show ego-relative Doppler by default."""
        from shapely import STRtree

        obstacle = _DummyObstacleObject(
            obj_id=2, geometry=shapely.Point(2.0, 0.0).buffer(0.2)
        )
        env = _DummySensorEnvParam([obstacle], STRtree([obstacle.geometry]))
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=1,
            angle_range=0.0,
            range_max=5.0,
        )
        sensor.parent = _DummySensorParent([1.0, 0.0], env)

        sensor.step(sensor.state)
        scan = sensor.get_scan()

        assert scan["valid"][0]
        assert scan["ranges"][0] == pytest.approx(1.8, abs=1e-3)
        assert scan["radial_velocity"][0] == pytest.approx(-1.0, abs=1e-6)
        assert scan["intensities"] is None

    def test_motion_compensation_removes_ego_velocity(self):
        """Motion compensation should report world-frame radial target speed."""
        from shapely import STRtree

        obstacle = _DummyObstacleObject(
            obj_id=2, geometry=shapely.Point(2.0, 0.0).buffer(0.2)
        )
        env = _DummySensorEnvParam([obstacle], STRtree([obstacle.geometry]))
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=1,
            angle_range=0.0,
            range_max=5.0,
            motion_compensate=True,
        )
        sensor.parent = _DummySensorParent([1.0, 0.0], env)

        sensor.step(sensor.state)

        assert sensor.radial_velocity[0] == pytest.approx(0.0, abs=1e-6)

    def test_tangential_motion_has_zero_radial_velocity(self):
        """Purely tangential target motion should not create Doppler shift."""
        from shapely import STRtree

        obstacle = _DummyObstacleObject(
            obj_id=2,
            geometry=shapely.Point(2.0, 0.0).buffer(0.2),
            velocity_xy=(0.0, 2.0),
        )
        env = _DummySensorEnvParam([obstacle], STRtree([obstacle.geometry]))
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=1,
            angle_range=0.0,
            range_max=5.0,
        )
        sensor.parent = _DummySensorParent([0.0, 0.0], env)

        sensor.step(sensor.state)

        assert sensor.radial_velocity[0] == pytest.approx(0.0, abs=1e-6)

    def test_no_hit_keeps_default_range_and_zero_velocity(self):
        """A beam with no hit should stay invalid at max range."""
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=1,
            angle_range=0.0,
            range_max=5.0,
        )
        sensor.parent = _DummySensorParent(
            [0.0, 0.0],
            _DummySensorEnvParam([], None),
        )

        sensor.step(sensor.state)
        scan = sensor.get_scan()

        assert not scan["valid"][0]
        assert scan["ranges"][0] == pytest.approx(5.0, abs=1e-6)
        assert scan["radial_velocity"][0] == pytest.approx(0.0, abs=1e-6)
        assert sensor.get_points() is None

    def test_velocity_visuals_use_different_colors(self):
        """Positive, negative, and invalid beams should render with distinct colors."""
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=3,
            angle_range=1.0,
            range_max=5.0,
            color="white",
            velocity_color_max=2.0,
        )
        sensor.valid[:] = np.array([True, True, False])
        sensor.radial_velocity[:] = np.array([1.0, -1.0, 0.0])

        colors, alphas = sensor._get_velocity_visuals()

        assert colors[0] != colors[1]
        assert colors[0] != colors[2]
        assert colors[1] != colors[2]
        assert alphas[0] == pytest.approx(1.0)
        assert alphas[1] == pytest.approx(1.0)
        assert alphas[2] < sensor.alpha

    def test_velocity_visuals_do_not_mutate_measurements(self):
        """Color mapping should not alter the computed range or Doppler data."""
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=2,
            angle_range=0.5,
            range_max=5.0,
        )
        sensor.valid[:] = np.array([True, False])
        sensor.range_data[:] = np.array([1.5, 5.0])
        sensor.radial_velocity[:] = np.array([0.75, 0.0])

        ranges_before = sensor.range_data.copy()
        velocity_before = sensor.radial_velocity.copy()

        sensor._get_velocity_visuals()

        assert np.array_equal(sensor.range_data, ranges_before)
        assert np.array_equal(sensor.radial_velocity, velocity_before)

    def test_velocity_visuals_use_wider_lines_for_hits(self):
        """Valid beams should render wider than no-hit beams."""
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=2,
            angle_range=0.5,
            range_max=5.0,
            velocity_linewidth=3.0,
            no_hit_linewidth=0.5,
        )
        sensor.valid[:] = np.array([True, False])

        linewidths = sensor._get_velocity_linewidths()

        assert linewidths == [3.0, 0.5]

    def test_velocity_markers_only_include_valid_hits(self):
        """Endpoint markers should only be generated for valid beams."""
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=3,
            angle_range=1.0,
            range_max=5.0,
        )
        sensor.valid[:] = np.array([True, False, True])
        sensor.range_data[:] = np.array([1.0, 5.0, 2.0])

        points, colors = sensor._get_velocity_marker_points()

        assert points.shape == (2, 2)
        assert colors.shape == (2, 3)

    def test_range_and_velocity_noise_are_applied(self):
        """``noise=True`` and ``velocity_noise_std>0`` should perturb measurements."""
        obstacle = _DummyObstacleObject(
            obj_id=2,
            geometry=shapely.Point(2.0, 0.0).buffer(0.2),
            velocity_xy=(0.5, 0.0),
        )
        env = _DummySensorEnvParam([obstacle], STRtree([obstacle.geometry]))
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=1,
            angle_range=0.0,
            range_max=5.0,
            noise=True,
            std=0.1,
            velocity_noise_std=0.2,
        )
        sensor.parent = _DummySensorParent([0.0, 0.0], env)

        set_seed(123)
        sensor.step(sensor.state)
        noisy_range = sensor.range_data[0]
        noisy_velocity = sensor.radial_velocity[0]

        # Repeat without noise to compare against the deterministic baseline.
        sensor.noise = False
        sensor.velocity_noise_std = 0.0
        sensor.step(sensor.state)

        assert noisy_range != pytest.approx(sensor.range_data[0], abs=1e-9)
        assert noisy_velocity != pytest.approx(sensor.radial_velocity[0], abs=1e-9)
        # Noise should stay close to the baseline within a generous bound.
        assert abs(noisy_range - sensor.range_data[0]) < 1.0
        assert abs(noisy_velocity - sensor.radial_velocity[0]) < 2.0

    def test_noise_pushed_out_of_range_invalidates_beam(self):
        """Hits whose noisy range falls outside [range_min, range_max] are invalid.

        Keeps ``valid`` consistent with ``range_data`` so downstream code
        (e.g. ``get_points``, which drops beams at ``range_max``) cannot
        silently disagree with the validity mask.
        """
        # Obstacle just inside range_max; large noise pushes most reads past it.
        obstacle = _DummyObstacleObject(
            obj_id=2, geometry=shapely.Point(4.95, 0.0).buffer(0.05)
        )
        env = _DummySensorEnvParam([obstacle], STRtree([obstacle.geometry]))
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=1,
            angle_range=0.0,
            range_max=5.0,
            noise=True,
            std=10.0,  # so large that almost every draw exits [range_min, range_max]
        )
        sensor.parent = _DummySensorParent([0.0, 0.0], env)

        set_seed(0)
        invalidated_at_max = 0
        for _ in range(200):
            sensor.step(sensor.state)
            if not sensor.valid[0]:
                invalidated_at_max += 1
                # Invalidated beams must report range_max and zero Doppler so
                # consumers see a coherent "no usable return" signal.
                assert sensor.range_data[0] == pytest.approx(5.0, abs=1e-9)
                assert sensor.radial_velocity[0] == pytest.approx(0.0, abs=1e-9)
            else:
                # Valid hits must always lie within the documented bounds.
                assert sensor.range_min <= sensor.range_data[0] <= sensor.range_max

        assert invalidated_at_max > 0  # the failure mode is actually exercised

    def test_step_skips_invalid_candidates(self):
        """A scan must skip self, invalid, and unobstructed objects."""
        circle = shapely.Point(2.0, 0.0).buffer(0.2)
        self_obj = _DummyObstacleObject(obj_id=1, geometry=circle)
        invalid_obj = _DummyObstacleObject(
            obj_id=3, geometry=circle, geometry_valid=False
        )
        unobstructed_obj = _DummyObstacleObject(
            obj_id=4, geometry=circle, unobstructed=True
        )
        # An off-axis obstacle whose tree query matches the beam bbox but whose
        # geometry does NOT actually intersect the ray (covers the None branch).
        off_axis = _DummyObstacleObject(
            obj_id=5, geometry=shapely.Point(1.5, 1.0).buffer(0.05)
        )
        # The actual hit object the beam should keep.
        hit = _DummyObstacleObject(
            obj_id=6,
            geometry=shapely.Point(3.0, 0.0).buffer(0.3),
            velocity_xy=(0.0, 0.0),
        )

        objects = [self_obj, invalid_obj, unobstructed_obj, off_axis, hit]
        env = _DummySensorEnvParam(objects, STRtree([o.geometry for o in objects]))
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=1,
            angle_range=0.0,
            range_max=5.0,
        )
        sensor.parent = _DummySensorParent([0.0, 0.0], env)

        sensor.step(sensor.state)

        assert sensor.valid[0]
        assert sensor.range_data[0] == pytest.approx(2.7, abs=1e-3)

    def test_map_shape_obstacle_is_intersected(self):
        """``shape == 'map'`` should route through ``geometry_tree`` and hit a wall."""
        wall = shapely.LineString([(2.0, -1.0), (2.0, 1.0)])
        far_wall = shapely.LineString([(4.0, -1.0), (4.0, 1.0)])
        map_obj = _DummyMapObject(obj_id=2, linestrings=[wall, far_wall])
        env = _DummySensorEnvParam([map_obj], STRtree([map_obj.geometry]))
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=1,
            angle_range=0.0,
            range_max=5.0,
        )
        sensor.parent = _DummySensorParent([0.0, 0.0], env)

        sensor.step(sensor.state)

        assert sensor.valid[0]
        assert sensor.range_data[0] == pytest.approx(2.0, abs=1e-6)

    def test_map_shape_with_no_intersection_returns_no_hit(self):
        """Map objects whose linestrings miss the beam should leave the beam invalid."""
        far_wall = shapely.LineString([(0.0, 4.0), (1.0, 4.0)])
        map_obj = _DummyMapObject(obj_id=2, linestrings=[far_wall])

        # Force the tree query to always return the map object so the
        # ``len(intersecting_indices) == 0`` branch is exercised.
        class _FakeTree:
            def query(self, _ray):
                return [0]

        env = _DummySensorEnvParam([map_obj], _FakeTree())
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=1,
            angle_range=0.0,
            range_max=5.0,
        )
        sensor.parent = _DummySensorParent([0.0, 0.0], env)

        sensor.step(sensor.state)

        assert not sensor.valid[0]
        assert sensor.range_data[0] == pytest.approx(5.0, abs=1e-6)

    def test_sensor_velocity_xy_falls_back_when_parent_missing(self):
        """``_sensor_velocity_xy`` should default to zeros without a parent."""
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=1,
            angle_range=0.0,
            range_max=5.0,
        )
        sensor.parent = None

        np.testing.assert_array_equal(sensor._sensor_velocity_xy(), np.zeros(2))

        # Default branch when parent exists but lacks ``velocity_xy``.
        sensor.parent = object()
        np.testing.assert_array_equal(sensor._sensor_velocity_xy(), np.zeros(2))

    def test_plot_and_step_plot_render_velocity_visuals(self):
        """End-to-end plotting should populate the line collection and markers."""
        obstacle = _DummyObstacleObject(
            obj_id=2,
            geometry=shapely.Point(2.0, 0.0).buffer(0.2),
            velocity_xy=(1.0, 0.0),
        )
        env = _DummySensorEnvParam([obstacle], STRtree([obstacle.geometry]))
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=5,
            angle_range=0.4,
            range_max=5.0,
            velocity_color=True,
            show_velocity_markers=True,
        )
        sensor.parent = _DummySensorParent([0.0, 0.0], env)
        sensor.step(sensor.state)

        fig = plt.figure()
        ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
        try:
            sensor._plot(ax, sensor.state)
            assert hasattr(sensor, "laser_LineCollection")
            assert hasattr(sensor, "velocity_marker_plot")
            assert sensor.velocity_marker_plot in sensor.plot_patch_list

            # Re-step so _step_plot exercises the update branches.
            sensor.step(sensor.state)
            sensor._step_plot()

            # When no beams are valid the marker update path returns the
            # ``np.empty((0, 4))`` fallback color array.
            sensor.valid[:] = False
            sensor._update_velocity_markers()
        finally:
            plt.close(fig)

    def test_step_plot_skips_when_velocity_color_disabled(self):
        """Disabling velocity_color should bypass the visuals branch entirely."""
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=2,
            angle_range=0.5,
            range_max=5.0,
            velocity_color=False,
            show_velocity_markers=False,
        )
        sensor.parent = _DummySensorParent([0.0, 0.0], _DummySensorEnvParam([], None))
        sensor.step(sensor.state)

        # Both visual hooks should be no-ops without raising.
        sensor._apply_velocity_visuals()
        sensor._update_velocity_markers()
        assert not hasattr(sensor, "laser_LineCollection")
        assert not hasattr(sensor, "velocity_marker_plot")

    def test_noise_is_seeded_reproducible(self):
        """Seeded range + velocity noise must reproduce run-to-run.

        The per-beam draw order (range noise, then velocity noise) is what keeps
        a seeded FMCW scan reproducible; batching the draws would silently change
        the noise sequence. Regression lock for that draw order.
        """
        from irsim.util.random import set_seed

        obstacle = _DummyObstacleObject(
            obj_id=2,
            geometry=shapely.Point(2.0, 0.0).buffer(0.6),
            velocity_xy=(1.0, -0.5),
        )
        env = _DummySensorEnvParam([obstacle], STRtree([obstacle.geometry]))

        def run(noise):
            set_seed(5)
            sensor = FMCWLidar2D(
                state=np.array([[0.0], [0.0], [0.0]]),
                obj_id=1,
                number=64,
                angle_range=6.283185,
                range_max=5.0,
                noise=noise,
                std=0.1,
                velocity_noise_std=0.2,
            )
            sensor.parent = _DummySensorParent([0.3, 0.0], env)
            sensor.step(sensor.state)
            return sensor

        a = run(True)
        b = run(True)
        np.testing.assert_array_equal(a.range_data, b.range_data)
        np.testing.assert_array_equal(a.radial_velocity, b.radial_velocity)
        np.testing.assert_array_equal(a.valid, b.valid)

        # Noise actually perturbed the hits vs the clean scan.
        clean = run(False)
        hit = a.valid
        assert hit.any()
        assert np.any(a.range_data[hit] != clean.range_data[hit])

    def test_scan_matches_legacy_fmcw_with_noise_map_and_motion(self):
        """All FMCW measurement fields match the previous per-beam scanner."""
        wall = shapely.LineString([(-2.0, -2.0), (-2.0, 2.0)])
        map_obj = _DummyMapObject(obj_id=2, linestrings=[wall])
        moving = _DummyObstacleObject(
            obj_id=3,
            geometry=shapely.Point(2.0, 0.0).buffer(0.6),
            velocity_xy=(1.0, -0.5),
        )
        objects = [map_obj, moving]
        env = _DummySensorEnvParam(objects, STRtree([obj.geometry for obj in objects]))
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.2]]),
            obj_id=1,
            number=64,
            angle_range=6.283185,
            range_min=0.1,
            range_max=5.0,
            offset=[0.2, -0.1, 0.15],
            noise=True,
            std=0.05,
            velocity_noise_std=0.02,
        )
        sensor.parent = _DummySensorParent([0.3, 0.1], env)

        set_seed(123)
        expected_ranges, expected_velocity, expected_valid = _legacy_fmcw_scan(sensor)
        set_seed(123)
        sensor.step(sensor.state)
        scan = sensor.get_scan()

        np.testing.assert_array_equal(scan["valid"], expected_valid)
        np.testing.assert_allclose(scan["ranges"], expected_ranges, atol=1e-12, rtol=0)
        np.testing.assert_allclose(
            scan["radial_velocity"], expected_velocity, atol=1e-12, rtol=0
        )
        assert tuple(scan) == (
            "angle_min",
            "angle_max",
            "angle_increment",
            "time_increment",
            "scan_time",
            "range_min",
            "range_max",
            "ranges",
            "intensities",
            "radial_velocity",
            "valid",
        )
        assert scan["intensities"] is None

    def test_collinear_wall_matches_legacy_fmcw(self):
        """A beam aligned with a wall must hit its nearest overlapping endpoint."""
        wall = shapely.LineString([(2.0, 0.0), (4.0, 0.0)])
        obstacle = _DummyObstacleObject(
            obj_id=2,
            geometry=wall,
            velocity_xy=(0.5, 0.0),
            shape="linestring",
        )
        env = _DummySensorEnvParam([obstacle], STRtree([wall]))
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=1,
            angle_range=0.0,
            range_max=5.0,
        )
        sensor.parent = _DummySensorParent([0.0, 0.0], env)

        expected_ranges, expected_velocity, expected_valid = _legacy_fmcw_scan(sensor)
        sensor.step(sensor.state)

        np.testing.assert_array_equal(sensor.valid, expected_valid)
        np.testing.assert_allclose(sensor.range_data, expected_ranges, atol=1e-12)
        np.testing.assert_allclose(
            sensor.radial_velocity, expected_velocity, atol=1e-12
        )
        assert sensor.range_data[0] == pytest.approx(2.0)

    def test_scan_matches_legacy_fmcw_in_randomized_free_space(self):
        """Random poses, offsets, shapes, motion, and noise match legacy GEOS."""
        random = np.random.default_rng(20260817)

        for scene in range(20):
            state = np.array(
                [
                    [random.uniform(-2.0, 2.0)],
                    [random.uniform(-2.0, 2.0)],
                    [random.uniform(-np.pi, np.pi)],
                ]
            )
            offset = [
                random.uniform(-0.3, 0.3),
                random.uniform(-0.3, 0.3),
                random.uniform(-0.4, 0.4),
            ]
            number = int(random.choice([1, 31, 64, 181]))
            angle_range = (
                0.0 if number == 1 else float(random.choice([0.7, 3.14, 6.283185]))
            )
            sensor = FMCWLidar2D(
                state=state,
                obj_id=1,
                number=number,
                angle_range=angle_range,
                range_min=0.05,
                range_max=8.0,
                offset=offset,
                noise=scene % 2 == 0,
                std=0.03,
                velocity_noise_std=0.02,
            )

            origin = sensor.lidar_origin[:2, 0]
            objects = []
            for object_id in range(2, 10):
                bearing = random.uniform(-np.pi, np.pi)
                center = origin + random.uniform(1.2, 6.5) * np.array(
                    [np.cos(bearing), np.sin(bearing)]
                )
                velocity = random.uniform(-0.8, 0.8, size=2)
                if object_id % 2:
                    geometry = shapely.Point(center).buffer(
                        random.uniform(0.1, 0.55), quad_segs=8
                    )
                    shape = "circle"
                else:
                    tangent = np.array([-np.sin(bearing), np.cos(bearing)])
                    half_length = random.uniform(0.15, 0.8)
                    geometry = shapely.LineString(
                        [center - half_length * tangent, center + half_length * tangent]
                    )
                    shape = "linestring"
                objects.append(
                    _DummyObstacleObject(
                        object_id,
                        geometry,
                        velocity_xy=velocity,
                        shape=shape,
                    )
                )

            env = _DummySensorEnvParam(
                objects,
                STRtree([obj.geometry for obj in objects]),
            )
            sensor.parent = _DummySensorParent(random.uniform(-0.5, 0.5, size=2), env)

            set_seed(1000 + scene)
            expected_ranges, expected_velocity, expected_valid = _legacy_fmcw_scan(
                sensor
            )
            set_seed(1000 + scene)
            sensor.step(sensor.state)

            np.testing.assert_array_equal(sensor.valid, expected_valid)
            np.testing.assert_allclose(
                sensor.range_data,
                expected_ranges,
                atol=1e-11,
                rtol=0,
            )
            np.testing.assert_allclose(
                sensor.radial_velocity,
                expected_velocity,
                atol=1e-12,
                rtol=0,
            )
            np.testing.assert_allclose(
                shapely.length(shapely.get_parts(sensor._geometry)),
                sensor.range_data,
                atol=1e-12,
                rtol=0,
            )


# ---------------------------------------------------------------------------
# Lidar2D grouped-difference scan (issue #302): linestrings and polygons are
# differenced separately for speed; both must still block beams in one scan.
# ---------------------------------------------------------------------------


class _EnvParam:
    def __init__(self, objects, tree):
        self.objects = objects
        self.GeometryTree = tree


class _Env:
    def __init__(self, env_param):
        self._env_param = env_param


class _Parent:
    def __init__(self, env_param):
        self._env = _Env(env_param)


class _MapObject:
    """Mimics a grid-map object: linestrings + an STRtree (``shape == 'map'``)."""

    def __init__(self, obj_id, linestrings):
        self._id = obj_id
        self.linestrings = list(linestrings)
        self._geometry = shapely.MultiLineString(self.linestrings)
        self._geometry_valid = True
        self.shape = "map"
        self.unobstructed = False
        self.geometry_tree = STRtree(self.linestrings)

    @property
    def geometry(self):
        return self._geometry


class _Obstacle:
    """Mimics a dynamic obstacle with a polygon geometry."""

    def __init__(self, obj_id, geometry, shape="circle", velocity_xy=(0.0, 0.0)):
        self._id = obj_id
        self._geometry = geometry
        self._geometry_valid = True
        self.shape = shape
        self.unobstructed = False
        self._velocity_xy = np.asarray(velocity_xy, dtype=float).reshape(2, 1)

    @property
    def geometry(self):
        return self._geometry

    @property
    def velocity_xy(self):
        return self._velocity_xy


def test_lidar_subtracts_map_lines_and_dynamic_polygon():
    """A static map linestring and a dynamic polygon must both block beams in one
    scan, exercising the grouped-difference path added for issue #302."""
    range_max = 10.0
    # static map: a short vertical wall at x = -3
    map_obj = _MapObject(2, [shapely.LineString([(-3.0, -0.5), (-3.0, 0.5)])])
    # dynamic obstacle: a circle (polygon) centred at x = 2, radius 0.5
    circle = _Obstacle(3, shapely.Point(2.0, 0.0).buffer(0.5))
    env_param = _EnvParam(
        [map_obj, circle], STRtree([map_obj.geometry, circle.geometry])
    )

    lidar = Lidar2D(
        state=np.array([[0.0], [0.0], [0.0]]),
        obj_id=1,
        number=36,
        angle_range=6.283185,
        range_max=range_max,
    )
    lidar.parent = _Parent(env_param)
    lidar.step(lidar.state)

    ranges = np.asarray(lidar.get_scan()["ranges"])
    blocked = ranges[ranges < range_max - 1e-6]

    # both kinds of obstacle were subtracted: the nearest hit is the polygon
    # (circle, near edge at x=1.5) and a farther hit is the line (wall at x=-3).
    assert len(blocked) >= 2
    assert ranges.min() == pytest.approx(1.5, abs=0.3)  # circle -> polygon group
    assert blocked.max() == pytest.approx(3.0, abs=0.3)  # wall   -> line group


def test_lidar_detects_disjoint_compound_geometry():
    """A MultiPolygon compound is treated as one obstacle without filling gaps."""
    compound = GeometryFactory.create_geometry(
        "compound",
        parts=[
            {
                "name": "rectangle",
                "length": 0.5,
                "width": 1.0,
                "pose": [2.0, 0.0, 0.0],
            },
            {
                "name": "rectangle",
                "length": 0.5,
                "width": 1.0,
                "pose": [4.0, 0.0, 0.0],
            },
        ],
    )
    obstacle = _Obstacle(2, compound.geometry, shape="compound")
    env_param = _EnvParam([obstacle], STRtree([obstacle.geometry]))
    lidar = Lidar2D(
        state=np.array([[0.0], [0.0], [0.0]]),
        obj_id=1,
        number=1,
        angle_range=0.0,
        range_max=6.0,
    )
    lidar.parent = _Parent(env_param)

    lidar.step(lidar.state)

    assert lidar.range_data[0] == pytest.approx(1.75, abs=1e-6)


def test_lidar_single_beam_tof():
    """A single-beam (1D ToF) lidar produces a valid MultiLineString scan.

    PR #200 added number=1 (1D ToF) support and needed a MultiLineString
    normalization workaround because the old GEOS ``difference`` returned a bare
    LineString for one beam. The ray-cast path builds the scan geometry natively
    as a MultiLineString, so the ToF case stays correct without that workaround.
    """
    obstacle = _Obstacle(2, shapely.box(2.0, -1.0, 2.5, 1.0), shape="rectangle")
    env_param = _EnvParam([obstacle], STRtree([obstacle.geometry]))
    tof = Lidar2D(
        state=np.array([[0.0], [0.0], [0.0]]),
        obj_id=1,
        number=1,
        angle_range=0.0,
        range_max=5.0,
    )
    tof.parent = _Parent(env_param)
    tof.step(tof.state)

    assert tof.range_data.shape == (1,)
    assert tof.range_data[0] == pytest.approx(2.0, abs=1e-6)
    assert tof._geometry.geom_type == "MultiLineString"
    assert len(shapely.get_parts(tof._geometry)) == 1
    assert np.asarray(tof.get_scan()["ranges"]).shape == (1,)
    assert tof.get_points().shape == (2, 1)


def _geos_difference_ranges(lidar, obstacle_geometries):
    """Reference range_data via the pre-ray-cast whole-geometry difference.

    Reproduces the previous GEOS approach: subtract linestrings and polygons as
    two merged groups, keep the origin-connected beam parts, and read their
    lengths. Used to prove the ray-cast reproduces that result.
    """
    from irsim.util.util import geometry_transform

    geom = geometry_transform(lidar._original_geometry, lidar._state)
    lines, polygons = [], []
    for g in obstacle_geometries:
        bucket = polygons if g.geom_type in ("Polygon", "MultiPolygon") else lines
        bucket.append(g)
    for group in (lines, polygons):
        if group:
            obstacle = group[0] if len(group) == 1 else shapely.union_all(group)
            geom = geom.difference(obstacle)
    origin = shapely.points(lidar.lidar_origin[0, 0], lidar.lidar_origin[1, 0])
    parts = shapely.get_parts(geom)
    kept = parts[shapely.intersects(parts, origin)]
    ranges = np.full(lidar.number, lidar.range_max, dtype=float)
    lengths = shapely.length(kept)
    ranges[: len(lengths)] = lengths
    return ranges


def test_lidar_origin_uses_exact_world_geometry_coordinate(monkeypatch):
    """The reported origin and transformed beam start must be bit-identical."""
    import irsim.world.sensors.lidar2d as lidar_module

    point_transform = lidar_module.transform_point_with_state

    def slightly_shifted_origin(point, state):
        origin = point_transform(point, state)
        origin[0, 0] = np.nextafter(origin[0, 0], np.inf)
        return origin

    monkeypatch.setattr(
        lidar_module,
        "transform_point_with_state",
        slightly_shifted_origin,
    )
    lidar = Lidar2D(
        state=np.array([[1.2], [-0.7], [0.9]]),
        obj_id=1,
        number=31,
        angle_range=3.14,
        range_max=5.0,
        offset=[0.2, -0.1, 0.3],
    )

    world_geometry = lidar._world_geometry(lidar.state)
    geometry_origin = shapely.get_coordinates(world_geometry)[0]

    np.testing.assert_array_equal(lidar.lidar_origin[:2, 0], geometry_origin)


def test_cast_rays_only_requires_detected_objects():
    """The geometry algorithm does not require the environment or its STRtree."""
    lidar = Lidar2D(
        state=np.array([[0.0], [0.0], [0.0]]),
        obj_id=1,
        number=1,
        angle_range=0.0,
        range_max=5.0,
    )
    detected_object = _Obstacle(
        2,
        shapely.LineString([(2.0, -1.0), (2.0, 1.0)]),
        shape="linestring",
    )

    ranges, hit_object_indices, origin, directions = cast_rays(
        lidar._world_geometry(lidar.state),
        [detected_object],
        lidar.range_max,
    )

    np.testing.assert_allclose(ranges, [2.0], atol=1e-12)
    np.testing.assert_array_equal(hit_object_indices, [0])
    np.testing.assert_array_equal(origin, [0.0, 0.0])
    np.testing.assert_array_equal(directions, [[1.0, 0.0]])


def test_cast_ray_segments_detects_collinear_segment():
    """The analytic kernel must preserve GEOS collinear-overlap behavior."""
    ranges, hit = cast_ray_segments(
        origin=np.array([0.0, 0.0]),
        directions=np.array([[1.0, 0.0]]),
        seg_start=np.array([[2.0, 0.0]]),
        seg_end=np.array([[4.0, 0.0]]),
        max_range=5.0,
    )

    np.testing.assert_allclose(ranges, [2.0], atol=1e-12)
    np.testing.assert_array_equal(hit, [0])


def test_cast_ray_segments_chunks_large_segment_sets(monkeypatch):
    """Chunking bounds each matrix and preserves ranges and global hit indices."""
    random = np.random.default_rng(20260817)
    angles = np.linspace(-np.pi, np.pi, 181)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    segment_start = random.uniform(-8.0, 8.0, size=(5000, 2))
    segment_end = segment_start + random.uniform(-0.5, 0.5, size=(5000, 2))

    monkeypatch.setattr(ray_casting_2d, "SEGMENT_CHUNK_SIZE", len(segment_start))
    expected_ranges, expected_hits = cast_ray_segments(
        np.zeros(2), directions, segment_start, segment_end, 10.0
    )

    block_sizes = []
    original = ray_casting_2d._nonparallel_hit_distances

    def track_block_size(directions, segment_vectors, start_to_origin, max_range):
        block_sizes.append(len(segment_vectors))
        return original(directions, segment_vectors, start_to_origin, max_range)

    monkeypatch.setattr(ray_casting_2d, "SEGMENT_CHUNK_SIZE", 64)
    monkeypatch.setattr(
        ray_casting_2d,
        "_nonparallel_hit_distances",
        track_block_size,
    )
    ranges, hits = cast_ray_segments(
        np.zeros(2), directions, segment_start, segment_end, 10.0
    )

    assert len(block_sizes) > 1
    assert max(block_sizes) <= 64
    np.testing.assert_array_equal(ranges, expected_ranges)
    np.testing.assert_array_equal(hits, expected_hits)

    # Two equal nearest walls straddle a chunk boundary. The original full
    # argmin keeps the lower global segment index, and chunk merging must too.
    tie_start = np.tile([4.0, -1.0], (65, 1))
    tie_end = np.tile([4.0, 1.0], (65, 1))
    tie_start[63:] = [1.0, -1.0]
    tie_end[63:] = [1.0, 1.0]
    tie_ranges, tie_hits = cast_ray_segments(
        np.zeros(2), np.array([[1.0, 0.0]]), tie_start, tie_end, 10.0
    )
    np.testing.assert_array_equal(tie_ranges, [1.0])
    np.testing.assert_array_equal(tie_hits, [63])


def test_lidar_collinear_wall_matches_geos_difference():
    """A beam running along a wall keeps the same range as the GEOS overlay."""
    wall = shapely.LineString([(2.0, 0.0), (4.0, 0.0)])
    obstacle = _Obstacle(2, wall, shape="linestring")
    lidar = Lidar2D(
        state=np.array([[0.0], [0.0], [0.0]]),
        obj_id=1,
        number=1,
        angle_range=0.0,
        range_max=5.0,
    )
    lidar.parent = _Parent(_EnvParam([obstacle], STRtree([wall])))

    lidar.step(lidar.state)
    expected = _geos_difference_ranges(lidar, [wall])

    np.testing.assert_allclose(lidar.range_data, expected, atol=1e-12, rtol=0)
    assert lidar.range_data[0] == pytest.approx(2.0)


@pytest.mark.parametrize(
    "state",
    [
        [0.0, 0.0, 0.0],
        [-0.5, 0.5, 0.3],
        [0.25, -0.75, -0.4],
    ],
)
def test_lidar_map_contours_match_geos_per_beam(state):
    """Multi-segment map contours preserve every beam's legacy range index."""
    contours = [
        shapely.LineString([(2.0, -4.0), (2.0, 4.0), (4.0, 4.0)]),
        shapely.LineString([(-4.0, -3.0), (-1.0, -3.0), (-1.0, 3.0), (-4.0, 3.0)]),
        shapely.LineString([(4.0, -2.0), (5.0, 0.0), (4.0, 2.0)]),
    ]
    map_obj = _MapObject(2, contours)
    lidar = Lidar2D(
        state=np.c_[state],
        obj_id=1,
        number=181,
        angle_range=6.283185,
        range_max=8.0,
    )
    lidar.parent = _Parent(_EnvParam([map_obj], STRtree([map_obj.geometry])))

    lidar.step(lidar.state)
    expected = _geos_difference_ranges(lidar, contours)

    np.testing.assert_allclose(lidar.range_data, expected, atol=1e-9, rtol=0)


def test_lidar_scan_fields_velocity_and_geometry_are_beam_aligned():
    """Ranges, velocity, metadata, and beam geometry describe the same scan."""
    obstacle = _Obstacle(
        2,
        shapely.Point(2.0, 0.0).buffer(0.7),
        velocity_xy=(0.4, -0.2),
    )
    lidar = Lidar2D(
        state=np.array([[0.0], [0.0], [0.0]]),
        obj_id=1,
        number=91,
        angle_range=3.14,
        range_max=5.0,
        has_velocity=True,
    )
    lidar.parent = _Parent(_EnvParam([obstacle], STRtree([obstacle.geometry])))

    lidar.step(lidar.state)
    scan = lidar.get_scan()
    expected = _geos_difference_ranges(lidar, [obstacle.geometry])

    np.testing.assert_allclose(scan["ranges"], expected, atol=1e-9, rtol=0)
    assert tuple(scan) == (
        "angle_min",
        "angle_max",
        "angle_increment",
        "time_increment",
        "scan_time",
        "range_min",
        "range_max",
        "ranges",
        "intensities",
        "velocity",
    )
    assert scan["intensities"] is None
    assert scan["angle_min"] == lidar.angle_min
    assert scan["angle_max"] == lidar.angle_max
    assert scan["angle_increment"] == lidar.angle_inc
    assert scan["time_increment"] == lidar.time_inc
    assert scan["scan_time"] == lidar.scan_time
    assert scan["range_min"] == lidar.range_min
    assert scan["range_max"] == lidar.range_max

    hit = scan["ranges"] < lidar.range_max - 1e-9
    np.testing.assert_allclose(
        scan["velocity"][:, hit],
        np.broadcast_to(obstacle.velocity_xy, (2, int(hit.sum()))),
    )
    np.testing.assert_array_equal(scan["velocity"][:, ~hit], 0.0)

    parts = shapely.get_parts(lidar._geometry)
    np.testing.assert_allclose(shapely.length(parts), scan["ranges"], atol=1e-12)
    starts = shapely.get_coordinates(parts)[::2]
    np.testing.assert_allclose(starts, np.zeros((lidar.number, 2)), atol=1e-12)


def test_lidar_velocity_uses_explicit_hit_near_max_range():
    """A real hit near range_max carries velocity instead of looking like a miss."""
    wall = shapely.LineString([(4.99, -1.0), (4.99, 1.0)])
    obstacle = _Obstacle(
        2,
        wall,
        shape="linestring",
        velocity_xy=(0.7, -0.2),
    )
    lidar = Lidar2D(
        state=np.array([[0.0], [0.0], [0.0]]),
        obj_id=1,
        number=1,
        angle_range=0.0,
        range_max=5.0,
        has_velocity=True,
    )
    lidar.parent = _Parent(_EnvParam([obstacle], STRtree([wall])))

    lidar.step(lidar.state)

    assert lidar.range_data[0] == pytest.approx(4.99, abs=1e-12)
    np.testing.assert_allclose(lidar.velocity, obstacle.velocity_xy)


def _raycast_case(kind):
    """Build (obstacle objects, flat geometry list) for one obstacle type."""
    if kind == "rectangle":
        obs = _Obstacle(2, shapely.box(2.0, -1.0, 3.0, 1.0), shape="rectangle")
        return [obs], [obs.geometry]
    if kind == "circle":  # a circle is a buffer polygon
        obs = _Obstacle(2, shapely.Point(3.0, 0.0).buffer(0.8))
        return [obs], [obs.geometry]
    if kind == "linestring":
        obs = _Obstacle(
            2, shapely.LineString([(3.0, -2.0), (3.0, 2.0)]), shape="linestring"
        )
        return [obs], [obs.geometry]
    if kind == "polygon_with_hole":
        shell = shapely.box(2.0, -1.5, 4.0, 1.5)
        hole = shapely.box(2.7, -0.5, 3.3, 0.5)
        obs = _Obstacle(2, shell.difference(hole), shape="polygon")
        return [obs], [obs.geometry]
    if kind == "map":  # grid-map style: many short linestrings
        walls = [
            shapely.LineString([(3.0, y), (3.0, y + 0.1)])
            for y in np.arange(-1.0, 1.0, 0.1)
        ]
        return [_MapObject(2, walls)], list(walls)
    if kind == "compound":  # disjoint MultiPolygon (issue #350)
        compound = GeometryFactory.create_geometry(
            "compound",
            parts=[
                {
                    "name": "rectangle",
                    "length": 0.5,
                    "width": 1.0,
                    "pose": [2.0, 0.0, 0.0],
                },
                {
                    "name": "rectangle",
                    "length": 0.5,
                    "width": 1.0,
                    "pose": [4.0, 0.0, 0.0],
                },
            ],
        )
        obs = _Obstacle(2, compound.geometry, shape="compound")
        return [obs], [obs.geometry]
    if kind == "mixed":  # map + dynamic polygon in one scan (issue #302 layout)
        walls = [
            shapely.LineString([(-3.0, y), (-3.0, y + 0.1)])
            for y in np.arange(-1.0, 1.0, 0.1)
        ]
        circle = _Obstacle(3, shapely.Point(3.0, 0.0).buffer(0.7))
        return [_MapObject(2, walls), circle], [*walls, circle.geometry]
    raise ValueError(kind)


@pytest.mark.parametrize(
    "kind",
    [
        "rectangle",
        "circle",
        "linestring",
        "polygon_with_hole",
        "map",
        "compound",
        "mixed",
    ],
)
def test_lidar_raycast_matches_geos_difference(kind):
    """Ray-cast range_data reproduces the GEOS difference for every obstacle type.

    The sensor sits in free space so both approaches agree to floating-point
    precision (the analytic ray-cast uses the same obstacle edges GEOS does).
    """
    objects, geometries = _raycast_case(kind)
    env_param = _EnvParam(objects, STRtree([o.geometry for o in objects]))
    lidar = Lidar2D(
        state=np.array([[0.0], [0.0], [0.0]]),
        obj_id=1,
        number=180,
        angle_range=6.283185,
        range_max=8.0,
    )
    lidar.parent = _Parent(env_param)

    lidar.step(lidar.state)
    expected = _geos_difference_ranges(lidar, geometries)

    # Beam order is part of the LaserScan contract: ranges[i] belongs to
    # angle_min + i * angle_increment, so a permutation is not acceptable.
    np.testing.assert_allclose(lidar.range_data, expected, atol=1e-9, rtol=0)
    assert (lidar.range_data < lidar.range_max - 1e-6).any()  # obstacle was seen


def test_lidar_raycast_matches_geos_in_randomized_free_space():
    """Random poses, offsets, and mixed shapes retain every GEOS beam range."""
    random = np.random.default_rng(20260816)

    for _ in range(20):
        state = np.array(
            [
                [random.uniform(-2.0, 2.0)],
                [random.uniform(-2.0, 2.0)],
                [random.uniform(-np.pi, np.pi)],
            ]
        )
        offset = [
            random.uniform(-0.3, 0.3),
            random.uniform(-0.3, 0.3),
            random.uniform(-0.4, 0.4),
        ]
        number = int(random.choice([1, 31, 90, 181]))
        angle_range = (
            0.0 if number == 1 else float(random.choice([0.7, 3.14, 6.283185]))
        )
        lidar = Lidar2D(
            state=state,
            obj_id=1,
            number=number,
            angle_range=angle_range,
            range_max=8.0,
            offset=offset,
        )
        origin = lidar.lidar_origin[:2, 0]
        objects = []
        geometries = []
        for obj_id in range(2, 8):
            bearing = random.uniform(-np.pi, np.pi)
            distance = random.uniform(1.5, 6.0)
            center = origin + distance * np.array([np.cos(bearing), np.sin(bearing)])
            if obj_id % 3 == 0:
                geometry = shapely.Point(center).buffer(
                    random.uniform(0.15, 0.6), quad_segs=8
                )
                shape = "circle"
            elif obj_id % 3 == 1:
                tangent = np.array([-np.sin(bearing), np.cos(bearing)])
                half_length = random.uniform(0.2, 0.8)
                geometry = shapely.LineString(
                    [center - half_length * tangent, center + half_length * tangent]
                )
                shape = "linestring"
            else:
                half_size = random.uniform(0.15, 0.5, size=2)
                geometry = shapely.box(
                    center[0] - half_size[0],
                    center[1] - half_size[1],
                    center[0] + half_size[0],
                    center[1] + half_size[1],
                )
                shape = "rectangle"
            geometries.append(geometry)
            objects.append(_Obstacle(obj_id, geometry, shape=shape))

        lidar.parent = _Parent(
            _EnvParam(objects, STRtree([obj.geometry for obj in objects]))
        )

        lidar.step(lidar.state)
        expected = _geos_difference_ranges(lidar, geometries)

        np.testing.assert_allclose(lidar.range_data, expected, atol=1e-9, rtol=0)
        np.testing.assert_allclose(
            shapely.length(shapely.get_parts(lidar._geometry)),
            expected,
            atol=1e-9,
            rtol=0,
        )


# ---------------------------------------------------------------------------
# Lidar2D range noise and pointcloud conversion
# ---------------------------------------------------------------------------


class TestLidar2DNoise:
    """Lidar2D range noise via the ray-cast step path."""

    def test_step_noise_applied_and_reproducible(self):
        """Seeded range noise perturbs the scan and reproduces run-to-run."""
        from irsim.util.random import set_seed

        obstacle = _Obstacle(2, shapely.box(2.0, -3.0, 2.5, 3.0), shape="rectangle")
        env_param = _EnvParam([obstacle], STRtree([obstacle.geometry]))

        def run(noise):
            set_seed(9)
            lidar = Lidar2D(
                state=np.array([[0.0], [0.0], [0.0]]),
                obj_id=1,
                number=120,
                angle_range=6.283185,
                range_max=8.0,
                noise=noise,
                std=0.2,
            )
            lidar.parent = _Parent(env_param)
            lidar.step(lidar.state)
            return lidar.range_data.copy()

        noisy_a = run(True)
        noisy_b = run(True)
        clean = run(False)
        np.testing.assert_array_equal(noisy_a, noisy_b)  # seeded reproducible
        set_seed(9)
        expected = clean + rng.normal(0, 0.2, len(clean))
        np.testing.assert_allclose(noisy_a, expected, atol=1e-12, rtol=0)
        assert np.any(noisy_a != clean)  # noise actually changed the scan
        assert np.all(np.isfinite(noisy_a))


class TestLidar2DScanToPointcloud:
    """Lidar2D conversion from a scan to a 2D point cloud."""

    def test_scan_to_pointcloud_with_hits(self):
        """Beams shorter than range_max convert into 2D points."""
        state = np.array([[0.0], [0.0], [0.0]])
        lidar = Lidar2D(state=state, obj_id=1, number=10, range_max=5.0)
        lidar.range_data[:5] = 2.0  # half of the beams hit something
        result = lidar.scan_to_pointcloud()
        assert result is not None
        assert result.shape[0] == 2  # 2D points
        assert result.shape[1] == 5  # 5 hit points

    def test_scan_to_pointcloud_no_hits(self):
        """A scan with every beam at range_max converts to None."""
        state = np.array([[0.0], [0.0], [0.0]])
        lidar = Lidar2D(state=state, obj_id=1, number=10, range_max=5.0)
        lidar.range_data[:] = 5.0
        result = lidar.scan_to_pointcloud()
        assert result is None
