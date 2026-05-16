import matplotlib.pyplot as plt
import numpy as np
import pytest
import shapely
from shapely import STRtree

from irsim.util.random import set_seed
from irsim.world.object_factory import ObjectFactory
from irsim.world.sensors.fmcw_lidar2d import FMCWLidar2D
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

    def test_find_nearest_hit_skips_invalid_candidates(self):
        """``_find_nearest_hit`` must skip self, invalid, and unobstructed objects."""
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

    def test_iter_intersection_points_handles_geometry_types(self):
        """Helper should yield points for every Shapely geometry kind it knows."""
        sensor = FMCWLidar2D(
            state=np.array([[0.0], [0.0], [0.0]]),
            obj_id=1,
            number=1,
            angle_range=0.0,
            range_max=5.0,
        )

        empty = list(sensor._iter_intersection_points(shapely.Point()))
        assert empty == []

        single = list(sensor._iter_intersection_points(shapely.Point(1.0, 2.0)))
        assert len(single) == 1
        assert single[0].geom_type == "Point"

        multipoint = shapely.MultiPoint([(0.0, 0.0), (1.0, 1.0)])
        assert len(list(sensor._iter_intersection_points(multipoint))) == 2

        line = shapely.LineString([(0.0, 0.0), (1.0, 0.0)])
        assert len(list(sensor._iter_intersection_points(line))) == 2

        ring = shapely.LinearRing([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)])
        assert len(list(sensor._iter_intersection_points(ring))) == 4

        multiline = shapely.MultiLineString(
            [[(0.0, 0.0), (1.0, 0.0)], [(2.0, 0.0), (3.0, 0.0)]]
        )
        assert len(list(sensor._iter_intersection_points(multiline))) == 4

        collection = shapely.GeometryCollection(
            [shapely.Point(0.0, 0.0), shapely.Point(1.0, 1.0)]
        )
        assert len(list(sensor._iter_intersection_points(collection))) == 2

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
