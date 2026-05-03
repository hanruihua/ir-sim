import numpy as np
import pytest
import shapely

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
    ):
        self._id = obj_id
        self._geometry = geometry
        self._geometry_valid = True
        self.shape = shape
        self.unobstructed = unobstructed
        self._velocity_xy = np.c_[velocity_xy]

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
