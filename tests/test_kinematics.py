import numpy as np
import pytest

from irsim.lib.algorithm.kinematics import (
    ackermann_kinematics,
    differential_kinematics,
    omni_angular_kinematics,
    omni_kinematics,
)
from irsim.lib.handler.kinematics_handler import (
    AckermannKinematics,
    DifferentialKinematics,
    KinematicsFactory,
    KinematicsHandler,
    OmniAngularKinematics,
    OmniKinematics,
    _kinematics_registry,
    register_kinematics,
)


def test_differential_kinematics():
    """Test differential drive robot kinematics"""
    # Test basic movement
    state = np.array([[0], [0], [0]])  # x, y, theta
    velocity = np.array([[1], [0]])  # linear, angular
    next_state = differential_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[1], [0], [0]]))

    # Test rotation
    velocity = np.array([[0], [1]])  # linear, angular
    next_state = differential_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[0], [0], [1]]))

    # Test with noise
    next_state_noisy = differential_kinematics(state, velocity, 1.0, noise=True)
    assert next_state_noisy.shape == (3, 1)

    # Test angle wrapping
    state = np.array([[0], [0], [np.pi]])
    velocity = np.array([[0], [np.pi]])
    next_state = differential_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state[2], 0)


def test_ackermann_kinematics():
    """Test Ackermann steering vehicle kinematics"""
    # Test steer mode
    state = np.array([[0], [0], [0], [0]])  # x, y, theta, steer_angle
    velocity = np.array([[1], [0]])  # linear, steer_angle
    next_state = ackermann_kinematics(state, velocity, 1.0, mode="steer")
    assert np.allclose(next_state[:3], np.array([[1], [0], [0]]))

    # Test angular mode
    velocity = np.array([[1], [0.1]])  # linear, angular
    next_state = ackermann_kinematics(state, velocity, 1.0, mode="angular")
    assert next_state.shape == (4, 1)

    # Test with noise
    next_state_noisy = ackermann_kinematics(state, velocity, 1.0, noise=True)
    assert next_state_noisy.shape == (4, 1)

    # Test angle wrapping
    state = np.array([[0], [0], [np.pi], [0]])
    velocity = np.array([[0], [np.pi]])
    next_state = ackermann_kinematics(state, velocity, 1.0)
    assert np.allclose(abs(next_state[2]), np.pi)


def test_omni_kinematics():
    """Test omnidirectional robot kinematics with body-frame velocity."""
    # Test forward movement at theta=0 (forward = +x in world)
    state = np.array([[0], [0], [0]])
    velocity = np.array([[1], [0]])  # forward, lateral
    next_state = omni_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[1], [0], [0]]))

    # Test forward movement at theta=pi/2 (forward = +y in world)
    state = np.array([[0], [0], [np.pi / 2]])
    velocity = np.array([[1], [0]])
    next_state = omni_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[0], [1], [np.pi / 2]]), atol=1e-10)

    # Test lateral movement at theta=0 (lateral = +y in world)
    state = np.array([[0], [0], [0]])
    velocity = np.array([[0], [1]])
    next_state = omni_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[0], [1], [0]]), atol=1e-10)

    # Test combined forward+lateral at theta=0
    state = np.array([[0], [0], [0]])
    velocity = np.array([[1], [1]])
    next_state = omni_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[1], [1], [0]]))

    # Test theta is preserved
    state = np.array([[0], [0], [1.5]])
    velocity = np.array([[1], [0]])
    next_state = omni_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state[2, 0], 1.5)

    # Test with noise
    next_state_noisy = omni_kinematics(state, velocity, 1.0, noise=True)
    assert next_state_noisy.shape == (3, 1)


def test_omni_angular_kinematics():
    """Test omnidirectional robot kinematics with body-frame velocity."""
    # Test forward movement at theta=0 (forward = +x in world)
    state = np.array([[0], [0], [0]])
    velocity = np.array([[1], [0], [0]])  # forward, lateral, yaw_rate
    next_state = omni_angular_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[1], [0], [0]]))

    # Test forward movement at theta=pi/2 (forward = +y in world)
    state = np.array([[0], [0], [np.pi / 2]])
    velocity = np.array([[1], [0], [0]])
    next_state = omni_angular_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[0], [1], [np.pi / 2]]), atol=1e-10)

    # Test lateral movement at theta=0 (lateral = -y in world... no, +y)
    state = np.array([[0], [0], [0]])
    velocity = np.array([[0], [1], [0]])  # pure lateral
    next_state = omni_angular_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[0], [1], [0]]), atol=1e-10)

    # Test pure rotation
    state = np.array([[0], [0], [0]])
    velocity = np.array([[0], [0], [1]])
    next_state = omni_angular_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[0], [0], [1]]))

    # Test combined forward and rotation
    state = np.array([[0], [0], [0]])
    velocity = np.array([[1], [0], [0.5]])
    next_state = omni_angular_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[1], [0], [0.5]]))

    # Test with noise
    next_state_noisy = omni_angular_kinematics(
        state, np.array([[1], [1], [0.5]]), 1.0, noise=True
    )
    assert next_state_noisy.shape == (3, 1)

    # Test angle wrapping
    state = np.array([[0], [0], [np.pi]])
    velocity = np.array([[0], [0], [np.pi]])
    next_state = omni_angular_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state[2], 0, atol=1e-10)

    # Test invalid noise parameters
    with pytest.raises(ValueError, match="alpha"):
        omni_angular_kinematics(
            np.array([[0], [0], [0]]),
            np.array([[1], [0], [0]]),
            1.0,
            noise=True,
            alpha=[0.03, 0.03],  # Too few parameters
        )


def test_kinematics_error_handling():
    """Test error handling in kinematics functions"""
    # Test invalid state dimensions
    state = np.array([[0], [0]])  # Too few dimensions
    velocity = np.array([[1], [0]])
    with pytest.raises(ValueError, match="shape"):
        differential_kinematics(state, velocity, 1.0)

    # Test invalid velocity dimensions
    state = np.array([[0], [0], [0]])
    velocity = np.array([[1]])  # Too few dimensions
    with pytest.raises(ValueError, match="shape"):
        differential_kinematics(state, velocity, 1.0)

    # Test invalid noise parameters
    state = np.array([[0], [0], [0]])
    velocity = np.array([[1], [0]])
    with pytest.raises(ValueError, match="alpha"):
        differential_kinematics(
            state, velocity, 1.0, noise=True, alpha=[0.03]
        )  # Too few parameters


# ---------------------------------------------------------------------------
# Registry tests
# ---------------------------------------------------------------------------


@pytest.fixture
def kinematics_registry():
    """Restore the kinematics registry after a test registers custom handlers."""
    saved = dict(_kinematics_registry)
    yield _kinematics_registry
    _kinematics_registry.clear()
    _kinematics_registry.update(saved)


class TestKinematicsRegistry:
    """Tests for the kinematics registry and @register_kinematics decorator."""

    def test_builtin_types_registered(self):
        """All four built-in kinematics types are in the registry."""
        assert "diff" in _kinematics_registry
        assert "omni" in _kinematics_registry
        assert "omni_angular" in _kinematics_registry
        assert "acker" in _kinematics_registry

    def test_registry_maps_to_correct_classes(self):
        assert _kinematics_registry["diff"] is DifferentialKinematics
        assert _kinematics_registry["omni"] is OmniKinematics
        assert _kinematics_registry["omni_angular"] is OmniAngularKinematics
        assert _kinematics_registry["acker"] is AckermannKinematics

    def test_get_handler_class(self):
        assert KinematicsFactory.get_handler_class("diff") is DifferentialKinematics
        assert KinematicsFactory.get_handler_class("omni") is OmniKinematics
        assert (
            KinematicsFactory.get_handler_class("omni_angular") is OmniAngularKinematics
        )
        assert KinematicsFactory.get_handler_class("acker") is AckermannKinematics
        assert KinematicsFactory.get_handler_class("nonexistent") is None

    def test_create_kinematics_uses_registry(self):
        handler = KinematicsFactory.create_kinematics(name="diff")
        assert isinstance(handler, DifferentialKinematics)

        handler = KinematicsFactory.create_kinematics(name="omni")
        assert isinstance(handler, OmniKinematics)

        handler = KinematicsFactory.create_kinematics(name="omni_angular")
        assert isinstance(handler, OmniAngularKinematics)

        handler = KinematicsFactory.create_kinematics(name="acker")
        assert isinstance(handler, AckermannKinematics)

    def test_create_kinematics_defaults_to_diff_only_when_name_is_missing(self):
        handler = KinematicsFactory.create_kinematics(name=None)
        assert isinstance(handler, DifferentialKinematics)
        assert handler.name == "diff"

        static_handler = KinematicsFactory.create_kinematics(name="static")
        assert isinstance(static_handler, DifferentialKinematics)
        assert static_handler.name == "static"

        with pytest.raises(NotImplementedError, match="not registered"):
            KinematicsFactory.create_kinematics(name="nonexistent")
        with pytest.raises(NotImplementedError, match="not registered"):
            KinematicsFactory.create_kinematics(name="")

    def test_register_custom_kinematics(self, kinematics_registry):
        """A custom kinematics type can be registered and created."""

        @register_kinematics("test_custom")
        class CustomKinematics(KinematicsHandler):
            action_dim = 3
            min_state_dim = 5
            state_dim = 5

            def velocity_to_xy(self, velocity, heading):
                """Simple stub: return a zero XY velocity vector."""
                return np.zeros(2, dtype=float)

            def compute_max_speed(self, velocity):
                """Simple stub: return a placeholder scalar speed."""
                return 0.0

            def compute_heading(self, state):
                """Simple stub: return a placeholder heading angle."""
                return 0.0

            def step(self, state, velocity, step_time):
                return state

        assert _kinematics_registry["test_custom"] is CustomKinematics
        assert KinematicsFactory.get_handler_class("test_custom") is CustomKinematics

        # Exercise the stub methods to ensure coverage
        handler = CustomKinematics("test_custom", False, None)
        np.testing.assert_array_equal(
            handler.velocity_to_xy(np.zeros(2), 0.0), np.zeros(2)
        )
        assert handler.compute_max_speed(np.zeros(2)) == 0.0
        assert handler.compute_heading(np.zeros(3)) == 0.0
        np.testing.assert_array_equal(
            handler.step(np.zeros(3), np.zeros(2), 0.1), np.zeros(3)
        )

        # Clean up
        del _kinematics_registry["test_custom"]


# ---------------------------------------------------------------------------
# Handler metadata tests
# ---------------------------------------------------------------------------


class TestHandlerMetadata:
    """Tests for class-attribute metadata on handler subclasses."""

    def test_omni_metadata(self):
        assert OmniKinematics.action_dim == 2
        assert OmniKinematics.min_state_dim == 3
        assert OmniKinematics.state_dim == 3
        assert OmniKinematics.show_arrow is False

    def test_diff_metadata(self):
        assert DifferentialKinematics.action_dim == 2
        assert DifferentialKinematics.min_state_dim == 3
        assert DifferentialKinematics.state_dim == 3
        assert DifferentialKinematics.show_arrow is True
        assert DifferentialKinematics.color == "g"

    def test_acker_metadata(self):
        assert AckermannKinematics.action_dim == 2
        assert AckermannKinematics.min_state_dim == 4
        assert AckermannKinematics.state_dim == 4
        assert AckermannKinematics.color == "y"
        assert AckermannKinematics.description == "car_green.png"
        assert AckermannKinematics.show_arrow is True

    def test_omni_angular_metadata(self):
        assert OmniAngularKinematics.action_dim == 3
        assert OmniAngularKinematics.min_state_dim == 3
        assert OmniAngularKinematics.state_dim == 3
        assert OmniAngularKinematics.show_arrow is True
        assert OmniAngularKinematics.color == "g"


# ---------------------------------------------------------------------------
# Polymorphic method tests
# ---------------------------------------------------------------------------


class TestVelocityToXY:
    """Tests for velocity_to_xy on each handler."""

    def test_omni(self):
        handler = OmniKinematics("omni", False, None)
        # At theta=0, forward=3 lateral=0 -> world vx=3, vy=0
        state = np.array([[0], [0], [0]])
        velocity = np.array([[3], [0]])
        result = handler.velocity_to_xy(state, velocity)
        np.testing.assert_allclose(result, np.array([[3], [0]]), atol=1e-10)

        # At theta=pi/2, forward=1 lateral=0 -> world vx=0, vy=1
        state = np.array([[0], [0], [np.pi / 2]])
        velocity = np.array([[1], [0]])
        result = handler.velocity_to_xy(state, velocity)
        np.testing.assert_allclose(result, np.array([[0], [1]]), atol=1e-10)

    def test_diff(self):
        handler = DifferentialKinematics("diff", False, None)
        state = np.array([[0], [0], [0]])  # theta=0
        velocity = np.array([[1], [0]])  # linear=1, angular=0
        result = handler.velocity_to_xy(state, velocity)
        np.testing.assert_allclose(result, np.array([[1], [0]]), atol=1e-10)

    def test_diff_with_angle(self):
        handler = DifferentialKinematics("diff", False, None)
        state = np.array([[0], [0], [np.pi / 2]])  # theta=pi/2
        velocity = np.array([[1], [0]])
        result = handler.velocity_to_xy(state, velocity)
        np.testing.assert_allclose(result, np.array([[0], [1]]), atol=1e-10)

    def test_acker(self):
        handler = AckermannKinematics("acker", False, None)
        state = np.array([[0], [0], [0], [0]])
        velocity = np.array([[2], [0]])
        result = handler.velocity_to_xy(state, velocity)
        np.testing.assert_allclose(result, np.array([[2], [0]]), atol=1e-10)

    def test_omni_angular(self):
        handler = OmniAngularKinematics("omni_angular", False, None)
        # At theta=0, forward=3 lateral=0 -> world vx=3, vy=0
        state = np.array([[0], [0], [0]])
        velocity = np.array([[3], [0], [1.0]])
        result = handler.velocity_to_xy(state, velocity)
        np.testing.assert_allclose(result, np.array([[3], [0]]), atol=1e-10)

        # At theta=pi/2, forward=1 lateral=0 -> world vx=0, vy=1
        state = np.array([[0], [0], [np.pi / 2]])
        velocity = np.array([[1], [0], [0.5]])
        result = handler.velocity_to_xy(state, velocity)
        np.testing.assert_allclose(result, np.array([[0], [1]]), atol=1e-10)


class TestComputeMaxSpeed:
    """Tests for compute_max_speed on each handler."""

    def test_omni(self):
        handler = OmniKinematics("omni", False, None)
        vel_max = np.array([[3], [4]])
        assert handler.compute_max_speed(vel_max) == pytest.approx(5.0)

    def test_diff(self):
        handler = DifferentialKinematics("diff", False, None)
        vel_max = np.array([[2], [1]])
        assert handler.compute_max_speed(vel_max) == pytest.approx(2.0)

    def test_acker(self):
        handler = AckermannKinematics("acker", False, None)
        vel_max = np.array([[3], [0.5]])
        assert handler.compute_max_speed(vel_max) == pytest.approx(3.0)

    def test_omni_angular(self):
        handler = OmniAngularKinematics("omni_angular", False, None)
        vel_max = np.array([[3], [4], [1]])
        # Only translational components count
        assert handler.compute_max_speed(vel_max) == pytest.approx(5.0)


class TestComputeHeading:
    """Tests for compute_heading on each handler."""

    def test_omni(self):
        handler = OmniKinematics("omni", False, None)
        state = np.array([[0], [0], [0]])
        velocity = np.array([[1], [1]])
        heading = handler.compute_heading(state, velocity)
        assert heading == pytest.approx(np.pi / 4)

    def test_diff(self):
        handler = DifferentialKinematics("diff", False, None)
        state = np.array([[0], [0], [1.5]])
        velocity = np.array([[1], [0]])
        assert handler.compute_heading(state, velocity) == pytest.approx(1.5)

    def test_acker(self):
        handler = AckermannKinematics("acker", False, None)
        state = np.array([[0], [0], [2.0], [0]])
        velocity = np.array([[1], [0]])
        assert handler.compute_heading(state, velocity) == pytest.approx(2.0)

    def test_omni_angular(self):
        handler = OmniAngularKinematics("omni_angular", False, None)
        state = np.array([[0], [0], [1.5]])
        velocity = np.array([[1], [1], [0.5]])
        # Heading comes from state[2], not velocity direction
        assert handler.compute_heading(state, velocity) == pytest.approx(1.5)


# ---------------------------------------------------------------------------
# Coverage: edge cases for base class and zero-shape branches
# ---------------------------------------------------------------------------


class TestRegistryDuplicateRaises:
    """Registering the same name for a different class raises ValueError."""

    def test_duplicate_registration(self):
        with pytest.raises(ValueError, match="already registered"):

            @register_kinematics("diff")
            class AnotherDiff(KinematicsHandler):
                def step(self, state, velocity, step_time):  # pragma: no cover
                    return state

                def velocity_to_xy(self, state, velocity):  # pragma: no cover
                    return np.zeros((2, 1))

                def compute_max_speed(self, vel_max):  # pragma: no cover
                    return 0.0

                def compute_heading(self, state, velocity):  # pragma: no cover
                    return 0.0


class TestBaseClassDefaults:
    """Base class provides differential-drive defaults for non-abstract methods."""

    def _make_bare_handler(self):
        """Create a minimal concrete subclass that only implements step()."""

        class BareHandler(KinematicsHandler):
            def step(self, state, velocity, step_time):  # pragma: no cover
                return state

        return BareHandler("bare", False, None)

    def test_velocity_to_xy_default(self):
        """Default velocity_to_xy projects linear speed through heading."""
        handler = self._make_bare_handler()
        state = np.array([[0.0], [0.0], [0.0]])  # heading = 0
        velocity = np.array([[2.0], [0.5]])
        result = handler.velocity_to_xy(state, velocity)
        np.testing.assert_allclose(result, np.array([[2.0], [0.0]]), atol=1e-10)

    def test_velocity_to_xy_zero_shape(self):
        """Default velocity_to_xy returns zeros for scalar velocity."""
        handler = self._make_bare_handler()
        state = np.array([[0.0], [0.0], [0.0]])
        velocity = np.float64(0.0)  # ndim == 0
        result = handler.velocity_to_xy(state, velocity)
        np.testing.assert_array_equal(result, np.zeros((2, 1)))

    def test_compute_max_speed_default(self):
        """Default compute_max_speed returns first component of vel_max."""
        handler = self._make_bare_handler()
        vel_max = np.array([[3.0], [4.0]])
        assert handler.compute_max_speed(vel_max) == pytest.approx(3.0)

    def test_compute_heading_fallback(self):
        """Base compute_heading returns state[2,0] when state has 3+ rows."""
        handler = self._make_bare_handler()
        state = np.array([[1], [2], [0.7]])
        velocity = np.zeros((2, 1))
        assert handler.compute_heading(state, velocity) == pytest.approx(0.7)

    def test_compute_heading_fallback_short_state(self):
        """Base compute_heading returns 0.0 when state has < 3 rows."""
        handler = self._make_bare_handler()
        state = np.array([[1], [2]])
        velocity = np.zeros((2, 1))
        assert handler.compute_heading(state, velocity) == 0.0


class TestVelocityToXYZeroShape:
    """Cover zero-shape velocity branches in diff/acker handlers."""

    def test_diff_zero_shape(self):
        handler = DifferentialKinematics("diff", False, None)
        state = np.array([[0], [0], [0]])
        velocity = np.float64(0.0)  # ndim == 0
        result = handler.velocity_to_xy(state, velocity)
        np.testing.assert_array_equal(result, np.zeros((2, 1)))

    def test_acker_zero_shape(self):
        handler = AckermannKinematics("acker", False, None)
        state = np.array([[0], [0], [0], [0]])
        velocity = np.float64(0.0)  # ndim == 0
        result = handler.velocity_to_xy(state, velocity)
        np.testing.assert_array_equal(result, np.zeros((2, 1)))


class TestKinematicsParameters:
    """Extra ``kinematics`` keys reach the handler's constructor."""

    def test_extra_parameters_are_forwarded(self, kinematics_registry):
        @register_kinematics("test_lag")
        class LagKinematics(DifferentialKinematics):
            def __init__(self, name, noise=False, alpha=None, tau=0.3):
                super().__init__(name, noise, alpha)
                self.tau = tau

        assert KinematicsFactory.create_kinematics(name="test_lag", tau=0.5).tau == 0.5
        acker = KinematicsFactory.create_kinematics(
            name="acker", mode="angular", wheelbase=2.0
        )
        assert (acker.mode, acker.wheelbase) == ("angular", 2.0)
        # the pre-existing positional order (name, noise, alpha, mode, wheelbase, role)
        positional = KinematicsFactory.create_kinematics(
            "acker", False, None, "angular", 2.0, "robot"
        )
        assert (positional.mode, positional.wheelbase) == ("angular", 2.0)
        with pytest.raises(TypeError, match="tau"):
            KinematicsFactory.create_kinematics(name="diff", tau=0.5)

        # a shape's wheelbase only concerns Ackermann handlers, and a YAML
        # ``wheelbase`` under ``kinematics`` overrides it
        assert (
            KinematicsFactory.create_kinematics(
                name="acker", shape_wheelbase=3.0
            ).wheelbase
            == 3.0
        )
        assert (
            KinematicsFactory.create_kinematics(
                name="acker", shape_wheelbase=3.0, wheelbase=2.5
            ).wheelbase
            == 2.5
        )
        assert KinematicsFactory.create_kinematics(name="acker").wheelbase == 1.0
        assert KinematicsFactory.create_kinematics(
            name="omni_angular", shape_wheelbase=3.0
        )

        # through an object: the kinematics block overrides the shape's wheelbase
        from irsim.world.object_base import ObjectBase

        car = ObjectBase(
            kinematics={"name": "acker", "wheelbase": 2.5},
            shape={"name": "rectangle", "length": 4.6, "width": 1.6, "wheelbase": 3},
        )
        assert (car.wheelbase, car.kf.wheelbase) == (3, 2.5)

        # a custom handler that is not an Ackermann subclass receives it as well
        @register_kinematics("test_bicycle")
        class BicycleLike(DifferentialKinematics):
            def __init__(self, name, noise=False, alpha=None, wheelbase=1.0):
                super().__init__(name, noise, alpha)
                self.wheelbase = wheelbase

        assert (
            KinematicsFactory.create_kinematics(
                name="test_bicycle", wheelbase=2.0
            ).wheelbase
            == 2.0
        )

        # the same path through an object's YAML ``kinematics`` block
        from irsim.world.object_base import ObjectBase

        robot = ObjectBase(
            kinematics={"name": "test_lag", "tau": 0.5},
            shape={"name": "circle", "radius": 0.2},
        )
        assert robot.kf.tau == 0.5
