import numpy as np
import pytest

from irsim.lib.algorithm.kinematics import (
    ackermann_kinematics,
    differential_kinematics,
    omni_kinematics,
)
from irsim.lib.handler.kinematics_handler import (
    AckermannKinematics,
    DifferentialKinematics,
    KinematicsFactory,
    KinematicsHandler,
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
    assert np.allclose(next_state[2], np.pi)


def test_omni_kinematics():
    """Test omnidirectional robot kinematics"""
    # Test basic movement
    state = np.array([[0], [0]])  # x, y
    velocity = np.array([[1], [0]])  # vx, vy
    next_state = omni_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[1], [0]]))

    # Test diagonal movement
    velocity = np.array([[1], [1]])  # vx, vy
    next_state = omni_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[1], [1]]))

    # Test with noise
    next_state_noisy = omni_kinematics(state, velocity, 1.0, noise=True)
    assert next_state_noisy.shape == (2, 1)


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


class TestKinematicsRegistry:
    """Tests for the kinematics registry and @register_kinematics decorator."""

    def test_builtin_types_registered(self):
        """All three built-in kinematics types are in the registry."""
        assert "diff" in _kinematics_registry
        assert "omni" in _kinematics_registry
        assert "acker" in _kinematics_registry

    def test_registry_maps_to_correct_classes(self):
        assert _kinematics_registry["diff"] is DifferentialKinematics
        assert _kinematics_registry["omni"] is OmniKinematics
        assert _kinematics_registry["acker"] is AckermannKinematics

    def test_get_handler_class(self):
        assert KinematicsFactory.get_handler_class("diff") is DifferentialKinematics
        assert KinematicsFactory.get_handler_class("omni") is OmniKinematics
        assert KinematicsFactory.get_handler_class("acker") is AckermannKinematics
        assert KinematicsFactory.get_handler_class("nonexistent") is None

    def test_create_kinematics_uses_registry(self):
        handler = KinematicsFactory.create_kinematics(name="diff")
        assert isinstance(handler, DifferentialKinematics)

        handler = KinematicsFactory.create_kinematics(name="omni")
        assert isinstance(handler, OmniKinematics)

        handler = KinematicsFactory.create_kinematics(name="acker")
        assert isinstance(handler, AckermannKinematics)

    def test_register_custom_kinematics(self):
        """A custom kinematics type can be registered and created."""

        @register_kinematics("test_custom")
        class CustomKinematics(KinematicsHandler):
            action_dim = 3
            min_state_dim = 5
            default_state_dim = 5

            def step(self, state, velocity, step_time):
                return state

        assert _kinematics_registry["test_custom"] is CustomKinematics
        assert KinematicsFactory.get_handler_class("test_custom") is CustomKinematics

        # Clean up
        del _kinematics_registry["test_custom"]


# ---------------------------------------------------------------------------
# Handler metadata tests
# ---------------------------------------------------------------------------


class TestHandlerMetadata:
    """Tests for class-attribute metadata on handler subclasses."""

    def test_omni_metadata(self):
        assert OmniKinematics.action_dim == 2
        assert OmniKinematics.min_state_dim == 2
        assert OmniKinematics.default_state_dim == 3
        assert OmniKinematics.show_arrow is False

    def test_diff_metadata(self):
        assert DifferentialKinematics.action_dim == 2
        assert DifferentialKinematics.min_state_dim == 3
        assert DifferentialKinematics.default_state_dim == 3
        assert DifferentialKinematics.show_arrow is True
        assert DifferentialKinematics.default_color == "g"

    def test_acker_metadata(self):
        assert AckermannKinematics.action_dim == 2
        assert AckermannKinematics.min_state_dim == 4
        assert AckermannKinematics.default_state_dim == 4
        assert AckermannKinematics.default_color == "y"
        assert AckermannKinematics.default_description == "car_green.png"
        assert AckermannKinematics.show_arrow is True


# ---------------------------------------------------------------------------
# Polymorphic method tests
# ---------------------------------------------------------------------------


class TestVelocityToXY:
    """Tests for velocity_to_xy on each handler."""

    def test_omni(self):
        handler = OmniKinematics("omni", False, None)
        state = np.array([[1], [2], [0.5]])
        velocity = np.array([[3], [4]])
        result = handler.velocity_to_xy(state, velocity)
        np.testing.assert_array_equal(result, velocity)

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
