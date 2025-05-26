import pytest
import numpy as np
from irsim.lib.algorithm.kinematics import (
    differential_kinematics,
    ackermann_kinematics,
    omni_kinematics
)

def test_differential_kinematics():
    """Test differential drive robot kinematics"""
    # Test basic movement
    state = np.array([[0], [0], [0]])  # x, y, theta
    velocity = np.array([[1], [0]])    # linear, angular
    next_state = differential_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[1], [0], [0]]))
    
    # Test rotation
    velocity = np.array([[0], [1]])    # linear, angular
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
    velocity = np.array([[1], [0]])         # linear, steer_angle
    next_state = ackermann_kinematics(state, velocity, 1.0, mode="steer")
    assert np.allclose(next_state[:3], np.array([[1], [0], [0]]))
    
    # Test angular mode
    velocity = np.array([[1], [0.1]])       # linear, angular
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
    state = np.array([[0], [0]])      # x, y
    velocity = np.array([[1], [0]])   # vx, vy
    next_state = omni_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[1], [0]]))
    
    # Test diagonal movement
    velocity = np.array([[1], [1]])   # vx, vy
    next_state = omni_kinematics(state, velocity, 1.0)
    assert np.allclose(next_state, np.array([[1], [1]]))
    
    # Test with noise
    next_state_noisy = omni_kinematics(state, velocity, 1.0, noise=True)
    assert next_state_noisy.shape == (2, 1)

def test_kinematics_error_handling():
    """Test error handling in kinematics functions"""
    # Test invalid state dimensions
    with pytest.raises(AssertionError):
        state = np.array([[0], [0]])  # Too few dimensions
        velocity = np.array([[1], [0]])
        differential_kinematics(state, velocity, 1.0)
    
    # Test invalid velocity dimensions
    with pytest.raises(AssertionError):
        state = np.array([[0], [0], [0]])
        velocity = np.array([[1]])    # Too few dimensions
        differential_kinematics(state, velocity, 1.0)
    
    # Test invalid noise parameters
    with pytest.raises(AssertionError):
        state = np.array([[0], [0], [0]])
        velocity = np.array([[1], [0]])
        differential_kinematics(state, velocity, 1.0, noise=True, alpha=[0.03])  # Too few parameters 

if __name__ == "__main__":
    test_differential_kinematics()
    test_ackermann_kinematics()
    test_omni_kinematics()
    test_kinematics_error_handling()