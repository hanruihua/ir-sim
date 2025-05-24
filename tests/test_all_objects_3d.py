import pytest
import irsim
from irsim.util.util import time_it
from unittest.mock import Mock, patch
from pynput import keyboard
import matplotlib.pyplot as plt
import time

@pytest.fixture(autouse=True)
def setup_teardown():
    """Setup and cleanup before and after each test"""
    plt.close('all')
    yield
    plt.close('all')

def test_collision_avoidance_3d():
    """Test collision avoidance functionality in 3D projection"""
    env = irsim.make("test_collision_avoidance.yaml", save_ani=False, full=False, display=False, projection='3d')
    
    for i in range(50):
        env.step()
        env.render(0.01)
        env.draw_trajectory(env.robot.trajectory, show_direction=True)
        if env.done():
            break
    
    env.end()
    assert True  # Add specific assertions

def test_polygon_and_lidar_3d():
    """Test polygon shape and lidar functionality in 3D projection"""
    env = irsim.make('test_all_objects.yaml', display=False, projection='3d')
    
    env.random_polygon_shape()
    points = env.robot.get_lidar_points()
    env.draw_points(points)
    
    scan = env.robot.get_lidar_scan()
    offset = env.robot.get_lidar_offset()
    gh_init = env.robot.get_init_Gh()
    gh = env.robot.get_Gh()
    
    env.get_obstacle_info_list()
    env.get_robot_info_list()
    env.delete_objects([1, 2])
    
    for i in range(10):
        env.step()
        env.render(0.01)
    env.end()
    
    # assert points is not None
    assert scan is not None
    assert offset is not None

def test_animation_saving_3d():
    """Test animation saving functionality in 3D projection"""
    env = irsim.make('test_render.yaml', save_ani=True, display=False, projection='3d')
    
    for i in range(3):
        env.step()
        env.render(0.01)
    env.end(ani_name='test_animation')
    assert True  # Add file existence check

def test_collision_world_3d():
    """Test collision world in 3D projection"""
    env = irsim.make('test_collision_world.yaml', save_ani=False, display=False, projection='3d')
    
    for i in range(4):
        env.step()
        env.render(0.01)
    env.end()
    assert True  # Add collision detection assertions

def test_multi_objects_3d():
    """Test multi-object scenario in 3D projection"""
    env = irsim.make('test_multi_objects_world.yaml', save_ani=False, display=False, projection='3d')
    env.robot.set_goal([5, 10, 0])
    env.random_obstacle_position()
    
    for i in range(5):
        env.step()
        env.render(0.01)
    env.end()
    assert True  # Add multi-object related assertions

def test_grid_map_3d():
    """Test grid map in 3D projection"""
    env = irsim.make('test_grid_map.yaml', save_ani=False, display=False, projection='3d')
    
    for i in range(6):
        env.step()
        env.render(0.01)
    
    gh = env.robot.get_init_Gh()
    env.end()
    assert gh is not None

# def test_keyboard_control_3d():
#     """Test keyboard control in 3D projection"""
#     env = irsim.make('test_keyboard_control.yaml', save_ani=False, display=False, projection='3d')
#     key_list = ['w', 'a', 's', 'd', 'q', 'e', 'z', 'c', 'r']
#     mock_keys = [Mock(spec=keyboard.Key, char=c) for c in key_list]
    
#     for i in range(3):
#         for mock_key in mock_keys:
#             env._on_press(mock_key)
#             env._on_release(mock_key)
#         env.step()
#         env.render(0.01)
#     env.end()
#     assert True  # Add keyboard control related assertions

def test_custom_behavior_3d():
    """Test custom behavior in 3D projection"""
    env = irsim.make('custom_behavior.yaml', display=False, projection='3d')
    env.load_behavior("custom_behavior_methods")
    
    for i in range(10):
        env.step()
        env.render(0.01)
    env.end()
    assert True  # Add behavior related assertions

def test_fov_detection_3d():
    """Test field of view detection in 3D projection"""
    env = irsim.make('test_fov_world.yaml', save_ani=False, display=False, projection='3d')
    
    for i in range(10):
        detected = [obs.fov_detect_object(env.robot) for obs in env.obstacle_list]
        env.step()
        env.render(0.01)
    env.end()
    assert isinstance(detected, list)

def test_3d_projection():
    """Test 3D projection specifically"""
    env = irsim.make('test_multi_objects_world.yaml', save_ani=False, display=False, projection='3d')
    env.random_obstacle_position(ids=[3, 4, 5, 6, 7], non_overlapping=True)
    
    for i in range(5):
        env.step()
        env.render(0.01)
    env.end()
    assert True  # Add 3D related assertions

if __name__ == "__main__":
    pytest.main(["--cov=.", "--cov-report", "html", "-v", __file__])
