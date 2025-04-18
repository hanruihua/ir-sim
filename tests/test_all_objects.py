import pytest
import irsim
from irsim.util.util import time_it2, file_check, WrapToRegion, convert_list_length_dict, is_list_not_list_of_lists, is_list_of_lists, get_transform, get_affine_transform, distance
from unittest.mock import Mock
from pynput import keyboard
import matplotlib.pyplot as plt
import numpy as np
from math import pi
import time

@pytest.fixture(autouse=True)
def setup_teardown():
    """Setup and cleanup before and after each test"""
    plt.close('all')
    yield
    plt.close('all')

def test_collision_avoidance():
    """Test collision avoidance functionality and other util functions"""
    env = irsim.make("test_collision_avoidance.yaml", save_ani=False, full=True, display=False)
    
    env.robot.info.add_property('test', 1)
    env.robot.get_obstacle_info().add_property('test', 2)
    print(env.robot)
    print(env.robot_list[0] == env.robot_list[1])
    print(env.robot_list[0] == env.robot_list)
    print(hash(env.robot))
    print(env.robot.abbr)
    print(env.robot.velocity)
    print(env.robot.arrive)
    print(env.robot.collision)
    print(env.robot.heading)
    print(env.robot_list[1].heading)
    print(env.robot.orientation)
    print(env.step_time)
    print(env.robot_number)

    env.robot.set_velocity([1, 1])
    env.robot.set_velocity([1, 1], init=True)
    env.robot.set_velocity(np.array([1, 1]).reshape(2, 1), init=True)
    env.robot.set_state([1, 1, 0])
    env.robot.set_state(np.array([1, 1, 0]).reshape(3, 1))
    
    obs = env.create_obstacle(shape={'name':'polygon', 'vertices': [[6, 5], [7, 5], [7, 6], [6, 6]]}) 
    env.add_object(obs)
    env.add_objects([obs])
    env.delete_object(obs.id)

    env.get_robot_state()
    env.get_lidar_scan()
    env.get_lidar_offset()
    env.get_robot_info()
    
    file_check('123.yaml')
    file_check('123.yaml', root_path='.')
    WrapToRegion(4, [-pi, pi])
    WrapToRegion(-4, [-pi, pi])
    convert_list_length_dict([1, 2, 3], 1)
    is_list_not_list_of_lists([1, 2, 3])
    is_list_of_lists([[1, 2, 3]])
    get_transform(np.array([1, 2]).reshape(2, 1))
    get_affine_transform(np.array([1, 2, 3]).reshape(3, 1))
    distance(np.array([1, 2]).reshape(2, 1), np.array([3, 4]).reshape(2, 1))

    for i in range(20):
        env.step()
        env.render(0.01)
        env.draw_trajectory(env.robot.trajectory, show_direction=True)
        if env.done():
            break

    env.robot.remove()
    env.end()
    assert True  # Add specific assertions

def test_polygon_and_lidar():
    """Test polygon shape and lidar functionality"""
    env = irsim.make('test_all_objects.yaml', display=False)
    
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

def test_animation_saving():
    """Test animation saving functionality"""
    env = irsim.make('test_render.yaml', save_ani=True, display=False)
    
    for i in range(20):
        env.step()
        env.render(0.01)
    env.end(ani_name='test_animation')
    assert True  # Add file existence check

def test_collision_world():
    """Test collision world"""
    env = irsim.make('test_collision_world.yaml', save_ani=False, display=False)
    
    for i in range(4):
        env.step()
        env.render(0.01)
    env.end()
    assert True  # Add collision detection assertions

def test_multi_objects():
    """Test multi-object scenario"""
    env = irsim.make('test_multi_objects_world.yaml', save_ani=False, display=False)
    env.robot.set_goal([5, 10, 0])
    env.random_obstacle_position()
    
    for i in range(5):
        env.step()
        env.render(0.01)
    
    action_list = [[1, 0], [2, 0]]
    action_id_list = [2, 3]
    env.step(action_list, action_id_list)

    env.end()
    assert True  # Add multi-object related assertions

def test_grid_map():
    """Test grid map"""
    env = irsim.make('test_grid_map.yaml', save_ani=False, display=False)
    env.robot.set_laser_color([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])

    for i in range(6):
        env.step()
        env.render(0.01)
    
    gh = env.robot.get_init_Gh()
    env.end()
    assert gh is not None

def test_keyboard_control():
    """Test keyboard control"""
    env = irsim.make('test_keyboard_control.yaml', save_ani=False, display=False)
    key_list = ['w', 'a', 's', 'd', 'q', 'e', 'z', 'c', 'r']
    mock_keys = [Mock(spec=keyboard.Key, char=c) for c in key_list]
    
    for i in range(3):
        for mock_key in mock_keys:
            env._on_press(mock_key)
            env._on_release(mock_key)
        env.step()
        env.render(0.01)
    env.end()
    assert True  # Add keyboard control related assertions

def test_custom_behavior():
    """Test custom behavior"""
    env = irsim.make('custom_behavior.yaml', display=False)
    env.load_behavior("custom_behavior_methods")
    
    for i in range(10):
        env.step()
        env.render(0.01)
    env.end()
    assert True  # Add behavior related assertions

def test_fov_detection():
    """Test field of view detection"""
    env = irsim.make('test_fov_world.yaml', save_ani=False, display=False)
    env.obstacle_list[0].get_fov_detected_objects()

    for i in range(30):
        detected = [obs.fov_detect_object(env.robot) for obs in env.obstacle_list]
        env.step()
        env.render(0.01)
    env.end()
    assert isinstance(detected, list)

def test_3d_projection():
    """Test 3D projection"""
    env = irsim.make('test_multi_objects_world.yaml', save_ani=False, display=False, projection='3d')
    env.random_obstacle_position(ids=[3, 4, 5, 6, 7], non_overlapping=True)
    
    for i in range(5):
        env.step()
        env.render(0.01)
    env.end()
    assert True  # Add 3D related assertions

def test_time_it2_decorator():
    """Test the time_it2 decorator functionality"""
    class TestClass:
        def __init__(self, time_print=True):
            self.time_print = time_print
        
        @time_it2(name="TestFunction")
        def test_method(self):
            time.sleep(0.1)  # Simulate time-consuming operation
            return "success"
    
    # Test when time_print is True
    test_obj = TestClass(time_print=True)
    

if __name__ == "__main__":
    pytest.main(["--cov=.", "--cov-report", "html", "-v", __file__])
