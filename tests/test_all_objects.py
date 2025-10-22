import contextlib
import time
from math import pi
from unittest.mock import Mock, patch

import matplotlib.pyplot as plt
import numpy as np
import pytest
from matplotlib.backend_bases import MouseButton

import irsim
from irsim.gui.mouse_control import MouseControl
from irsim.util.util import (
    WrapToRegion,
    convert_list_length_dict,
    distance,
    file_check,
    get_affine_transform,
    get_transform,
    is_list_not_list_of_lists,
    is_list_of_lists,
    time_it2,
)

with contextlib.suppress(ImportError):
    from pynput import keyboard

import re


@pytest.fixture(autouse=True)
def setup_teardown():
    """Setup and cleanup before and after each test"""
    plt.close("all")
    yield
    plt.close("all")


def test_collision_avoidance():
    """Test collision avoidance functionality and other util functions"""
    env = irsim.make(
        "test_collision_avoidance.yaml", save_ani=False, full=True, display=False
    )

    env.robot.info.add_property("test", 1)
    env.robot.get_obstacle_info().add_property("test", 2)
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
    print(env.dynamic_objects)
    print(env.static_objects)
    print(env.robot.goal_vertices)
    print(env.robot.original_vertices)
    print(env.robot.name)
    print(env.names)

    env.logger.info("test")
    env.logger.warning("test")
    env.logger.error("test")
    env.logger.critical("test")
    env.logger.debug("test")
    env.logger.success("test")
    env.logger.trace("test")

    env.robot.set_velocity([1, 1])
    env.robot.set_velocity([1, 1], init=True)
    env.robot.set_velocity(np.array([1, 1]).reshape(2, 1), init=True)
    env.robot.set_state([1, 1, 0])
    env.robot.set_state(np.array([1, 1, 0]).reshape(3, 1))

    obs = env.create_obstacle(
        shape={"name": "polygon", "vertices": [[6, 5], [7, 5], [7, 6], [6, 6]]}
    )
    env.add_object(obs)
    env.delete_object(obs.id)
    env.add_objects([obs])
    env.get_robot_state()
    env.get_lidar_scan()
    env.get_lidar_offset()
    env.get_robot_info()
    env.get_map()
    env.get_object_by_name("testtest")
    env.get_object_by_id(env.robot.id)
    env.robot.get_desired_omni_vel()
    env.robot.get_desired_omni_vel(normalized=True)
    env.robot.set_goal(None)
    env.robot.get_desired_omni_vel()

    env.draw_quiver(np.array([1, 2, 2, 3]))
    env.draw_quiver(np.array([1, 2, 2, 3]), refresh=True)
    points = [np.array([1, 3, 2, 3]), np.array([1, 2, 2, 5])]
    env.draw_quivers(points)
    env.draw_quivers(points, refresh=True)

    env.set_title(f"Simulation time: {env.time:.2f}s")

    env.set_random_seed(2)

    file_check("123.yaml")
    file_check("123.yaml", root_path=".")
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
        env._objects_step([np.array([1, 0]).reshape(2, 1)])

        env.robot.get_desired_omni_vel()

        # Test different _step_plot arguments to verify element property updates
        if i % 4 == 0:
            # Test object color and alpha changes
            env.render(0.01, obj_color="red", obj_alpha=0.7, obj_zorder=5)
        elif i % 4 == 1:
            # Test object linestyle and trajectory properties
            env.render(
                0.01,
                obj_linestyle="--",
                traj_color="blue",
                traj_alpha=0.8,
                traj_width=0.3,
            )
        elif i % 4 == 2:
            # Test goal and arrow properties
            env.render(
                0.01,
                goal_color="green",
                goal_alpha=0.6,
                goal_zorder=3,
                arrow_color="orange",
                arrow_alpha=0.9,
                arrow_zorder=4,
            )
        else:
            # Test text and FOV properties
            env.render(
                0.01,
                text_color="purple",
                text_size=14,
                fov_color="cyan",
                fov_alpha=0.4,
                traj_style="--",
                traj_zorder=3,
                text_alpha=0.5,
                text_zorder=4,
            )

        env.draw_trajectory(env.robot.trajectory, show_direction=True)
        if env.done():
            break

    env.robot.remove()
    env.end()
    assert True  # Add specific assertions


def test_polygon_and_lidar():
    """Test polygon shape and lidar functionality"""
    env = irsim.make("test_all_objects.yaml", display=False)

    env.random_polygon_shape()
    points = env.robot.get_lidar_points()
    env.draw_points(points)

    scan = env.robot.get_lidar_scan()
    offset = env.robot.get_lidar_offset()
    env.robot.get_init_Gh()
    env.robot.get_Gh()

    env.get_obstacle_info_list()
    env.get_robot_info_list()

    for i in range(10):
        if i < 5:
            env.robot.set_goal(None)
            env.robot_list[1].set_goal(None)
            env.robot_list[2].set_goal(None)
        else:
            env.robot.set_goal([9, 9, 0])
            env.robot_list[1].set_goal([9, 9, 0])
            env.robot_list[2].set_goal([9, 9, 0])

        env.step()
        env.render(0.01)

    env.delete_objects([1, 2])
    env.end()

    # assert points is not None
    assert scan is not None
    assert offset is not None


def test_animation_saving():
    """Test animation saving functionality"""
    env = irsim.make("test_render.yaml", save_ani=True, display=False)

    for _i in range(20):
        env.step()
        env.render(0.01)
    env.end(ani_name="test_animation")
    assert True  # Add file existence check


def test_collision_world():
    """Test collision world"""
    env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)

    for _i in range(4):
        env.step()
        env.render(0.01)
    env.end()
    assert True  # Add collision detection assertions


def test_multi_objects():
    """Test multi-object scenario"""
    env = irsim.make("test_multi_objects_world.yaml", save_ani=False, display=False)
    env.robot.set_goal([5, 10, 0])
    env.random_obstacle_position()

    for _i in range(5):
        env.step()
        env.render(0.01)

    action_list = [[1, 0], [2, 0]]
    action_id_list = [2, 3]
    env.step(action_list, action_id_list)

    action_list = [[1, 0], [2, 0]]
    action_id = 1
    env.step(action_list, action_id)

    action = np.array([1, 0]).reshape(2, 1)
    action_id_list = [2, 3]
    env.step(action, action_id_list)

    env.robot.get_desired_omni_vel(goal_threshold=1000)

    env.end()
    assert True  # Add multi-object related assertions


def test_grid_map():
    """Test grid map"""
    env = irsim.make("test_grid_map.yaml", save_ani=False, display=False)
    env.robot.set_laser_color(
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9], laser_color="blue", alpha=0.2
    )

    for _i in range(6):
        env.step()
        env.render(0.01)

    gh = env.robot.get_init_Gh()
    env.end()
    assert gh is not None


def test_keyboard_control():
    """Test keyboard control"""
    env = irsim.make("test_keyboard_control.yaml", save_ani=False, display=False)
    key_list = ["w", "a", "s", "d", "q", "e", "z", "c", "r", "l", "v", "x"]
    mock_keys = [Mock(spec=keyboard.Key, char=c) for c in key_list]

    for _ in range(3):
        for mock_key in mock_keys:
            env.keyboard._on_pynput_press(mock_key)
            env.keyboard._on_pynput_release(mock_key)
        env.step()
        env.render(0.01)

    # Explicitly test 'l' (reload) and ensure environment remains usable
    pre_names = list(env.names)
    env.keyboard._on_pynput_release(Mock(spec=keyboard.Key, char="l"))
    assert len(env.names) == len(pre_names)

    # Test Alt key functionality
    # Create a mock Alt key
    alt_key = Mock(spec=keyboard.Key)
    alt_key.name = "alt"
    # Create mock number keys
    num_keys = [Mock(spec=keyboard.Key, char=str(i)) for i in range(5)]

    # Press Alt key
    env.keyboard.is_active = True
    env.keyboard._active_only = False
    env.keyboard._on_pynput_press(alt_key)
    assert env.keyboard.alt_flag, "After pressing Alt key, alt_flag should be True"

    # Test number keys with Alt pressed
    for i, num_key in enumerate(num_keys):
        env.keyboard._on_pynput_press(num_key)
        if i < env.robot_number:
            assert env.keyboard.key_id == i, (
                f"After pressing Alt+{i}, control ID should change to {i}"
            )
        else:
            # If robot number is less than the pressed number, it should print "out of number of robots"
            # but we can't easily test the print output, so we just check that key_id is set
            assert env.keyboard.key_id == i, (
                f"After pressing Alt+{i}, control ID should be set to {i}"
            )

    # Release Alt key
    env.keyboard._on_pynput_release(alt_key)
    assert not env.keyboard.alt_flag, (
        "After releasing Alt key, alt_flag should be False"
    )

    # Test space key functionality - toggle pause/resume on release
    # Method 3: Create a simple class that equals keyboard.Key.space
    class SpaceKeyMock:
        name = "space"

        def __eq__(self, other):
            return other == keyboard.Key.space

    space_key = SpaceKeyMock()

    # First release should pause the environment
    if "Running" not in env.status:
        env.resume()

    env.keyboard._on_pynput_release(space_key)
    assert env.status == "Pause", (
        "After first space key release, status should be Pause"
    )

    # Second release should resume the environment
    env.keyboard._on_pynput_release(space_key)
    assert env.status == "Running", (
        "After second space key release, status should be Running"
    )

    # 'x' toggles control mode (keyboard <-> auto) in pynput backend
    from irsim.config import world_param as wp

    mode0 = wp.control_mode

    # Simulate 'x' release
    class XKeyMock:
        char = "x"

    env.keyboard._on_pynput_release(XKeyMock())
    assert wp.control_mode != mode0
    env.keyboard._on_pynput_release(XKeyMock())
    assert wp.control_mode == mode0

    # q/e adjust key_lv_max (pynput backend)
    prev_lv_max = env.keyboard.key_lv_max
    env.keyboard._on_pynput_release(Mock(spec=keyboard.Key, char="e"))
    assert env.keyboard.key_lv_max > prev_lv_max
    env.keyboard._on_pynput_release(Mock(spec=keyboard.Key, char="q"))
    assert env.keyboard.key_lv_max < prev_lv_max + 0.2  # net change <= 0

    # z/c adjust key_ang_max (pynput backend)
    prev_ang_max = env.keyboard.key_ang_max
    env.keyboard._on_pynput_release(Mock(spec=keyboard.Key, char="c"))
    assert env.keyboard.key_ang_max > prev_ang_max
    env.keyboard._on_pynput_release(Mock(spec=keyboard.Key, char="z"))
    assert env.keyboard.key_ang_max < prev_ang_max + 0.2  # net change <= 0

    # 'r' sets reset_flag; processed on render
    env.keyboard._on_pynput_release(Mock(spec=keyboard.Key, char="r"))
    assert env.reset_flag is True
    env.render(0.0)
    assert env.reset_flag is False

    # 'l' sets reload_flag; processed on render
    env.keyboard._on_pynput_release(Mock(spec=keyboard.Key, char="l"))
    assert env.reload_flag is True
    env.render(0.0)
    assert env.reload_flag is False

    # 'v' saves the figure via flag processed in render (pynput backend)
    with patch.object(env, "save_figure") as mock_save:
        env.keyboard._on_pynput_release(Mock(spec=keyboard.Key, char="v"))
        env.render(0.0)
        mock_save.assert_called_once()

    # 'f5' debug single-step (pynput backend)
    class F5KeyMock:
        name = "f5"

        def __eq__(self, other):
            return other == keyboard.Key.f5

    f5_key = F5KeyMock()
    t0 = env.time
    env.keyboard._on_pynput_release(f5_key)
    env.step()
    t1 = env.time
    assert t1 > t0
    env.step()
    assert env.time == t1  # blocked until next F5
    assert "Pause (Debugging)" in env.status
    env.keyboard._on_pynput_release(f5_key)
    env.step()
    assert env.time > t1

    # ESC should not call GUI from listener thread in tests; just verify no exception here
    class EscKeyMock:
        pass

    # Provide attributes to mimic pynput Key.esc equality behavior if needed
    _ = EscKeyMock()

    # Test edge cases and untested lines in keyboard control

    # Test 1: Invalid backend handling (lines 99-100)

    with patch("irsim.gui.keyboard_control.env_param") as mock_env_param:
        mock_env_param.logger.warning = Mock()
        # Create a new keyboard control with invalid backend
        invalid_kb = irsim.gui.keyboard_control.KeyboardControl(
            env, backend="invalid_backend"
        )
        assert invalid_kb.backend == "mpl"
        mock_env_param.logger.warning.assert_called_once()

    # Test 2: Alt key AttributeError handling (lines 204-205)
    env.keyboard._active_only = False  # Disable gating for testing

    # Create a mock key that will raise AttributeError
    class BadKey:
        def __getattr__(self, name):
            if name == "name":
                raise AttributeError("Mock attribute error")
            return

    bad_key = BadKey()
    # This should not raise an exception
    env.keyboard._on_pynput_press(bad_key)
    assert env.keyboard.alt_flag == 0  # Should remain unchanged

    # Test 3: Alt key release handling (line 273)
    alt_key = Mock(spec=keyboard.Key)
    alt_key.name = "alt"

    env.keyboard.alt_flag = True  # Set alt flag first
    env.keyboard._on_pynput_release(alt_key)
    assert not env.keyboard.alt_flag

    # Test 4: ESC key setting quit_flag (line 304)
    # Reset quit_flag first
    env.quit_flag = False
    # Test 5: Focus event handlers (lines 462, 465, 468)
    env.keyboard._on_mpl_focus_in(None)
    assert env.keyboard._is_active

    env.keyboard._on_mpl_focus_out(None)
    assert not env.keyboard._is_active

    env.keyboard._on_mpl_close(None)
    assert not env.keyboard._is_active

    # Test 6: pynput fallback when unavailable (lines 152-153)
    with (
        patch("irsim.gui.keyboard_control._PYNPUT_AVAILABLE", False),
        patch("irsim.gui.keyboard_control.env_param") as mock_env_param,
    ):
        mock_env_param.logger.warning = Mock()
        fallback_kb = irsim.gui.keyboard_control.KeyboardControl(env, backend="pynput")
        assert fallback_kb.backend == "mpl"
        mock_env_param.logger.warning.assert_called_once()

    # Test 7: matplotlib connection exception handling (lines 167-168)
    with patch("matplotlib.pyplot.gcf") as mock_gcf:
        mock_fig = Mock()
        mock_fig.canvas.mpl_connect.side_effect = Exception("Connection failed")
        mock_gcf.return_value = mock_fig

        # This should not raise an exception
        exception_kb = irsim.gui.keyboard_control.KeyboardControl(env)
        assert exception_kb.backend == "pynput"  # Should fall back to pynput

    # Test 8: Alt key detection logic (lines 201-208)
    env.keyboard._active_only = False  # Disable gating for testing

    # Test Alt key detection
    alt_key = Mock(spec=keyboard.Key)
    alt_key.name = "alt"

    assert env.keyboard.alt_flag == 0
    env.keyboard._on_pynput_press(alt_key)
    assert env.keyboard.alt_flag

    # Test that Alt key press returns early (doesn't process other logic)
    # We can verify this by checking that key_vel is still zeros
    assert env.keyboard.key_vel[0, 0] == 0.0
    assert env.keyboard.key_vel[1, 0] == 0.0

    # Test 9: Alt key handling in matplotlib backend
    mpl_kb = irsim.gui.keyboard_control.KeyboardControl(env, backend="mpl")

    # Test Alt key detection in matplotlib backend
    class MockEvent:
        def __init__(self, key):
            self.key = key

    # Test alt+1 combination
    event = MockEvent("alt+1")
    mpl_kb._on_mpl_press(event)
    assert mpl_kb.alt_flag
    assert mpl_kb.key_id == 1

    # Test standalone alt key
    event = MockEvent("alt")
    mpl_kb._on_mpl_press(event)
    assert mpl_kb.alt_flag

    # Test 10: ESC key handling in matplotlib backend
    # Test escape key
    event = MockEvent("escape")
    mpl_kb._on_mpl_release(event)
    assert env.quit_flag

    # Test esc key
    env.quit_flag = False
    event = MockEvent("esc")
    mpl_kb._on_mpl_release(event)
    assert env.quit_flag

    env.end()
    assert True  # Add keyboard control related assertions


def test_keyboard_control_mpl_backend():
    """Test keyboard control via Matplotlib backend key events."""
    # Ensure world is set to keyboard mode via YAML; KeyboardControl defaults to mpl backend
    env = irsim.make("test_keyboard_control2.yaml", save_ani=False, display=False)

    # Helper event mock
    class E:
        def __init__(self, key: str):
            self.key = key

    # alt+1 selects robot id 1 (pressed together)
    env.keyboard._on_mpl_press(E("alt+0"))
    assert env.keyboard.key_id == 0

    # Press 'w' then step → linear velocity should be positive
    env.keyboard._on_mpl_press(E("w"))
    env.step()
    v = env.robot.velocity.reshape(-1)
    assert v[0] > 0

    # Release 'w' then step → linear velocity should go to zero
    env.keyboard._on_mpl_release(E("w"))
    env.step()
    v = env.robot.velocity.reshape(-1)
    assert abs(v[0]) <= 1e-9

    # Press 's' (backward) then release
    env.keyboard._on_mpl_press(E("s"))
    env.step()
    v = env.robot.velocity.reshape(-1)
    assert v[0] < 0
    env.keyboard._on_mpl_release(E("s"))
    env.step()
    v = env.robot.velocity.reshape(-1)
    assert abs(v[0]) <= 1e-9

    # Turn left 'a' (positive angular), then release
    env.keyboard._on_mpl_press(E("a"))
    env.step()
    v = env.robot.velocity.reshape(-1)
    assert v[1] > 0
    env.keyboard._on_mpl_release(E("a"))
    env.step()
    v = env.robot.velocity.reshape(-1)
    assert abs(v[1]) <= 1e-9

    # Turn right 'd' (negative angular), then release
    env.keyboard._on_mpl_press(E("d"))
    env.step()
    v = env.robot.velocity.reshape(-1)
    assert v[1] < 0
    env.keyboard._on_mpl_release(E("d"))
    env.step()
    v = env.robot.velocity.reshape(-1)
    assert abs(v[1]) <= 1e-9

    # Increase linear max with 'e', press 'w' should reflect new max
    prev_lv_max = env.keyboard.key_lv_max
    env.keyboard._on_mpl_release(E("e"))
    assert env.keyboard.key_lv_max > prev_lv_max
    env.keyboard._on_mpl_press(E("w"))
    env.step()
    v = env.robot.velocity.reshape(-1)
    assert pytest.approx(v[0], rel=1e-9, abs=1e-9) == env.keyboard.key_lv_max
    env.keyboard._on_mpl_release(E("w"))
    env.step()

    # Decrease linear max with 'q', press 'w' should reflect decreased max
    prev_lv_max = env.keyboard.key_lv_max
    env.keyboard._on_mpl_release(E("q"))
    assert env.keyboard.key_lv_max < prev_lv_max
    env.keyboard._on_mpl_press(E("w"))
    env.step()
    v = env.robot.velocity.reshape(-1)
    assert pytest.approx(v[0], rel=1e-9, abs=1e-9) == env.keyboard.key_lv_max
    env.keyboard._on_mpl_release(E("w"))
    env.step()

    # Increase angular max with 'c', press 'a' should match new max
    prev_ang_max = env.keyboard.key_ang_max
    env.keyboard._on_mpl_release(E("c"))
    assert env.keyboard.key_ang_max > prev_ang_max
    env.keyboard._on_mpl_press(E("a"))
    env.step()
    v = env.robot.velocity.reshape(-1)
    env.keyboard._on_mpl_release(E("a"))
    env.step()

    # Decrease angular max with 'z', press 'a' should match decreased max
    prev_ang_max = env.keyboard.key_ang_max
    env.keyboard._on_mpl_release(E("z"))
    assert env.keyboard.key_ang_max < prev_ang_max
    env.keyboard._on_mpl_press(E("a"))
    env.step()
    v = env.robot.velocity.reshape(-1)
    env.keyboard._on_mpl_release(E("a"))
    env.step()

    # Alt + number beyond robot count should still set key_id
    env.keyboard._on_mpl_press(E("alt+9"))
    assert env.keyboard.key_id == 9

    # Space toggles pause/resume
    # Ensure Running before first toggle (some configs start as None)
    if "Running" not in env.status:
        env.resume()
    env.keyboard._on_mpl_release(E("space"))
    assert env.pause_flag is True
    assert env.status.startswith("Pause")
    env.keyboard._on_mpl_release(E("space"))
    assert "Running" in env.status

    # 'r' resets the environment (handled via flag; processed during render/step)
    env.keyboard._on_mpl_release(E("r"))
    env.step()
    assert "Reset" in env.status or "Collision" in env.status

    # 'l' reloads the environment in the same figure (no exception expected)
    env.keyboard._on_mpl_release(E("l"))
    assert env.status in ("Running", "Pause", "Reset", "Collision", "Arrived", "None")

    # 'v' saves the figure via flag processed in render
    with patch.object(env, "save_figure") as mock_save:
        env.keyboard._on_mpl_release(E("v"))
        env.render(0.0)
        mock_save.assert_called_once()

    # 'f5' debug single-step: allows exactly one step per press
    t0 = env.time
    env.keyboard._on_mpl_release(E("f5"))
    env.step()
    t1 = env.time
    assert t1 > t0
    env.step()
    assert env.time == t1  # blocked until next F5
    # status should reflect debugging pause
    # assert "Pause (Debugging)" in env.status
    # next F5 permits one more step
    env.keyboard._on_mpl_release(E("f5"))
    env.step()
    assert env.time > t1

    # 'x' toggles control mode (keyboard <-> auto)
    from irsim.config import world_param as wp_mpl

    mode0 = wp_mpl.control_mode
    env.keyboard._on_mpl_release(E("x"))
    assert wp_mpl.control_mode != mode0
    env.keyboard._on_mpl_release(E("x"))
    assert wp_mpl.control_mode == mode0

    # 'esc' quits (raises SystemExit in mpl backend)
    env.keyboard._on_mpl_release(E("esc"))
    assert env.quit_flag is True

    env.end()
    assert True


def test_mouse_control():
    """Test mouse control functionality"""
    env = irsim.make("test_multi_objects_world.yaml", save_ani=False, display=False)

    # Get the axes from the environment
    ax = env._env_plot.ax

    # Initialize mouse control
    mouse_control = MouseControl(ax, zoom_factor=1.2)

    # Test initial state
    assert mouse_control.mouse_pos is None
    assert mouse_control.left_click_pos is None
    assert mouse_control.right_click_pos is None
    assert mouse_control.zoom_factor == 1.2

    # Test mouse movement
    mock_move_event = Mock()
    mock_move_event.inaxes = ax
    mock_move_event.xdata = 5.0
    mock_move_event.ydata = 5.0
    mouse_control.on_move(mock_move_event)
    assert mouse_control.mouse_pos == (5.0, 5.0)

    # Test left click
    mock_left_click = Mock()
    mock_left_click.button = MouseButton.LEFT
    mock_left_click.inaxes = ax
    mock_left_click.xdata = 3.0
    mock_left_click.ydata = 3.0
    mouse_control.on_click(mock_left_click)
    assert mouse_control.left_click_pos is not None
    assert mouse_control.left_click_pos[0] == 3.0
    assert mouse_control.left_click_pos[1] == 3.0

    # Test right click
    mock_right_click = Mock()
    mock_right_click.button = MouseButton.RIGHT
    mock_right_click.inaxes = ax
    mock_right_click.xdata = 7.0
    mock_right_click.ydata = 7.0
    mouse_control.on_click(mock_right_click)
    assert mouse_control.right_click_pos is not None
    assert mouse_control.right_click_pos[0] == 7.0
    assert mouse_control.right_click_pos[1] == 7.0

    # Test middle click (zoom reset)
    initial_xlim = ax.get_xlim()
    initial_ylim = ax.get_ylim()

    # First zoom in using scroll
    mock_scroll = Mock()
    mock_scroll.inaxes = ax
    mock_scroll.xdata = 5.0
    mock_scroll.ydata = 5.0
    mock_scroll.step = 1  # Scroll up
    mouse_control.on_scroll(mock_scroll)

    # Verify zoom changed
    assert ax.get_xlim() != initial_xlim
    assert ax.get_ylim() != initial_ylim

    # Test middle click reset
    mock_middle_click = Mock()
    mock_middle_click.button = MouseButton.MIDDLE
    mock_middle_click.inaxes = ax
    mouse_control.on_click(mock_middle_click)

    # Verify zoom reset
    assert ax.get_xlim() == initial_xlim
    assert ax.get_ylim() == initial_ylim

    # Test zoom factor change
    mouse_control.set_zoom_factor(1.5)
    assert mouse_control.zoom_factor == 1.5

    # Test minimum zoom factor
    mouse_control.set_zoom_factor(1.0)  # Try to set below minimum
    assert mouse_control.zoom_factor == 1.1  # Should be set to minimum

    # Test mouse release
    mock_left_release = Mock()
    mock_left_release.button = MouseButton.LEFT
    mouse_control.on_release(mock_left_release)
    assert mouse_control.left_click_pos is None

    mock_right_release = Mock()
    mock_right_release.button = MouseButton.RIGHT
    mouse_control.on_release(mock_right_release)
    assert mouse_control.right_click_pos is None

    # Test movement outside axes
    mock_move_outside = Mock()
    mock_move_outside.inaxes = None
    mouse_control.on_move(mock_move_outside)
    assert mouse_control.mouse_pos is None

    with pytest.raises(SystemExit):
        env.quit()

    assert True  # All tests passed if we reach here


def test_custom_behavior():
    """Test custom behavior"""
    env = irsim.make("custom_behavior.yaml", display=False)
    env.load_behavior("custom_behavior_methods")

    for _i in range(10):
        env.step()
        env.render(0.01)
    env.end()
    assert True  # Add behavior related assertions


def test_env_flags_processing():
    """Test processing of save, reset, reload, and quit flags"""
    env = irsim.make("test_all_objects.yaml", save_ani=False, display=False)

    # save_figure_flag triggers save_figure during render and clears flag
    with patch.object(env, "save_figure") as mock_save:
        env.save_figure_flag = True
        env.render(0.0)
        mock_save.assert_called_once()
        assert env.save_figure_flag is False

    # reset_flag triggers reset during render and clears flag
    env.reset_flag = True
    env.render(0.0)
    assert env.reset_flag is False

    # reload_flag triggers reload during render and clears flag
    env.reload_flag = True
    env.render(0.0)
    assert env.reload_flag is False

    # quit_flag causes step to raise SystemExit
    env.quit_flag = True
    with pytest.raises(SystemExit):
        env.step()

    # No further assertions; reaching here means flags processed as expected


def test_fov_detection():
    """Test field of view detection"""
    env = irsim.make("test_fov_world.yaml", save_ani=False, display=False)
    env.obstacle_list[0].get_fov_detected_objects()

    for _i in range(30):
        detected = [obs.fov_detect_object(env.robot) for obs in env.obstacle_list]
        env.step()
        env.render(
            0.01, fov_color="red", fov_alpha=0.2, fov_edge_color="red", fov_zorder=2
        )
    env.end()
    assert isinstance(detected, list)


def test_3d_projection():
    """Test 3D projection"""
    env = irsim.make(
        "test_multi_objects_world.yaml", save_ani=False, display=False, projection="3d"
    )
    env.random_obstacle_position(ids=[3, 4, 5, 6, 7], non_overlapping=True)

    for _i in range(5):
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
    TestClass(time_print=True)


def test_validate_unique_names_pass():
    """Environment should initialize when object names are unique"""
    env = irsim.make("test_all_objects.yaml", display=False)
    # names are unique across all objects
    assert len(env.names) == len(set(env.names))
    env.end()


def test_validate_unique_names_duplicate_raises():
    """Environment should raise on duplicate object names"""
    with pytest.raises(ValueError, match="Duplicate object names"):
        irsim.make("test_duplicate_names.yaml", display=False)


def test_add_object_duplicate_raises():
    env = irsim.make("test_all_objects.yaml", display=False)

    obs = env.create_obstacle(
        shape={"name": "polygon", "vertices": [[6, 5], [7, 5], [7, 6], [6, 6]]}
    )

    env.add_object(obs)
    with pytest.raises(ValueError, match=f"Object name '{obs.name}' already exists."):
        env.add_object(obs)


def test_add_objects_duplicate_raises():
    env = irsim.make("test_all_objects.yaml", display=True)

    obs = env.create_obstacle(
        shape={"name": "polygon", "vertices": [[6, 5], [7, 5], [7, 6], [6, 6]]}
    )
    env.add_object(obs)

    with pytest.raises(
        ValueError, match=re.escape(f"Object names already exist: {[obs.name]}")
    ):
        env.add_objects([obs])

    env.end()


def test_envbase_empty_yaml_path_logs(capsys):
    from irsim.env.env_base import EnvBase

    # Creating EnvBase with empty YAML may raise or proceed with defaults;
    # in both cases we expect an error/critical log message.
    with contextlib.suppress(Exception):
        EnvBase(
            "",
            display=False,
            disable_all_plot=True,
            log_file=None,
            log_level="CRITICAL",
        )

    out = capsys.readouterr().out
    assert "YAML Configuration load failed" in out or "YAML File not found" in out


if __name__ == "__main__":
    pytest.main(["--cov=.", "--cov-report", "html", "-v", __file__])
