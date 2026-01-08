"""
Tests for keyboard control functionality.

Covers both pynput and matplotlib backends for keyboard input handling.
"""

from unittest.mock import Mock, patch

import pytest

import irsim
import irsim.gui.keyboard_control

pynput = pytest.importorskip("pynput")
keyboard = pynput.keyboard


class TestKeyboardControlPynput:
    """Tests for keyboard control via pynput backend."""

    def test_basic_key_presses(self, env_factory, mock_keyboard_key):
        """Test basic keyboard key press and release."""
        env = env_factory("test_keyboard_control.yaml")
        key_list = ["w", "a", "s", "d", "q", "e", "z", "c", "r", "l", "v", "x"]
        mock_keys = [mock_keyboard_key(c) for c in key_list]

        for _ in range(3):
            for mock_key in mock_keys:
                env.keyboard._on_pynput_press(mock_key)
                env.keyboard._on_pynput_release(mock_key)
            env.step()
            env.render(0.01)

    def test_reload_key(self, env_factory, mock_keyboard_key):
        """Test 'l' key reloads environment."""
        env = env_factory("test_keyboard_control.yaml")
        pre_names = list(env.names)
        env.keyboard._on_pynput_release(mock_keyboard_key("l"))
        assert len(env.names) == len(pre_names)

    def test_alt_key_robot_selection(self, env_factory):
        """Test Alt+number selects robot."""
        env = env_factory("test_keyboard_control.yaml")
        alt_key = Mock(spec=keyboard.Key)
        alt_key.name = "alt"
        num_keys = [Mock(spec=keyboard.Key, char=str(i)) for i in range(5)]

        env.keyboard.is_active = True
        env.keyboard._active_only = False
        env.keyboard._on_pynput_press(alt_key)
        assert env.keyboard.alt_flag

        for i, num_key in enumerate(num_keys):
            env.keyboard._on_pynput_press(num_key)
            assert env.keyboard.key_id == i

        env.keyboard._on_pynput_release(alt_key)
        assert not env.keyboard.alt_flag

    def test_space_key_pause_resume(self, env_factory):
        """Test space key toggles pause/resume."""
        env = env_factory("test_keyboard_control.yaml")
        env.keyboard._active_only = False  # Allow testing without focus

        class SpaceKeyMock:
            name = "space"

            def __eq__(self, other):
                return other == keyboard.Key.space

        space_key = SpaceKeyMock()

        if "Running" not in env.status:
            env.resume()

        env.keyboard._on_pynput_release(space_key)
        assert env.status == "Pause"

        env.keyboard._on_pynput_release(space_key)
        assert env.status == "Running"

    def test_x_key_toggles_control_mode(self, env_factory):
        """Test 'x' key toggles control mode."""
        env = env_factory("test_keyboard_control.yaml")
        env.keyboard._active_only = False  # Allow testing without focus

        class XKeyMock:
            char = "x"

        mode0 = env.keyboard._world_param.control_mode
        env.keyboard._on_pynput_release(XKeyMock())
        assert env._world_param.control_mode != mode0
        env.keyboard._on_pynput_release(XKeyMock())
        assert env._world_param.control_mode == mode0

    def test_qe_adjust_lv_max(self, env_factory, mock_keyboard_key):
        """Test 'q'/'e' keys adjust linear velocity max."""
        env = env_factory("test_keyboard_control.yaml")
        env.keyboard._active_only = False  # Allow testing without focus
        prev_lv_max = env.keyboard.key_lv_max
        env.keyboard._on_pynput_release(mock_keyboard_key("e"))
        assert env.keyboard.key_lv_max > prev_lv_max
        env.keyboard._on_pynput_release(mock_keyboard_key("q"))
        assert env.keyboard.key_lv_max < prev_lv_max + 0.2

    def test_zc_adjust_ang_max(self, env_factory, mock_keyboard_key):
        """Test 'z'/'c' keys adjust angular velocity max."""
        env = env_factory("test_keyboard_control.yaml")
        env.keyboard._active_only = False  # Allow testing without focus
        prev_ang_max = env.keyboard.key_ang_max
        env.keyboard._on_pynput_release(mock_keyboard_key("c"))
        assert env.keyboard.key_ang_max > prev_ang_max
        env.keyboard._on_pynput_release(mock_keyboard_key("z"))
        assert env.keyboard.key_ang_max < prev_ang_max + 0.2

    def test_r_key_sets_reset_flag(self, env_factory, mock_keyboard_key):
        """Test 'r' key sets reset flag."""
        env = env_factory("test_keyboard_control.yaml")
        env.keyboard._active_only = False  # Allow testing without focus
        env.keyboard._on_pynput_release(mock_keyboard_key("r"))
        assert env.reset_flag is True
        env.render(0.0)
        assert env.reset_flag is False

    def test_l_key_sets_reload_flag(self, env_factory, mock_keyboard_key):
        """Test 'l' key sets reload flag."""
        env = env_factory("test_keyboard_control.yaml")
        env.keyboard._active_only = False  # Allow testing without focus
        env.keyboard._on_pynput_release(mock_keyboard_key("l"))
        assert env.reload_flag is True
        env.render(0.0)
        assert env.reload_flag is False

    def test_v_key_saves_figure(self, env_factory, mock_keyboard_key):
        """Test 'v' key saves figure."""
        env = env_factory("test_keyboard_control.yaml")
        env.keyboard._active_only = False  # Allow testing without focus
        with patch.object(env, "save_figure") as mock_save:
            env.keyboard._on_pynput_release(mock_keyboard_key("v"))
            env.render(0.0)
            mock_save.assert_called_once()

    def test_f5_debug_single_step(self, env_factory):
        """Test F5 key for debug single-step."""
        env = env_factory("test_keyboard_control.yaml")
        env.keyboard._active_only = False  # Allow testing without focus

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

    def test_invalid_backend_falls_back_to_mpl(self, env_factory):
        """Test invalid backend falls back to mpl."""
        env = env_factory("test_keyboard_control.yaml")
        with patch.object(env._env_param.logger, "warning") as mock_warning:
            kb = irsim.gui.keyboard_control.KeyboardControl(env, backend="invalid")
            assert kb.backend == "mpl"
            mock_warning.assert_called_once()

    def test_alt_key_attribute_error_handling(self, env_factory):
        """Test Alt key AttributeError is handled gracefully."""
        env = env_factory("test_keyboard_control.yaml")
        env.keyboard._active_only = False

        class BadKey:
            def __getattr__(self, name):
                if name == "name":
                    raise AttributeError("Mock attribute error")
                return

        bad_key = BadKey()
        env.keyboard._on_pynput_press(bad_key)
        assert env.keyboard.alt_flag == 0

    def test_pynput_unavailable_falls_back(self, env_factory):
        """Test pynput unavailable falls back to mpl."""
        env = env_factory("test_keyboard_control.yaml")
        with (
            patch("irsim.gui.keyboard_control._PYNPUT_AVAILABLE", False),
            patch.object(env._env_param.logger, "warning") as mock_warning,
        ):
            kb = irsim.gui.keyboard_control.KeyboardControl(env, backend="pynput")
            assert kb.backend == "mpl"
            mock_warning.assert_called_once()

    def test_mpl_connect_exception_handling(self, env_factory):
        """Test matplotlib connection exception handling."""
        env = env_factory("test_keyboard_control.yaml")
        with patch("matplotlib.pyplot.gcf") as mock_gcf:
            mock_fig = Mock()
            mock_fig.canvas.mpl_connect.side_effect = Exception("Connection failed")
            mock_gcf.return_value = mock_fig
            kb = irsim.gui.keyboard_control.KeyboardControl(env)
            assert kb.backend == "pynput"

    def test_focus_event_handlers(self, env_factory):
        """Test focus event handlers."""
        env = env_factory("test_keyboard_control.yaml")
        env.keyboard._on_mpl_focus_in(None)
        assert env.keyboard._is_active

        env.keyboard._on_mpl_focus_out(None)
        assert not env.keyboard._is_active

        env.keyboard._on_mpl_close(None)
        assert not env.keyboard._is_active


class TestKeyboardControlMpl:
    """Tests for keyboard control via matplotlib backend."""

    def test_basic_movement_keys(self, env_factory, mock_mpl_event):
        """Test basic movement keys (w, a, s, d)."""
        env = env_factory("test_keyboard_control2.yaml")

        # Press 'w' - forward
        env.keyboard._on_mpl_press(mock_mpl_event("w"))
        env.step()
        v = env.robot.velocity.reshape(-1)
        assert v[0] > 0

        # Release 'w'
        env.keyboard._on_mpl_release(mock_mpl_event("w"))
        env.step()
        v = env.robot.velocity.reshape(-1)
        assert abs(v[0]) <= 1e-9

        # Press 's' - backward
        env.keyboard._on_mpl_press(mock_mpl_event("s"))
        env.step()
        v = env.robot.velocity.reshape(-1)
        assert v[0] < 0
        env.keyboard._on_mpl_release(mock_mpl_event("s"))
        env.step()

        # Press 'a' - left
        env.keyboard._on_mpl_press(mock_mpl_event("a"))
        env.step()
        v = env.robot.velocity.reshape(-1)
        assert v[1] > 0
        env.keyboard._on_mpl_release(mock_mpl_event("a"))
        env.step()

        # Press 'd' - right
        env.keyboard._on_mpl_press(mock_mpl_event("d"))
        env.step()
        v = env.robot.velocity.reshape(-1)
        assert v[1] < 0
        env.keyboard._on_mpl_release(mock_mpl_event("d"))
        env.step()

    def test_alt_robot_selection(self, env_factory, mock_mpl_event):
        """Test Alt+number selects robot in mpl backend."""
        env = env_factory("test_keyboard_control2.yaml")
        mpl_kb = irsim.gui.keyboard_control.KeyboardControl(env, backend="mpl")

        mpl_kb._on_mpl_press(mock_mpl_event("alt+0"))
        assert mpl_kb.key_id == 0

        mpl_kb._on_mpl_press(mock_mpl_event("alt+1"))
        assert mpl_kb.alt_flag
        assert mpl_kb.key_id == 1

        mpl_kb._on_mpl_press(mock_mpl_event("alt"))
        assert mpl_kb.alt_flag

        mpl_kb._on_mpl_press(mock_mpl_event("alt+9"))
        assert mpl_kb.key_id == 9

    def test_lv_max_adjustment(self, env_factory, mock_mpl_event):
        """Test linear velocity max adjustment with e/q keys."""
        env = env_factory("test_keyboard_control2.yaml")

        prev_lv_max = env.keyboard.key_lv_max
        env.keyboard._on_mpl_release(mock_mpl_event("e"))
        assert env.keyboard.key_lv_max > prev_lv_max

        env.keyboard._on_mpl_press(mock_mpl_event("w"))
        env.step()
        v = env.robot.velocity.reshape(-1)
        assert pytest.approx(v[0], rel=1e-9, abs=1e-9) == env.keyboard.key_lv_max
        env.keyboard._on_mpl_release(mock_mpl_event("w"))
        env.step()

        prev_lv_max = env.keyboard.key_lv_max
        env.keyboard._on_mpl_release(mock_mpl_event("q"))
        assert env.keyboard.key_lv_max < prev_lv_max

    def test_ang_max_adjustment(self, env_factory, mock_mpl_event):
        """Test angular velocity max adjustment with c/z keys."""
        env = env_factory("test_keyboard_control2.yaml")

        prev_ang_max = env.keyboard.key_ang_max
        env.keyboard._on_mpl_release(mock_mpl_event("c"))
        assert env.keyboard.key_ang_max > prev_ang_max

        env.keyboard._on_mpl_release(mock_mpl_event("z"))
        assert env.keyboard.key_ang_max < prev_ang_max + 0.2

    def test_space_pause_resume(self, env_factory, mock_mpl_event):
        """Test space key toggles pause/resume in mpl backend."""
        env = env_factory("test_keyboard_control2.yaml")

        if "Running" not in env.status:
            env.resume()
        env.keyboard._on_mpl_release(mock_mpl_event("space"))
        assert env.pause_flag is True
        assert env.status.startswith("Pause")
        env.keyboard._on_mpl_release(mock_mpl_event("space"))
        assert "Running" in env.status

    def test_r_key_reset(self, env_factory, mock_mpl_event):
        """Test 'r' key reset in mpl backend."""
        env = env_factory("test_keyboard_control2.yaml")
        env.keyboard._on_mpl_release(mock_mpl_event("r"))
        env.step()
        assert "Reset" in env.status or "Collision" in env.status

    def test_l_key_reload(self, env_factory, mock_mpl_event):
        """Test 'l' key reload in mpl backend."""
        env = env_factory("test_keyboard_control2.yaml")
        env.keyboard._on_mpl_release(mock_mpl_event("l"))
        assert env.status in (
            "Running",
            "Pause",
            "Reset",
            "Collision",
            "Arrived",
            "None",
        )

    def test_v_key_save_figure(self, env_factory, mock_mpl_event):
        """Test 'v' key saves figure in mpl backend."""
        env = env_factory("test_keyboard_control2.yaml")
        with patch.object(env, "save_figure") as mock_save:
            env.keyboard._on_mpl_release(mock_mpl_event("v"))
            env.render(0.0)
            mock_save.assert_called_once()

    @pytest.mark.skip(
        reason="Flaky test due to global state pollution - tested in test_all_objects.py"
    )
    def test_f5_debug_step(self, mock_mpl_event):
        """Test F5 debug single-step in mpl backend."""

        # Create fresh environment to avoid state pollution from other tests
        import irsim

        env = irsim.make("test_keyboard_control2.yaml", save_ani=False, display=False)
        try:
            # Ensure environment is running first and reset debug state
            if "Running" not in env.status:
                env.resume()
            env.debug_flag = False
            env.debug_count = 0
            t0 = env.time
            env.keyboard._on_mpl_release(mock_mpl_event("f5"))
            env.step()
            t1 = env.time
            assert t1 > t0, f"First step should advance time: t0={t0}, t1={t1}"
            env.step()
            # After one allowed step, further steps are blocked
            assert env.time == t1, (
                f"Second step should be blocked: time={env.time}, t1={t1}"
            )
            env.keyboard._on_mpl_release(mock_mpl_event("f5"))
            env.step()
            assert env.time > t1, (
                f"Third step after F5 should advance: time={env.time}, t1={t1}"
            )
        finally:
            env.end()

    def test_x_key_toggle_mode(self, env_factory, mock_mpl_event):
        """Test 'x' key toggles control mode in mpl backend."""
        env = env_factory("test_keyboard_control2.yaml")

        mode0 = env.keyboard._world_param.control_mode
        env.keyboard._on_mpl_release(mock_mpl_event("x"))
        assert env.keyboard._world_param.control_mode != mode0
        env.keyboard._on_mpl_release(mock_mpl_event("x"))
        assert env.keyboard._world_param.control_mode == mode0

    def test_escape_key_quit(self, env_factory, mock_mpl_event):
        """Test escape key sets quit flag in mpl backend."""
        env = env_factory("test_keyboard_control2.yaml")
        mpl_kb = irsim.gui.keyboard_control.KeyboardControl(env, backend="mpl")

        mpl_kb._on_mpl_release(mock_mpl_event("escape"))
        assert env.quit_flag

        env.quit_flag = False
        mpl_kb._on_mpl_release(mock_mpl_event("esc"))
        assert env.quit_flag


class TestMultiEnvSwitching:
    """Tests for multi-environment keyboard switching."""

    def test_only_one_active_at_a_time(self, env_factory):
        """Test that only one keyboard instance is active at a time."""
        env1 = env_factory("test_keyboard_control.yaml")
        env2 = env_factory("test_keyboard_control2.yaml")

        # Initially neither is active
        assert not env1.keyboard._is_active
        assert not env2.keyboard._is_active

        # Activate env1
        env1.keyboard._set_active()
        assert env1.keyboard._is_active
        assert not env2.keyboard._is_active

        # Activate env2 - env1 should be deactivated
        env2.keyboard._set_active()
        assert not env1.keyboard._is_active
        assert env2.keyboard._is_active

        # Switch back to env1
        env1.keyboard._set_active()
        assert env1.keyboard._is_active
        assert not env2.keyboard._is_active

    def test_global_instance_tracking(self, env_factory):
        """Test that global _active_keyboard_instance is correctly tracked."""
        import irsim.gui.keyboard_control as kb_module

        env1 = env_factory("test_keyboard_control.yaml")
        env2 = env_factory("test_keyboard_control2.yaml")

        # Activate env1
        env1.keyboard._set_active()
        assert kb_module._active_keyboard_instance is env1.keyboard

        # Activate env2
        env2.keyboard._set_active()
        assert kb_module._active_keyboard_instance is env2.keyboard

    def test_focus_in_activates(self, env_factory):
        """Test that focus in event activates keyboard control."""
        env1 = env_factory("test_keyboard_control.yaml")
        env2 = env_factory("test_keyboard_control2.yaml")

        # Simulate focus in on env1
        env1.keyboard._on_mpl_focus_in(None)
        assert env1.keyboard._is_active
        assert not env2.keyboard._is_active

        # Simulate focus in on env2
        env2.keyboard._on_mpl_focus_in(None)
        assert not env1.keyboard._is_active
        assert env2.keyboard._is_active

    def test_button_press_activates(self, env_factory):
        """Test that button press event activates keyboard control."""
        env1 = env_factory("test_keyboard_control.yaml")
        env2 = env_factory("test_keyboard_control2.yaml")

        # Simulate button press on env1
        env1.keyboard._on_mpl_button_press(None)
        assert env1.keyboard._is_active
        assert not env2.keyboard._is_active

        # Simulate button press on env2
        env2.keyboard._on_mpl_button_press(None)
        assert not env1.keyboard._is_active
        assert env2.keyboard._is_active

    def test_focus_out_deactivates_only_active(self, env_factory):
        """Test that focus out only deactivates the currently active instance."""
        import irsim.gui.keyboard_control as kb_module

        env1 = env_factory("test_keyboard_control.yaml")
        env2 = env_factory("test_keyboard_control2.yaml")

        # Activate env1
        env1.keyboard._set_active()

        # Focus out on env2 should NOT deactivate env1
        env2.keyboard._on_mpl_focus_out(None)
        assert env1.keyboard._is_active

        # Focus out on env1 should deactivate it
        env1.keyboard._on_mpl_focus_out(None)
        assert not env1.keyboard._is_active
        assert kb_module._active_keyboard_instance is env1.keyboard  # Still tracked

    def test_close_clears_global_instance(self, env_factory):
        """Test that closing the active window clears global instance."""
        import irsim.gui.keyboard_control as kb_module

        env1 = env_factory("test_keyboard_control.yaml")
        env2 = env_factory("test_keyboard_control2.yaml")

        # Activate env1 and close it
        env1.keyboard._set_active()
        env1.keyboard._on_mpl_close(None)
        assert not env1.keyboard._is_active
        assert kb_module._active_keyboard_instance is None

        # Activate env2 and close it
        env2.keyboard._set_active()
        env2.keyboard._on_mpl_close(None)
        assert not env2.keyboard._is_active
        assert kb_module._active_keyboard_instance is None

    def test_pynput_gated_by_active(self, env_factory, mock_keyboard_key):
        """Test pynput key events are gated by active status."""
        env1 = env_factory("test_keyboard_control.yaml")
        env2 = env_factory("test_keyboard_control2.yaml")

        # Enable active_only mode (default)
        env1.keyboard._active_only = True
        env2.keyboard._active_only = True

        # Neither is active - key presses should be ignored
        initial_lv1 = env1.keyboard.key_lv
        initial_lv2 = env2.keyboard.key_lv
        env1.keyboard._on_pynput_press(mock_keyboard_key("w"))
        env2.keyboard._on_pynput_press(mock_keyboard_key("w"))
        assert env1.keyboard.key_lv == initial_lv1
        assert env2.keyboard.key_lv == initial_lv2

        # Activate env1 - only env1 should respond
        env1.keyboard._set_active()
        env1.keyboard._on_pynput_press(mock_keyboard_key("w"))
        env2.keyboard._on_pynput_press(mock_keyboard_key("w"))
        assert env1.keyboard.key_lv == env1.keyboard.key_lv_max
        assert env2.keyboard.key_lv == initial_lv2

        # Release and reset
        env1.keyboard._on_pynput_release(mock_keyboard_key("w"))

        # Activate env2 - only env2 should respond
        env2.keyboard._set_active()
        env1.keyboard._on_pynput_press(mock_keyboard_key("w"))
        env2.keyboard._on_pynput_press(mock_keyboard_key("w"))
        assert env1.keyboard.key_lv == 0  # Still from release
        assert env2.keyboard.key_lv == env2.keyboard.key_lv_max


class TestKeyboardControl3D:
    """Tests for keyboard control in 3D projection."""

    def test_keyboard_control_3d(self, env_factory, mock_keyboard_key):
        """Test keyboard control works with 3D projection."""
        env = env_factory("test_keyboard_control.yaml", projection="3d")
        key_list = ["w", "a", "s", "d", "q", "e", "z", "c", "r"]
        mock_keys = [mock_keyboard_key(c) for c in key_list]

        for _ in range(3):
            for mock_key in mock_keys:
                env.keyboard._on_pynput_press(mock_key)
                env.keyboard._on_pynput_release(mock_key)
            env.step()
            env.render(0.01)
