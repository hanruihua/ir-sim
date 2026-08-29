from typing import TYPE_CHECKING, Any, Optional

import matplotlib.pyplot as plt
import numpy as np

if TYPE_CHECKING:
    pass

# pynput is optional and costs ~85 ms to import, so it is loaded on first use:
# only when a keyboard controller with the pynput backend is created.
keyboard: Any | None = None
_pynput_missing = False  # a failed import is not retried for every environment


def _load_pynput() -> Any | None:
    """Import ``pynput.keyboard`` on demand; ``None`` when it is unavailable."""
    global keyboard, _pynput_missing
    if keyboard is None and not _pynput_missing:
        try:  # pragma: no cover - availability depends on environment
            from pynput import keyboard as pynput_keyboard
        except Exception:
            pynput_keyboard = None
        keyboard = pynput_keyboard if hasattr(pynput_keyboard, "Listener") else None
        _pynput_missing = keyboard is None
    return keyboard


# Global registry to track which KeyboardControl instance is currently active
# This ensures only one environment responds to keyboard input at a time
_active_keyboard_instance: Optional["KeyboardControl"] = None


class KeyboardControl:
    """
    Keyboard input handler for IR-SIM with dual backends.

    Overview:
        - Provides manual control of robots and environment using keyboard keys.
        - Supports two backends:
            - "mpl" (default): Matplotlib figure key events (no extra dependency).
            - "pynput": Global keyboard hook (requires the ``pynput`` package).
        - If the requested backend is unavailable, it automatically falls back to "mpl".

    Control mode gating:
        - Robot control keys (w/s/a/d, q/e, z/c, alt+number) only take effect when
          ``world_param.control_mode == "keyboard"``.
        - Environment keys (space to pause/resume, r to reset, esc to quit) are active
          in either control mode, and are ignored when no ``env_ref`` was given.

    Key mappings (both backends):
        - w/s: Increase/decrease linear velocity (forward/backward)
        - a/d: Turn left/right (diff/acker) or strafe left/right (omni/omni_angular)
        - q/e: Rotate left/right (omni_angular only)
        - z/c: Decrease/increase maximum angular velocity (key_ang_max)
        - shift+z/c: Decrease/increase maximum linear velocity (key_lv_max)
        - alt + number: Change current controlled robot id
        - r: Reset the environment (if ``env_ref`` provided)
        - space: Toggle pause/resume
        - esc: Quit the environment immediately (closes figure and raises ``SystemExit(0)``)
        - x: Switch between keyboard and auto control modes
        - l: Reload the environment
        - F5: Debug the environment
        - y: Toggle display render window

    Notes:
        - The "mpl" backend requires the Matplotlib figure window to have focus to receive key events.
        - In keyboard control mode, behavior planners are ignored; robot motion follows key input.
    """

    def __init__(self, env_ref: Any | None = None, **keyboard_kwargs: Any) -> None:
        """
        Initialize keyboard control for the environment.

        Args:
            env_ref: Reference to the environment instance, used by the command
                keys - reset, pause/resume, reload, debug, save figure, display
                and quit. Those keys are ignored when it is omitted.
            keyboard_kwargs (dict): Optional settings for keyboard control.

                - key_lv_max (float): Maximum linear velocity. Default is 3.0.
                - key_ang_max (float): Maximum angular velocity. Default is 1.0.
                - key_lv (float): Initial linear velocity. Default is 0.0.
                - key_ang (float): Initial angular velocity. Default is 0.0.
                - key_id (int): Initial robot control ID. Default is 0.
                - backend (str): Keyboard backend. ``"mpl"`` (default) uses
                  Matplotlib figure key events; ``"pynput"`` uses a global
                  keyboard hook if the ``pynput`` package is available.

        Keyboard mappings (both backends):
            - w: Move forward
            - s: Move backward
            - a: Turn left (diff/acker) or strafe left (omni/omni_angular)
            - d: Turn right (diff/acker) or strafe right (omni/omni_angular)
            - q: Rotate left (omni_angular)
            - e: Rotate right (omni_angular)
            - z/c: Decrease/Increase max angular velocity
            - shift+z/c: Decrease/Increase max linear velocity
            - alt + number: Change current control robot id
            - r: Reset the environment (if ``env_ref`` provided)
            - space: Pause/Resume the environment
            - esc: Quit the environment
            - x: Switch between keyboard and auto control modes
            - l: Reload the environment
            - F5: Debug the environment
            - y: Toggle display render window
        """

        # Store environment reference for reset functionality
        self.env_ref = env_ref

        self.key_lv_max = keyboard_kwargs.get("key_lv_max", 3.0)
        self.key_ang_max = keyboard_kwargs.get("key_ang_max", 1.0)
        self.key_lv = keyboard_kwargs.get("key_lv", 0.0)
        self.key_ang = keyboard_kwargs.get("key_ang", 0.0)
        self.key_rot = keyboard_kwargs.get("key_rot", 0.0)
        self.key_id = keyboard_kwargs.get("key_id", 0)
        self.alt_flag = 0

        # backend: 'pynput' or 'mpl' (matplotlib figure key events)
        # Default to MPL backend
        self.backend = keyboard_kwargs.get("backend", "pynput").strip().lower()
        if self.backend not in ("pynput", "mpl"):
            self.logger.warning(
                f"Invalid backend: {self.backend}. Using matplotlib backend by default."
            )
            self.backend = "mpl"

        # Only honor global hook when MPL window is active/focused
        self._active_only = not bool(keyboard_kwargs.get("global_hook", False))
        self._is_active = False

        if "s" in plt.rcParams["keymap.save"]:
            plt.rcParams["keymap.save"].remove("s")

        if "q" in plt.rcParams["keymap.quit"]:
            plt.rcParams["keymap.quit"].remove("q")

        if "l" in plt.rcParams["keymap.yscale"]:
            plt.rcParams["keymap.yscale"].remove("l")

        if "L" in plt.rcParams["keymap.xscale"]:
            plt.rcParams["keymap.xscale"].remove("L")

        self.key_vel = np.zeros((3, 1))

        if self._world_param.control_mode == "keyboard":
            self.logger.info("start to keyboard control")

            commands = [
                ["w", "forward"],
                ["s", "backward"],
                ["a", "turn left / strafe left"],
                ["d", "turn right / strafe right"],
                ["q", "rotate left (omni_angular)"],
                ["e", "rotate right (omni_angular)"],
                ["shift+z", "decrease linear velocity"],
                ["shift+c", "increase linear velocity"],
                ["z", "decrease angular velocity"],
                ["c", "increase angular velocity"],
                ["alt+num", "change current control robot id"],
                ["r", "reset the environment"],
                ["space", "pause/resume the environment"],
                ["esc", "quit the environment"],
                ["x", "switch keyboard control and auto control"],
                ["l", "reload the environment"],
                ["F5", "debug the environment"],
                ["v", "save the current figure"],
                ["y", "toggle display render window"],
            ]

            headers = ["Key", "Function"]
            print(self._format_grid_table(headers, commands))

        if self.backend == "pynput" and _load_pynput() is None:
            self.logger.warning("pynput is not available. Using matplotlib backend.")
            self.backend = "mpl"

        # Track Matplotlib window focus to gate pynput callbacks
        # Keyboard control is activated when mouse enters the figure area
        # and deactivated when mouse leaves the figure area
        try:
            fig = plt.gcf()
            self._mpl_enter_cid = fig.canvas.mpl_connect(
                "figure_enter_event", self._on_mpl_focus_in
            )
            self._mpl_leave_cid = fig.canvas.mpl_connect(
                "figure_leave_event", self._on_mpl_focus_out
            )
            self._mpl_close_cid = fig.canvas.mpl_connect(
                "close_event", self._on_mpl_close
            )
        except Exception:
            pass

        # Keyboard input is only meaningful with an interactive display. When
        # headless (display=False) skip installing the global OS keyboard
        # listener (and the matplotlib key-event fallback): pynput's listener
        # triggers OS input-monitoring permission prompts and can overflow the
        # event queue when many environments are spawned for parallel training.
        # The matplotlib focus wiring above still runs; the handlers stay
        # callable directly so tests and programmatic use are unaffected.
        interactive = getattr(self.env_ref, "display", True) if self.env_ref else True

        if not interactive:
            self.listener = None
        elif self.backend == "pynput":
            # Use pynput global keyboard listener
            self.listener = keyboard.Listener(
                on_press=self._on_pynput_press, on_release=self._on_pynput_release
            )
            self.listener.start()

        else:
            # Fallback to matplotlib figure key events
            self.listener = None
            fig = plt.gcf()
            self._mpl_press_cid = fig.canvas.mpl_connect(
                "key_press_event", self._on_mpl_press
            )
            self._mpl_release_cid = fig.canvas.mpl_connect(
                "key_release_event", self._on_mpl_release
            )

    # ------------------------------------------------------------------
    # Shared key actions (both backends)
    # ------------------------------------------------------------------

    def _select_robot_id(self, digit: str) -> None:
        """Point keyboard control at the robot with the given id."""
        target = int(digit)
        if self.env_ref and target >= self.env_ref.robot_number:
            self.logger.warning(
                f"{target} over the maximum id: {self.env_ref.robot_number - 1}"
            )
        else:
            self.logger.info(f"Current control id: {target}")
        self.key_id = target

    def _press_motion_key(self, key: str) -> None:
        """Apply the velocity a motion key commands while it is held."""
        if key == "w":
            self.key_lv = self.key_lv_max
        elif key == "s":
            self.key_lv = -self.key_lv_max
        elif key == "a":
            self.key_ang = self.key_ang_max
        elif key == "d":
            self.key_ang = -self.key_ang_max
        elif key == "q":
            self.key_rot = self.key_ang_max
        elif key == "e":
            self.key_rot = -self.key_ang_max

    def _release_motion_key(self, key: str) -> None:
        """Stop the motion a released key was commanding."""
        if key in ("w", "s"):
            self.key_lv = 0
        elif key in ("a", "d"):
            self.key_ang = 0
        elif key in ("q", "e"):
            self.key_rot = 0

    def _adjust_speed_limit(self, key: str, shift: bool) -> None:
        """Step the linear (with shift) or angular speed limit by 0.2."""
        if key not in ("z", "c"):
            return

        step = -0.2 if key == "z" else 0.2
        if shift:
            self.key_lv_max = self.key_lv_max + step
            self.logger.info(f"current linear velocity: {self.key_lv_max}")
        else:
            self.key_ang_max = self.key_ang_max + step
            self.logger.info(f"current angular velocity: {self.key_ang_max}")

    def _update_key_vel(self) -> None:
        """Publish the current keyboard velocity command."""
        self.key_vel = np.array([[self.key_lv], [self.key_ang], [self.key_rot]])

    def _reset_env(self) -> None:
        """Ask the environment to reset on its next step."""
        self.logger.info("reset the environment")
        if self.env_ref is not None:
            self.env_ref.reset_flag = True
        else:
            self.logger.warning("Environment reference not set. Cannot reset.")

    def _toggle_pause(self) -> None:
        """Pause a running environment, or resume a paused one."""
        if "Pause" not in self.env_ref.status:
            self.logger.info("pause the environment")
            self.env_ref.pause()
        else:
            self.logger.info("resume the environment")
            self.env_ref.resume()

    def _step_debug(self) -> None:
        """Enter single-step debug mode, or advance it by one step."""
        if not self.env_ref.debug_flag:
            self.env_ref.debug_flag = True
            self.env_ref.debug_count = self._world_param.count
            self.env_ref.pause_flag = False
        else:
            self.env_ref.debug_count += 1

    def _save_figure(self) -> None:
        """Ask the environment to save the current figure."""
        self.logger.info("save the figure")
        self.env_ref.save_figure_flag = True

    def _reload_env(self) -> None:
        """Ask the environment to reload its YAML configuration."""
        self.env_ref.reload_flag = True
        self.logger.info("reload the environment")

    def _toggle_display(self) -> None:
        """Turn rendering of the environment on or off."""
        self.env_ref.display = not self.env_ref.display
        state = "on" if self.env_ref.display else "off"
        self.logger.info(f"toggle display: {state}")

    def _quit_env(self) -> None:
        """Ask the environment to quit."""
        self.env_ref.quit_flag = True

    def _toggle_control_mode(self) -> None:
        """Switch between keyboard and automatic control, and log the new mode."""
        if self._world_param.control_mode == "keyboard":
            self._world_param.control_mode = "auto"
            self.logger.info("switch to auto control")
        else:
            self._world_param.control_mode = "keyboard"
            self.logger.info("switch to keyboard control")

    def _run_command_key(self, key: str) -> None:
        """Run the environment command bound to a released key, if any.

        A command that acts on the environment is ignored when the control was
        built without a reference to one.
        """
        commands = {
            "r": self._reset_env,
            "space": self._toggle_pause,
            "x": self._toggle_control_mode,
            "l": self._reload_env,
            "f5": self._step_debug,
            "v": self._save_figure,
            "y": self._toggle_display,
            "escape": self._quit_env,
        }
        command = commands.get(key)
        if command is None:
            return

        # Every command but the control-mode switch acts on the environment.
        if self.env_ref is None and command != self._toggle_control_mode:
            self.logger.warning(f"Environment reference not set. Ignoring '{key}'.")
            return

        command()

    # ------------------------------------------------------------------
    # pynput key event handlers (backend = 'pynput')
    # ------------------------------------------------------------------

    def _on_pynput_press(self, key: Any) -> None:
        """
        Handle key press events (pynput backend).

        Args:
            key: pynput.keyboard.Key instance.
        """

        # Gate by window activity if requested
        if self._active_only and not self._is_active:
            return

        # Check for Alt key first (special key)
        try:
            if hasattr(key, "name") and "alt" in key.name:
                self.alt_flag = True
                return
        except AttributeError:
            pass

        # Handle character keys
        try:
            if self._world_param.control_mode == "keyboard":
                if key.char.isdigit() and self.alt_flag:
                    self._select_robot_id(key.char)

                self._press_motion_key(key.char)
                self._update_key_vel()

        except AttributeError:
            # Handle other special keys that don't have char attribute
            pass

    def _on_pynput_release(self, key: Any) -> None:
        """
        Handle key release events (pynput backend).

        Args:
            key: pynput.keyboard.Key instance.
        """

        # Gate by window activity if requested
        if self._active_only and not self._is_active:
            return

        try:
            # Keys are matched as typed here: 'Z'/'C' carry the shift variant.
            char = key.char or ""
            if self._world_param.control_mode == "keyboard":
                self._release_motion_key(char)
                self._adjust_speed_limit(char.lower(), char.isupper())

            self._run_command_key(char)
            self._update_key_vel()

        except AttributeError:
            self._on_pynput_special_release(key)

    def _on_pynput_special_release(self, key: Any) -> None:
        """Handle release of keys that carry no character (alt, space, F5, ESC)."""
        if "alt" in key.name:
            self.alt_flag = False

        if keyboard is None:
            return

        # Named after the matplotlib keys so both backends dispatch through
        # _run_command_key, and share its missing-environment guard.
        if key == keyboard.Key.space:
            self._run_command_key("space")
        elif key == keyboard.Key.f5:
            self._run_command_key("f5")
        elif key == keyboard.Key.esc:
            self._run_command_key("escape")

    # ------------------------------------------------------------------
    # Matplotlib key event handlers (backend = 'mpl')
    # ------------------------------------------------------------------

    def _on_mpl_press(self, event: Any) -> None:
        """
        Handle Matplotlib figure key press events.

        Args:
            event: Matplotlib key press event with ``event.key`` string.
        """
        key = (event.key or "").lower()

        # Update alt flag
        self.alt_flag = key.startswith("alt+") or key == "alt"

        # Extract base key without modifiers
        base = self._mpl_base_key(key)

        if self._world_param.control_mode == "keyboard":
            if base.isdigit() and self.alt_flag:
                self._select_robot_id(base)

            self._press_motion_key(base)
            self._update_key_vel()

    def _on_mpl_release(self, event: Any) -> None:
        """
        Handle Matplotlib figure key release events.

        Args:
            event: Matplotlib key release event with ``event.key`` string.
        """
        key = (event.key or "").lower()
        has_shift = "shift+" in key
        base = self._mpl_base_key(key)

        if self._world_param.control_mode == "keyboard":
            self._release_motion_key(base)
            self._adjust_speed_limit(base, has_shift)

        self._run_command_key(base)
        self._update_key_vel()

    @staticmethod
    def _mpl_base_key(key: str) -> str:
        """Strip modifiers from a Matplotlib key name and normalize its spelling."""
        base = key.replace("alt+", "").replace("shift+", "").replace("ctrl+", "")
        if base == " ":
            return "space"
        if base == "esc":
            return "escape"
        return base

    # Minimal grid table formatter to avoid external dependency
    def _format_grid_table(self, headers: list[str], rows: list[list[Any]]) -> str:
        """
        Render a simple grid table using only standard Python.

        Args:
            headers: Column headers.
            rows: Data rows (list of lists).

        Returns:
            A multi-line string representing the table.
        """

        def to_str(x: Any) -> str:
            return str(x)

        cols = len(headers)
        col_widths = [len(to_str(h)) for h in headers]
        for row in rows:
            for i in range(cols):
                w = len(to_str(row[i])) if i < len(row) else 0
                if w > col_widths[i]:
                    col_widths[i] = w

        def hline(sep: str = "+") -> str:
            return sep + sep.join(["-" * (w + 2) for w in col_widths]) + sep

        def fmt_row(cells: list[str]) -> str:
            padded = [to_str(cells[i]).ljust(col_widths[i]) for i in range(cols)]
            return "| " + " | ".join(padded) + " |"

        lines: list[str] = []
        lines.append(hline("+"))
        lines.append(fmt_row(headers))
        lines.append(hline("+"))
        for r in rows:
            # ensure length
            r_norm = [to_str(r[i]) if i < len(r) else "" for i in range(cols)]
            lines.append(fmt_row(r_norm))
        lines.append(hline("+"))
        return "\n".join(lines)

    @property
    def _world_param(self):
        """Access world_param via env_ref if available."""
        if self.env_ref is not None:
            return self.env_ref._world_param
        from irsim.config import world_param

        return world_param

    @property
    def _env_param(self):
        """Access env_param via env_ref if available."""
        if self.env_ref is not None:
            return self.env_ref._env_param
        from irsim.config import env_param

        return env_param

    @property
    def logger(self) -> Any:
        """
        Get the environment logger.

        Returns:
            EnvLogger: The environment logger, or the default loguru logger
            when the control runs without an environment to borrow one from.
        """
        if self._env_param.logger is None:
            from loguru import logger

            return logger

        return self._env_param.logger

    # Window focus helpers (used to gate pynput when active_only=True)
    def _on_mpl_focus_in(self, event: Any) -> None:
        self._set_active()

    def _on_mpl_focus_out(self, event: Any) -> None:
        # Only deactivate if we are the currently active instance
        global _active_keyboard_instance
        if _active_keyboard_instance is self:
            self._is_active = False

    def close(self) -> None:
        """Release the controller: stop the pynput listener and deactivate it."""
        if self.listener is not None:
            self.listener.stop()
        self._deactivate()

    def _deactivate(self) -> None:
        global _active_keyboard_instance
        self._is_active = False
        if _active_keyboard_instance is self:
            _active_keyboard_instance = None

    def _on_mpl_close(self, event: Any) -> None:
        """Matplotlib ``close_event`` handler; the event itself is not needed."""
        self._deactivate()

    def _set_active(self) -> None:
        """Set this instance as the active keyboard controller."""
        global _active_keyboard_instance
        # Deactivate the previous active instance
        if (
            _active_keyboard_instance is not None
            and _active_keyboard_instance is not self
        ):
            _active_keyboard_instance._is_active = False
        # Activate this instance
        self._is_active = True
        _active_keyboard_instance = self
