from typing import TYPE_CHECKING, Any, Optional

import matplotlib.pyplot as plt
import numpy as np

if TYPE_CHECKING:
    pass

# Optional pynput import (allows fallback to Matplotlib key events)
# Predeclare for type checkers
keyboard: Optional[Any] = None
try:  # pragma: no cover - availability depends on environment
    from pynput import keyboard as _pynput_keyboard

    if getattr(_pynput_keyboard, "Listener", None) is not None:
        keyboard = _pynput_keyboard
        _PYNPUT_AVAILABLE = True
    else:
        _PYNPUT_AVAILABLE = False
except Exception:  # pragma: no cover
    _PYNPUT_AVAILABLE = False


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
        - Environment keys (space to pause/resume, r to reset, esc to quit) are always active.

    Key mappings (both backends):
        - w/s: Increase/decrease linear velocity (forward/backward)
        - a/d: Increase/decrease angular velocity (turn left/right)
        - q/e: Decrease/increase maximum linear velocity (key_lv_max)
        - z/c: Decrease/increase maximum angular velocity (key_ang_max)
        - alt + number: Change current controlled robot id
        - r: Reset the environment (if ``env_ref`` provided)
        - space: Toggle pause/resume
        - esc: Quit the environment immediately (closes figure and raises ``SystemExit(0)``)
        - x: Switch between keyboard and auto control modes
        - l: Reload the environment
        - F5: Debug the environment

    Notes:
        - The "mpl" backend requires the Matplotlib figure window to have focus to receive key events.
        - In keyboard control mode, behavior planners are ignored; robot motion follows key input.
    """

    def __init__(self, env_ref: Optional[Any] = None, **keyboard_kwargs: Any) -> None:
        """
        Initialize keyboard control for the environment.

        Args:
            env_ref: Reference to the environment instance. Used for pause/resume and reset.
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
            - a: Turn left
            - d: Turn right
            - q/e: Decrease/Increase linear velocity
            - z/c: Decrease/Increase angular velocity
            - alt + number: Change current control robot id
            - r: Reset the environment (if ``env_ref`` provided)
            - space: Pause/Resume the environment
            - esc: Quit the environment
            - x: Switch between keyboard and auto control modes
            - l: Reload the environment
            - F5: Debug the environment
        """

        # Store environment reference for reset functionality
        self.env_ref = env_ref

        self.key_lv_max = keyboard_kwargs.get("key_lv_max", 3.0)
        self.key_ang_max = keyboard_kwargs.get("key_ang_max", 1.0)
        self.key_lv = keyboard_kwargs.get("key_lv", 0.0)
        self.key_ang = keyboard_kwargs.get("key_ang", 0.0)
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

        self.key_vel = np.zeros((2, 1))

        if self._world_param.control_mode == "keyboard":
            self.logger.info("start to keyboard control")

            commands = [
                ["w", "forward"],
                ["s", "backward"],
                ["a", "turn left"],
                ["d", "turn right"],
                ["q", "decrease linear velocity"],
                ["e", "increase linear velocity"],
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
            ]

            headers = ["Key", "Function"]
            print(self._format_grid_table(headers, commands))
        # else:
        #     commands = [
        #         ["r", "reset the environment"],
        #         ["space", "pause/resume the environment"],
        #         ["esc", "quit the environment"],
        #         ["x", "switch keyboard control and auto control"],
        #         ["l", "reload the environment"],
        #     ]

        if self.backend == "pynput" and not _PYNPUT_AVAILABLE:
            self.logger.warning("pynput is not available. Using matplotlib backend.")
            self.backend = "mpl"

        # Track Matplotlib window focus/enter/leave to gate pynput callbacks
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

        if self.backend == "pynput" and _PYNPUT_AVAILABLE:
            # Use pynput global keyboard listener
            self.listener = keyboard.Listener(
                on_press=self._on_pynput_press, on_release=self._on_pynput_release
            )
            self.listener.start()

        else:
            # Fallback to matplotlib figure key events
            fig = plt.gcf()
            self._mpl_press_cid = fig.canvas.mpl_connect(
                "key_press_event", self._on_mpl_press
            )
            self._mpl_release_cid = fig.canvas.mpl_connect(
                "key_release_event", self._on_mpl_release
            )

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
                    if self.env_ref and int(key.char) >= self.env_ref.robot_number:
                        self.logger.warning(
                            f"{int(key.char)} over the maximum id: {self.env_ref.robot_number - 1}"
                        )
                        self.key_id = int(key.char)
                    else:
                        self.logger.info(f"Current control id: {int(key.char)}")
                        self.key_id = int(key.char)

                if key.char == "w":
                    self.key_lv = self.key_lv_max
                if key.char == "s":
                    self.key_lv = -self.key_lv_max
                if key.char == "a":
                    self.key_ang = self.key_ang_max
                if key.char == "d":
                    self.key_ang = -self.key_ang_max

                self.key_vel = np.array([[self.key_lv], [self.key_ang]])

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
            if key.char == "w":
                self.key_lv = 0
            if key.char == "s":
                self.key_lv = 0
            if key.char == "a":
                self.key_ang = 0
            if key.char == "d":
                self.key_ang = 0
            if key.char == "q":
                self.key_lv_max = self.key_lv_max - 0.2
                self.logger.info(f"current linear velocity: {self.key_lv_max}")
            if key.char == "e":
                self.key_lv_max = self.key_lv_max + 0.2
                self.logger.info(f"current linear velocity: {self.key_lv_max}")

            if key.char == "z":
                self.key_ang_max = self.key_ang_max - 0.2
                self.logger.info(f"current angular velocity: {self.key_ang_max}")
            if key.char == "c":
                self.key_ang_max = self.key_ang_max + 0.2
                self.logger.info(f"current angular velocity: {self.key_ang_max}")

            if key.char == "r":
                self.logger.info("reset the environment")
                if self.env_ref is not None:
                    self.env_ref.reset_flag = True
                else:
                    self.logger.warning("Environment reference not set. Cannot reset.")

            # Switch control mode with 'x'
            if key.char == "x":
                if self._world_param.control_mode == "keyboard":
                    self._world_param.control_mode = "auto"
                    self.logger.info("switch to auto control")
                else:
                    self._world_param.control_mode = "keyboard"
                    self.logger.info("switch to keyboard control")

            if key.char == "v":
                self.logger.info("save the figure")
                self.env_ref.save_figure_flag = True

            if key.char == "l":
                self.env_ref.reload_flag = True
                self.logger.info("reload the environment")

            self.key_vel = np.array([[self.key_lv], [self.key_ang]])

        except AttributeError:
            if "alt" in key.name:
                self.alt_flag = False

            if keyboard is not None and key == keyboard.Key.space:
                if "Pause" not in self.env_ref.status:
                    self.logger.info("pause the environment")
                    self.env_ref.pause()
                else:
                    self.logger.info("resume the environment")
                    self.env_ref.resume()

            # Single-step debug on F5
            if keyboard is not None and key == keyboard.Key.f5:
                if not self.env_ref.debug_flag:
                    self.env_ref.debug_flag = True
                    self.env_ref.debug_count = self._world_param.count
                    self.env_ref.pause_flag = False
                else:
                    self.env_ref.debug_count += 1

            # Quit environment on ESC
            if keyboard is not None and key == keyboard.Key.esc:
                self.env_ref.quit_flag = True

    # Matplotlib key event handlers (backend = 'mpl')
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
        base = key.replace("alt+", "").replace("shift+", "").replace("ctrl+", "")

        if self._world_param.control_mode == "keyboard":
            if base.isdigit() and self.alt_flag:
                if self.env_ref and int(base) >= self.env_ref.robot_number:
                    self.logger.warning(
                        f"{int(base)} over the maximum id: {self.env_ref.robot_number - 1}"
                    )
                    self.key_id = int(base)
                else:
                    self.logger.info(f"Current control id: {int(base)}")
                    self.key_id = int(base)

            if base == "w":
                self.key_lv = self.key_lv_max
            if base == "s":
                self.key_lv = -self.key_lv_max
            if base == "a":
                self.key_ang = self.key_ang_max
            if base == "d":
                self.key_ang = -self.key_ang_max

            self.key_vel = np.array([[self.key_lv], [self.key_ang]])

    def _on_mpl_release(self, event: Any) -> None:
        """
        Handle Matplotlib figure key release events.

        Args:
            event: Matplotlib key release event with ``event.key`` string.
        """
        key = (event.key or "").lower()
        base = key.replace("alt+", "").replace("shift+", "").replace("ctrl+", "")

        if self._world_param.control_mode == "keyboard":
            if base == "w":
                self.key_lv = 0
            if base == "s":
                self.key_lv = 0
            if base == "a":
                self.key_ang = 0
            if base == "d":
                self.key_ang = 0
            if base == "q":
                self.key_lv_max = self.key_lv_max - 0.2
                self.logger.info(f"current linear velocity: {self.key_lv_max}")
            if base == "e":
                self.key_lv_max = self.key_lv_max + 0.2
                self.logger.info(f"current linear velocity: {self.key_lv_max}")
            if base == "z":
                self.key_ang_max = self.key_ang_max - 0.2
                self.logger.info(f"current angular velocity: {self.key_ang_max}")
            if base == "c":
                self.key_ang_max = self.key_ang_max + 0.2
                self.logger.info(f"current angular velocity: {self.key_ang_max}")

        if base == "r":
            self.logger.info("reset the environment")
            if self.env_ref is not None:
                self.env_ref.reset_flag = True
            else:
                self.logger.warning("Environment reference not set. Cannot reset.")

        if base in ("space", " "):
            if "Pause" not in self.env_ref.status:
                self.logger.info("pause the environment")
                self.env_ref.pause()
            else:
                self.logger.info("resume the environment")
                self.env_ref.resume()

        # Switch control mode with 'x'
        if base == "x":
            if self._world_param.control_mode == "keyboard":
                self._world_param.control_mode = "auto"
                self.logger.info("switch to auto control")
            else:
                self._world_param.control_mode = "keyboard"
                self.logger.info("switch to keyboard control")

        if base == "l":
            self.env_ref.reload_flag = True
            self.logger.info("reload the environment")

        # Single-step debug on F5
        if base == "f5":
            if not self.env_ref.debug_flag:
                self.env_ref.debug_flag = True
                self.env_ref.debug_count = self._world_param.count
                self.env_ref.pause_flag = False
            else:
                self.env_ref.debug_count += 1

        if base == "v":
            self.logger.info("save the figure")
            self.env_ref.save_figure_flag = True

        # Quit environment on ESC/escape
        if base in ("escape", "esc"):
            self.env_ref.quit_flag = True

        self.key_vel = np.array([[self.key_lv], [self.key_ang]])

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
            EnvLogger: The logger instance for the environment.
        """
        return self._env_param.logger

    # Window focus helpers (used to gate pynput when active_only=True)
    def _on_mpl_focus_in(self, event: Any) -> None:
        self._is_active = True

    def _on_mpl_focus_out(self, event: Any) -> None:
        self._is_active = False

    def _on_mpl_close(self, event: Any) -> None:
        self._is_active = False
