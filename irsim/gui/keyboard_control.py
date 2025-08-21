from typing import Any, Optional

import matplotlib.pyplot as plt
import numpy as np
from pynput import keyboard
from tabulate import tabulate

from irsim.config import env_param


class KeyboardControl:
    def __init__(self, env_ref: Optional[Any] = None, **keyboard_kwargs: Any) -> None:
        """
        Initialize keyboard control for the environment.

        Args:
            env_ref: Reference to the environment instance to access reset functionality
            keyboard_kwargs (dict): Dictionary of keyword arguments for keyboard control settings

                - vel_max (list): Maximum velocities [linear, angular]. Default is [3.0, 1.0].

                - key_lv_max (float): Maximum linear velocity. Default is vel_max [0].

                - key_ang_max (float): Maximum angular velocity. Default is vel_max [1].

                - key_lv (float): Initial linear velocity. Default is 0.0.

                - key_ang (float): Initial angular velocity. Default is 0.0.

                - key_id (int): Initial robot control ID. Default is 0.

            Keys:
                - w: Move forward.
                - s: Move backward.
                - a: Turn left.
                - d: Turn right.
                - q: Decrease linear velocity.
                - e: Increase linear velocity.
                - z: Decrease angular velocity.
                - c: Increase angular velocity.
                - alt + num: Change the current control robot id.
                - r: Reset the environment.
                - space: pause/resume the environment.
        """

        # Store environment reference for reset functionality
        self.env_ref = env_ref

        vel_max = keyboard_kwargs.get("vel_max", [1.0, 1.0])
        self.key_lv_max = keyboard_kwargs.get("key_lv_max", vel_max[0])
        self.key_ang_max = keyboard_kwargs.get("key_ang_max", vel_max[1])
        self.key_lv = keyboard_kwargs.get("key_lv", 0.0)
        self.key_ang = keyboard_kwargs.get("key_ang", 0.0)
        self.key_id = keyboard_kwargs.get("key_id", 0)
        self.alt_flag = 0

        if "s" in plt.rcParams["keymap.save"]:
            plt.rcParams["keymap.save"].remove("s")

        if "q" in plt.rcParams["keymap.quit"]:
            plt.rcParams["keymap.quit"].remove("q")

        self.key_vel = np.zeros((2, 1))

        self.logger.info("start to keyboard control")

        commands = [
            ["w", "forward"],
            ["s", "back forward"],
            ["a", "turn left"],
            ["d", "turn right"],
            ["q", "decrease linear velocity"],
            ["e", "increase linear velocity"],
            ["z", "decrease angular velocity"],
            ["c", "increase angular velocity"],
            ["alt+num", "change current control robot id"],
            ["r", "reset the environment"],
            ["space", "pause/resume the environment"],
        ]
        # headers = ["key", "function"]

        headers = ["Key", "Function"]
        # Generate the table using tabulate
        table = tabulate(commands, headers=headers, tablefmt="grid")
        print(table)

        self.listener = keyboard.Listener(
            on_press=self._on_press, on_release=self._on_release
        )
        self.listener.start()

    def _on_press(self, key: keyboard.Key) -> None:
        """
        Handle key press events for keyboard control.

        Args:
            key (pynput.keyboard.Key): The key that was pressed.
        """

        try:
            if key.char.isdigit() and self.alt_flag:
                if self.env_ref and int(key.char) >= self.env_ref.robot_number:
                    print("out of number of robots")
                    self.key_id = int(key.char)
                else:
                    print("current control id: ", int(key.char))
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
            try:
                if "alt" in key.name:
                    self.alt_flag = True

            except AttributeError:
                if key.char.isdigit() and self.alt_flag:
                    if self.env_ref and int(key.char) >= self.env_ref.robot_number:
                        print("out of number of robots")
                        self.key_id = int(key.char)
                    else:
                        print("current control id: ", int(key.char))
                        self.key_id = int(key.char)

    def _on_release(self, key: keyboard.Key) -> None:
        """
        Handle key release events for keyboard control.

        Args:
            key (pynput.keyboard.Key): The key that was released.
        """

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
                print("current linear velocity", self.key_lv_max)
            if key.char == "e":
                self.key_lv_max = self.key_lv_max + 0.2
                print("current linear velocity", self.key_lv_max)

            if key.char == "z":
                self.key_ang_max = self.key_ang_max - 0.2
                print("current angular velocity ", self.key_ang_max)
            if key.char == "c":
                self.key_ang_max = self.key_ang_max + 0.2
                print("current angular velocity ", self.key_ang_max)

            if key.char == "r":
                print("reset the environment")
                if self.env_ref is not None:
                    self.env_ref.reset()
                else:
                    self.logger.warning("Environment reference not set. Cannot reset.")

            self.key_vel = np.array([[self.key_lv], [self.key_ang]])

        except AttributeError:
            if "alt" in key.name:
                self.alt_flag = False

            if key == keyboard.Key.space:
                if self.env_ref.status == "Running":
                    self.logger.info("pause the environment")
                    self.env_ref.pause()
                else:
                    self.logger.info("resume the environment")
                    self.env_ref.resume()

    @property
    def logger(self) -> Any:
        """
        Get the environment logger.

        Returns:
            EnvLogger: The logger instance for the environment.
        """
        return env_param.logger
