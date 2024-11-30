import numpy as np
from irsim.lib.algorithm.kinematics import (
    differential_kinematics,
    ackermann_kinematics,
    omni_kinematics,
)

class kinematics_handler:
    """
    This class is the handler for the kinematics of the robot. All the kinematics related functions are defined here. The step function is used to calculate the next state of the robot based on the current state and the velocity vector. Support kinematics include 'diff', 'acker', 'omni', and 'custom'.

    Args:
        name (str): Name of the robot.
        noise (bool): Whether to add noise (default False).
        alpha (list): Noise parameters.
        **kwargs: Additional parameters for the acker kinematics.
            wheelbase: Wheelbase for acker kinematics (default 1.0).
            mode: Mode for acker kinematics (default "steer", options: "steer", "angular").
    """

    def __init__(self, name, noise=False, alpha=[], **kwargs) -> None:

        self.name = name
        self.noise = noise
        self.alpha = alpha
        self.wheelbase = kwargs.get("wheelbase", 1.0)
        self.mode = kwargs.get("mode", "steer")

    def step(
        self,
        state,
        velocity,
        step_time,
    ):
        """
        Calculate the next state using the kinematics model.

        Args:
            state (np.ndarray): Current state.
            velocity (np.ndarray): Velocity vector.
            step_time (float): Time step for simulation.
            noise (bool): Whether to add noise (default False).
            alpha (list): Noise parameters.

        Returns:
            np.ndarray: Next state.
        """

        if self.name == "omni":
            next_state = omni_kinematics(
                state, velocity, step_time, self.noise, self.alpha
            )
        
        elif self.name == "diff":
            next_state = differential_kinematics(
                state, velocity, step_time, self.noise, self.alpha
            )
        
        elif self.name == "acker":

            next_state = ackermann_kinematics(
                state,
                velocity,
                step_time,
                self.noise,
                self.alpha,
                self.mode,
                self.wheelbase
            )

        elif self.kinematics == "custom":
            raise NotImplementedError("custom kinematics is not implemented")

        else:
            raise ValueError(
                "kinematics should be one of the following: omni, diff, acker"
            )

        return next_state
