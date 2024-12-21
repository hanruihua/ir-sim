import numpy as np
from abc import ABC, abstractmethod
from irsim.lib.algorithm.kinematics import (
    differential_kinematics,
    ackermann_kinematics,
    omni_kinematics,
    # rigid3d_kinematics,
)


class KinematicsHandler(ABC):
    """
    Abstract base class for handling robot kinematics.
    """

    def __init__(self, name, noise: bool = False, alpha: list = None):

        '''
        Initialize the KinematicsHandler class.
        
        Args:
            noise (bool): Boolean indicating whether to add noise to the velocity (default False).
            alpha (list): List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]).
        '''

        self.name = name
        self.noise = noise
        self.alpha = alpha or [0.03, 0, 0, 0.03]

    @abstractmethod
    def step(self, state: np.ndarray, velocity: np.ndarray, step_time: float) -> np.ndarray:
        """
        Calculate the next state using the kinematics model.

        Args:
            state (np.ndarray): Current state.
            velocity (np.ndarray): Velocity vector.
            step_time (float): Time step for simulation.

        Returns:
            np.ndarray: Next state.
        """
        pass


class OmniKinematics(KinematicsHandler):

    def __init__(self, name, noise, alpha):
        super().__init__(name, noise, alpha)

    def step(self, state: np.ndarray, velocity: np.ndarray, step_time: float) -> np.ndarray:
        next_state = omni_kinematics(state, velocity, step_time, self.noise, self.alpha)
        return next_state


class DifferentialKinematics(KinematicsHandler):

    def __init__(self, name, noise, alpha):
        super(DifferentialKinematics, self).__init__(name, noise, alpha)

    def step(self, state: np.ndarray, velocity: np.ndarray, step_time: float) -> np.ndarray:
        next_state = differential_kinematics(state, velocity, step_time, self.noise, self.alpha)
        return next_state


class AckermannKinematics(KinematicsHandler):

    def __init__(self, name, noise: bool = False, alpha: list = None, mode: str = "steer", wheelbase: float = 1.0):
        super().__init__(name, noise, alpha)
        self.mode = mode
        self.wheelbase = wheelbase

    def step(self, state: np.ndarray, velocity: np.ndarray, step_time: float) -> np.ndarray:
        next_state = ackermann_kinematics(
            state, velocity, step_time, self.noise, self.alpha, self.mode, self.wheelbase
        )
        return next_state

# class Rigid3DKinematics(KinematicsHandler):
    
#     def __init__(self, name, noise, alpha):
#         super().__init__(name, noise, alpha)

#     def step(self, state: np.ndarray, velocity: np.ndarray, step_time: float) -> np.ndarray:
#         next_state = rigid3d_kinematics(state, velocity, step_time, self.noise, self.alpha)
#         return next_state

class KinematicsFactory:
    """
    Factory class to create kinematics handlers.
    """

    @staticmethod
    def create_kinematics(
        name: str = None,
        noise: bool = False,
        alpha: list = None,
        mode: str = 'steer',
        wheelbase: float = None,
    ) -> KinematicsHandler:
        name = name.lower() if name else None
        if name == "omni":
            return OmniKinematics(name, noise, alpha)
        elif name == "diff":
            return DifferentialKinematics(name, noise, alpha)
        elif name == "acker":
            return AckermannKinematics(name, noise, alpha, mode, wheelbase)
        elif name == 'rigid3d':
            return Rigid3DKinematics(name, noise, alpha)
        else:
            print(f"Unknown kinematics type: {name}, object will be stationary.")
            return None
            