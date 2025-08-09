irsim.lib.algorithm.kinematics
==============================

.. py:module:: irsim.lib.algorithm.kinematics

.. autoapi-nested-parse::

   This file is the implementation of the kinematics for different robots.

   reference: Lynch, Kevin M., and Frank C. Park. Modern Robotics: Mechanics, Planning, and Control. 1st ed. Cambridge, MA: Cambridge University Press, 2017.



Functions
---------

.. autoapisummary::

   irsim.lib.algorithm.kinematics.differential_kinematics
   irsim.lib.algorithm.kinematics.ackermann_kinematics
   irsim.lib.algorithm.kinematics.omni_kinematics


Module Contents
---------------

.. py:function:: differential_kinematics(state: numpy.ndarray, velocity: numpy.ndarray, step_time: float, noise: bool = False, alpha: Optional[list[float]] = None) -> numpy.ndarray

   Calculate the next state for a differential wheel robot.

   :param state: A 3x1 vector [x, y, theta] representing the current position and orientation.
   :param velocity: A 2x1 vector [linear, angular] representing the current velocities.
   :param step_time: The time step for the simulation.
   :param noise: Boolean indicating whether to add noise to the velocity (default False).
   :param alpha: List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]). alpha[0] and alpha[1] are for linear velocity, alpha[2] and alpha[3] are for angular velocity.

   :returns: A 3x1 vector [x, y, theta] representing the next state.
   :rtype: next_state


.. py:function:: ackermann_kinematics(state: numpy.ndarray, velocity: numpy.ndarray, step_time: float, noise: bool = False, alpha: Optional[list[float]] = None, mode: str = 'steer', wheelbase: float = 1) -> numpy.ndarray

   Calculate the next state for an Ackermann steering vehicle.

   :param state: A 4x1 vector [x, y, theta, steer_angle] representing the current state.
   :param velocity: A 2x1 vector representing the current velocities, format depends on mode.
                    For "steer" mode, [linear, steer_angle] is expected.
                    For "angular" mode, [linear, angular] is expected.
   :param step_time: The time step for the simulation.
   :param noise: Boolean indicating whether to add noise to the velocity (default False).
   :param alpha: List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]). alpha[0] and alpha[1] are for linear velocity, alpha[2] and alpha[3] are for angular velocity.
   :param mode: The kinematic mode, either "steer" or "angular" (default "steer").
   :param wheelbase: The distance between the front and rear axles (default 1).

   :returns: A 4x1 vector representing the next state.
   :rtype: new_state


.. py:function:: omni_kinematics(state: numpy.ndarray, velocity: numpy.ndarray, step_time: float, noise: bool = False, alpha: Optional[list[float]] = None) -> numpy.ndarray

   Calculate the next position for an omnidirectional robot.

   :param state: A 2x1 vector [x, y] representing the current position.
   :param velocity: A 2x1 vector [vx, vy] representing the current velocities.
   :param step_time: The time step for the simulation.
   :param noise: Boolean indicating whether to add noise to the velocity (default False).
   :param alpha: List of noise parameters for the velocity model (default [0.03, 0.03]). alpha[0] is for x velocity, alpha[1] is for y velocity.

   :returns: A 2x1 vector [x, y] representing the next position.
   :rtype: new_position


