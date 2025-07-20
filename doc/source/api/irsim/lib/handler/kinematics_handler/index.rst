irsim.lib.handler.kinematics_handler
====================================

.. py:module:: irsim.lib.handler.kinematics_handler


Classes
-------

.. autoapisummary::

   irsim.lib.handler.kinematics_handler.KinematicsHandler
   irsim.lib.handler.kinematics_handler.OmniKinematics
   irsim.lib.handler.kinematics_handler.DifferentialKinematics
   irsim.lib.handler.kinematics_handler.AckermannKinematics
   irsim.lib.handler.kinematics_handler.KinematicsFactory


Module Contents
---------------

.. py:class:: KinematicsHandler(name, noise: bool = False, alpha: list = None)

   Bases: :py:obj:`abc.ABC`


   Abstract base class for handling robot kinematics.

   Initialize the KinematicsHandler class.

   :param noise: Boolean indicating whether to add noise to the velocity (default False).
   :type noise: bool
   :param alpha: List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]).
   :type alpha: list


   .. py:attribute:: name


   .. py:attribute:: noise
      :value: False



   .. py:attribute:: alpha
      :value: [0.03, 0, 0, 0.03]



   .. py:method:: step(state: numpy.ndarray, velocity: numpy.ndarray, step_time: float) -> numpy.ndarray
      :abstractmethod:


      Calculate the next state using the kinematics model.

      :param state: Current state.
      :type state: np.ndarray
      :param velocity: Velocity vector.
      :type velocity: np.ndarray
      :param step_time: Time step for simulation.
      :type step_time: float

      :returns: Next state.
      :rtype: np.ndarray



.. py:class:: OmniKinematics(name, noise, alpha)

   Bases: :py:obj:`KinematicsHandler`


   Abstract base class for handling robot kinematics.

   Initialize the KinematicsHandler class.

   :param noise: Boolean indicating whether to add noise to the velocity (default False).
   :type noise: bool
   :param alpha: List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]).
   :type alpha: list


   .. py:method:: step(state: numpy.ndarray, velocity: numpy.ndarray, step_time: float) -> numpy.ndarray

      Calculate the next state using the kinematics model.

      :param state: Current state.
      :type state: np.ndarray
      :param velocity: Velocity vector.
      :type velocity: np.ndarray
      :param step_time: Time step for simulation.
      :type step_time: float

      :returns: Next state.
      :rtype: np.ndarray



.. py:class:: DifferentialKinematics(name, noise, alpha)

   Bases: :py:obj:`KinematicsHandler`


   Abstract base class for handling robot kinematics.

   Initialize the KinematicsHandler class.

   :param noise: Boolean indicating whether to add noise to the velocity (default False).
   :type noise: bool
   :param alpha: List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]).
   :type alpha: list


   .. py:method:: step(state: numpy.ndarray, velocity: numpy.ndarray, step_time: float) -> numpy.ndarray

      Calculate the next state using the kinematics model.

      :param state: Current state.
      :type state: np.ndarray
      :param velocity: Velocity vector.
      :type velocity: np.ndarray
      :param step_time: Time step for simulation.
      :type step_time: float

      :returns: Next state.
      :rtype: np.ndarray



.. py:class:: AckermannKinematics(name, noise: bool = False, alpha: list = None, mode: str = 'steer', wheelbase: float = 1.0)

   Bases: :py:obj:`KinematicsHandler`


   Abstract base class for handling robot kinematics.

   Initialize the KinematicsHandler class.

   :param noise: Boolean indicating whether to add noise to the velocity (default False).
   :type noise: bool
   :param alpha: List of noise parameters for the velocity model (default [0.03, 0, 0, 0.03]).
   :type alpha: list


   .. py:attribute:: mode
      :value: 'steer'



   .. py:attribute:: wheelbase
      :value: 1.0



   .. py:method:: step(state: numpy.ndarray, velocity: numpy.ndarray, step_time: float) -> numpy.ndarray

      Calculate the next state using the kinematics model.

      :param state: Current state.
      :type state: np.ndarray
      :param velocity: Velocity vector.
      :type velocity: np.ndarray
      :param step_time: Time step for simulation.
      :type step_time: float

      :returns: Next state.
      :rtype: np.ndarray



.. py:class:: KinematicsFactory

   Factory class to create kinematics handlers.


   .. py:method:: create_kinematics(name: str = None, noise: bool = False, alpha: list = None, mode: str = 'steer', wheelbase: float = None, role: str = 'robot') -> KinematicsHandler
      :staticmethod:



