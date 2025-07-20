irsim.gui.keyboard_control
==========================

.. py:module:: irsim.gui.keyboard_control


Classes
-------

.. autoapisummary::

   irsim.gui.keyboard_control.KeyboardControl


Module Contents
---------------

.. py:class:: KeyboardControl(env_ref=None, **keyboard_kwargs)

   
   Initialize keyboard control for the environment.

   :param env_ref: Reference to the environment instance to access reset functionality
   :param keyboard_kwargs: Dictionary of keyword arguments for keyboard control settings

                           - vel_max (list): Maximum velocities [linear, angular]. Default is [3.0, 1.0].

                           - key_lv_max (float): Maximum linear velocity. Default is vel_max [0].

                           - key_ang_max (float): Maximum angular velocity. Default is vel_max [1].

                           - key_lv (float): Initial linear velocity. Default is 0.0.

                           - key_ang (float): Initial angular velocity. Default is 0.0.

                           - key_id (int): Initial robot control ID. Default is 0.
   :type keyboard_kwargs: dict
   :param Keys:
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


   .. py:attribute:: env_ref
      :value: None



   .. py:attribute:: key_lv_max


   .. py:attribute:: key_ang_max


   .. py:attribute:: key_lv


   .. py:attribute:: key_ang


   .. py:attribute:: key_id


   .. py:attribute:: alt_flag
      :value: 0



   .. py:attribute:: key_vel


   .. py:attribute:: listener


   .. py:property:: logger

      Get the environment logger.

      :returns: The logger instance for the environment.
      :rtype: EnvLogger


