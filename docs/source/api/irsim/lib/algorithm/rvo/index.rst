irsim.lib.algorithm.rvo
=======================

.. py:module:: irsim.lib.algorithm.rvo

.. autoapi-nested-parse::

   This file is the implementation of the Reciprocal Velocity Obstacle (RVO) algorithm for multi-robot collision avoidance.

   Author: Ruihua Han

   reference: https://github.com/MengGuo/RVO_Py_MAS



Classes
-------

.. autoapisummary::

   irsim.lib.algorithm.rvo.reciprocal_vel_obs


Module Contents
---------------

.. py:class:: reciprocal_vel_obs(state: list, obs_state_list=None, vxmax=1.5, vymax=1.5, acce=0.5, factor=1.0)

   A class to implement the Reciprocal Velocity Obstacle (RVO) algorithm for multi-robot collision avoidance.

   :param state: The rvo state of the agent [x, y, vx, vy, radius, vx_des, vy_des].
   :type state: list
   :param obs_state_list: List of states of static obstacles [[x, y, vx, vy, radius]].
   :type obs_state_list: list
   :param vxmax: Maximum velocity in the x direction.
   :type vxmax: float
   :param vymax: Maximum velocity in the y direction.
   :type vymax: float
   :param acce: Acceleration limit.
   :type acce: float


   .. py:attribute:: state


   .. py:attribute:: obs_state_list
      :value: None



   .. py:attribute:: vxmax
      :value: 1.5



   .. py:attribute:: vymax
      :value: 1.5



   .. py:attribute:: acce
      :value: 0.5



   .. py:attribute:: factor
      :value: 1.0



   .. py:method:: update(state, obs_state_list)


   .. py:method:: cal_vel(mode='rvo')

      Calculate the velocity of the agent based on the Reciprocal Velocity Obstacle (RVO) algorithm.

      :param mode: The vo configure to calculate the velocity. It can be "rvo", "hrvo", or "vo".
                   - rvo: Reciprocal Velocity Obstacle (RVO) algorithm, for multi-robot collision avoidance.
                   - hrvo: Hybrid Reciprocal Velocity Obstacle (HRVO) algorithm, for multi-robot collision avoidance.
                   - vo: Velocity Obstacle (VO) algorithm, for obstacle-robot collision avoidance.
      :type mode: str



   .. py:method:: config_rvo()


   .. py:method:: config_rvo_mode(obstacle)


   .. py:method:: config_hrvo()


   .. py:method:: config_hrvo_mode(obstacle)


   .. py:method:: config_vo()


   .. py:method:: config_vo_mode(obstacle)


   .. py:method:: vel_candidate(rvo_list)


   .. py:method:: vo_out(vx, vy, rvo_list)


   .. py:method:: vel_select(vo_outside, vo_inside)


   .. py:method:: penalty(vel, vel_des, factor)


   .. py:method:: between_vector(line_left_vector, line_right_vector, line_vector)
      :staticmethod:



   .. py:method:: cross_product(vector1, vector2)
      :staticmethod:



