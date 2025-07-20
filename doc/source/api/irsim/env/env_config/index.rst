irsim.env.env_config
====================

.. py:module:: irsim.env.env_config


Classes
-------

.. autoapisummary::

   irsim.env.env_config.EnvConfig


Module Contents
---------------

.. py:class:: EnvConfig(world_name)

   The base class of environment parameters read from yaml file.
       basic categories: world, plot, robot, obstacle


   .. py:property:: parse

      The parsed kwargs from the yaml file.


   .. py:property:: logger

      Get the logger of the env_param.


