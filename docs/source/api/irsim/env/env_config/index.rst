irsim.env.env_config
====================

.. py:module:: irsim.env.env_config


Classes
-------

.. autoapisummary::

   irsim.env.env_config.EnvConfig


Module Contents
---------------

.. py:class:: EnvConfig(world_name: Optional[str])

   The base class of environment parameters read from yaml file.
       basic categories: world, plot, robot, obstacle


   .. py:property:: parse
      :type: dict[str, Any]


      The parsed kwargs from the yaml file.


   .. py:property:: logger

      Get the logger of the env_param.


