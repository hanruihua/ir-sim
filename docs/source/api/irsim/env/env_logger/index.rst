irsim.env.env_logger
====================

.. py:module:: irsim.env.env_logger


Classes
-------

.. autoapisummary::

   irsim.env.env_logger.EnvLogger


Module Contents
---------------

.. py:class:: EnvLogger(log_file: Optional[str] = 'irsim_error.log', log_level: str = 'WARNING')

   
   Initialize the EnvLogger.

   :param log_file: Path to the log file. Default is 'irsim_error.log'.
   :type log_file: str, optional
   :param log_level: Logging level. Default is 'WARNING'.
   :type log_level: str, optional


   .. py:method:: info(msg: str) -> None

      Log an info message.

      :param msg: The message to log.
      :type msg: str



   .. py:method:: error(msg: str) -> None

      Log an error message.

      :param msg: The message to log.
      :type msg: str



   .. py:method:: debug(msg: str) -> None

      Log a debug message.

      :param msg: The message to log.
      :type msg: str



   .. py:method:: warning(msg: str) -> None

      Log a warning message.

      :param msg: The message to log.
      :type msg: str



