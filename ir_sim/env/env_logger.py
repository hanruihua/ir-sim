from loguru import logger
import sys

class EnvLogger:
    def __init__(self, log_file='ir_sim_error.log', log_level='WARNING'):
        '''
        Initialize the EnvLogger.

        Args:
            log_file (str, optional): Path to the log file. Default is 'ir_sim_error.log'.
            log_level (str, optional): Logging level. Default is 'WARNING'.
        '''
        logger.remove()
        logger.add(sys.stdout, level=log_level, format="<green>{time:YYYY-MM-DD HH:mm}</green> | " "<level>{level: <8}</level> | " "<level>{message}</level>")

        if log_file is not None:
            logger.add(log_file, level=log_level)

    def info(self, msg):
        '''
        Log an info message.

        Args:
            msg (str): The message to log.
        '''
        logger.info(msg)

    def error(self, msg):
        '''
        Log an error message.

        Args:
            msg (str): The message to log.
        '''
        logger.error(msg)

    def debug(self, msg):
        '''
        Log a debug message.

        Args:
            msg (str): The message to log.
        '''
        logger.debug(msg)

    def warning(self, msg):
        '''
        Log a warning message.

        Args:
            msg (str): The message to log.
        '''
        logger.warning(msg)