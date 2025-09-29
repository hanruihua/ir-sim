import sys
from typing import Optional

from loguru import logger


class EnvLogger:
    def __init__(
        self, log_file: Optional[str] = "irsim_error.log", log_level: str = "WARNING"
    ) -> None:
        """
        Initialize the EnvLogger.

        Args:
            log_file (str, optional): Path to the log file. Default is 'irsim_error.log'.
            log_level (str, optional): Logging level. Default is 'WARNING'.
        """
        logger.remove()
        logger.add(
            sys.stdout,
            level=log_level,
            format="<green>{time:YYYY-MM-DD HH:mm}</green> | "
            "<level>{level: <8}</level> | "
            "<level>{message}</level>",
        )

        if log_file is not None:
            logger.add(log_file, level=log_level)

    def trace(self, msg: str) -> None:
        """
        Log a trace message.

        Args:
            msg (str): The message to log.
        """
        logger.trace(msg)

    def info(self, msg: str) -> None:
        """
        Log an info message.

        Args:
            msg (str): The message to log.
        """
        logger.info(msg)

    def error(self, msg: str) -> None:
        """
        Log an error message.

        Args:
            msg (str): The message to log.
        """
        logger.error(msg)

    def debug(self, msg: str) -> None:
        """
        Log a debug message.

        Args:
            msg (str): The message to log.
        """
        logger.debug(msg)

    def warning(self, msg: str) -> None:
        """
        Log a warning message.

        Args:
            msg (str): The message to log.
        """
        logger.warning(msg)

    def success(self, msg: str) -> None:
        """
        Log a success message.

        Args:
            msg (str): The message to log.
        """
        logger.success(msg)

    def critical(self, msg: str) -> None:
        """
        Log a critical message.

        Args:
            msg (str): The message to log.
        """
        logger.critical(msg)
