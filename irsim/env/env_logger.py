import contextlib
import itertools
import sys
import weakref
from collections.abc import Callable
from typing import Any

from loguru import logger


class EnvLogger:
    """Thin wrapper around Loguru used by IR-SIM environments.

    Each environment owns a console sink at its ``log_level`` and, when
    ``log_file`` is given, a file sink. Its messages are tagged and its sinks
    accept only that tag, so a second environment in the same process neither
    replaces the first one's sinks nor receives its messages. Sinks added by
    user code are left alone. ``close`` removes the sinks; garbage collection
    does the same.
    """

    CONSOLE_FORMAT = (
        "<green>{time:YYYY-MM-DD HH:mm}</green> | "
        "<level>{level: <8}</level> | "
        "<level>{message}</level>"
    )
    _tags = itertools.count()
    _default_handler_dropped = False

    def __init__(
        self, log_file: str | None = "irsim_error.log", log_level: str = "WARNING"
    ) -> None:
        """
        Initialize the EnvLogger.

        Args:
            log_file (str, optional): Path to the log file. Default is 'irsim_error.log'.
            log_level (str, optional): Logging level. Default is 'WARNING'.
        """
        self._drop_loguru_default_handler()
        self._tag = next(self._tags)
        self._logger = logger.bind(irsim_env=self._tag)
        self._sink_ids = self._add_sinks(log_file, log_level)
        self._finalizer = weakref.finalize(
            self, self._remove_sinks, list(self._sink_ids)
        )
        self._once_keys: set[str] = set()

    @classmethod
    def _drop_loguru_default_handler(cls) -> None:
        """Remove loguru's own stderr handler once; every environment adds its own sinks."""
        if cls._default_handler_dropped:
            return
        with contextlib.suppress(ValueError):
            logger.remove(0)
        cls._default_handler_dropped = True

    @staticmethod
    def _messages_of(tag: int) -> Callable[[Any], bool]:
        """Sink filter accepting the messages of one environment.

        Messages logged through a plain loguru logger carry no tag and are
        accepted by every environment's sinks.
        """

        def accept(record: Any) -> bool:
            return record["extra"].get("irsim_env", tag) == tag

        return accept

    def _add_sinks(self, log_file: str | None, log_level: str) -> list[int]:
        """Add this environment's console sink and optional file sink."""
        accept = self._messages_of(self._tag)
        sink_ids = [
            logger.add(
                sys.stdout, level=log_level, format=self.CONSOLE_FORMAT, filter=accept
            )
        ]
        if log_file is not None:
            sink_ids.append(logger.add(log_file, level=log_level, filter=accept))
        return sink_ids

    @staticmethod
    def _remove_sinks(sink_ids: list[int]) -> None:
        for sink_id in sink_ids:
            with contextlib.suppress(ValueError):
                logger.remove(sink_id)

    def close(self) -> None:
        """Remove this environment's sinks; later messages go nowhere."""
        self._finalizer()
        self._sink_ids = []

    def trace(self, msg: str) -> None:
        """
        Log a trace message.

        Args:
            msg (str): The message to log.
        """
        self._logger.trace(msg)

    def info(self, msg: str) -> None:
        """
        Log an info message.

        Args:
            msg (str): The message to log.
        """
        self._logger.info(msg)

    def error(self, msg: str) -> None:
        """
        Log an error message.

        Args:
            msg (str): The message to log.
        """
        self._logger.error(msg)

    def debug(self, msg: str) -> None:
        """
        Log a debug message.

        Args:
            msg (str): The message to log.
        """
        self._logger.debug(msg)

    def warning(self, msg: str) -> None:
        """
        Log a warning message.

        Args:
            msg (str): The message to log.
        """
        self._logger.warning(msg)

    def warning_once(self, msg: str, key: str | None = None) -> None:
        """
        Log a warning the first time ``key`` (default: ``msg``) is seen, then at DEBUG.

        For per-step conditions that would otherwise flood the log every step.

        Args:
            msg (str): The message to log.
            key (str, optional): Identity of the condition when ``msg`` varies.
        """
        key = key or msg
        if key in self._once_keys:
            self._logger.debug(msg)
            return

        self._once_keys.add(key)
        self._logger.warning(f"{msg} (further occurrences are logged at DEBUG level)")

    def success(self, msg: str) -> None:
        """
        Log a success message.

        Args:
            msg (str): The message to log.
        """
        self._logger.success(msg)

    def critical(self, msg: str) -> None:
        """
        Log a critical message.

        Args:
            msg (str): The message to log.
        """
        self._logger.critical(msg)
