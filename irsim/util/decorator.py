"""Decorators shared by IR-SIM environments and utilities.

``bind_env``, ``plot_only`` and ``normalize_actions`` wrap :class:`EnvBase`
methods; ``time_it`` / ``time_it2`` are general timing helpers.
"""

import functools
import time
from typing import Any

import numpy as np

from irsim.util.util import find_object_by_identity, is_list_of_numbers


def time_it(name: str = "Function") -> Any:
    """
    Decorator to measure function execution time.

    Args:
        name (str): Function name for logging (default "Function").

    Returns:
        function: Wrapped function with timing.
    """

    def decorator(func):
        def wrapper(*args, **kwargs):
            wrapper.count += 1
            start = time.time()
            result = func(*args, **kwargs)
            end = time.time()
            wrapper.func_count += 1
            print(f"{name} execute time {(end - start):.6f} seconds")
            return result

        wrapper.count = 0
        wrapper.func_count = 0
        return wrapper

    return decorator


def _as_action_list(action: Any, n_targets: int) -> list:
    """Return ``action`` as a list with one entry per target object.

    A single action (an ndarray, or a flat list/tuple of numbers such as
    ``[1.0, 0.5]``) is repeated for every target; a list/tuple of actions is
    used as is.
    """
    if isinstance(action, tuple):
        action = list(action)
    if isinstance(action, np.ndarray) or (action and is_list_of_numbers(action)):
        return [action] * n_targets
    if isinstance(action, list):
        return action
    raise TypeError(
        f"action must be a list, tuple, or ndarray, got {type(action).__name__}"
    )


def _target_positions(objects: list, action_id: Any, count: int) -> list[int]:
    """Return the positions in ``objects`` that ``action_id`` refers to.

    ``action_id`` is an object id (``obj.id``) or name, or a list of them. A
    single id (``None`` means the first object) selects ``count`` consecutive
    objects starting from it; an unknown id or name raises ``ValueError``.
    """

    def position(key):
        if isinstance(key, str):
            obj = find_object_by_identity(objects, name=key)
        else:
            obj = find_object_by_identity(objects, object_id=int(key))
        if obj is None:
            raise ValueError(f"no object with id or name {key!r}")
        return objects.index(obj)

    if isinstance(action_id, list):
        return [position(i) for i in action_id]
    start = 0 if action_id is None else position(action_id)
    return list(range(start, min(start + count, len(objects))))


def bind_env(method):
    """Decorator making ``self`` the current environment before ``method`` runs.

    Object ids, the random generator, and context-free logging are reached
    through the module-level ``env_param``/``world_param`` proxies, so methods
    that construct objects or draw random numbers bind their environment first.
    """

    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        self._bind_config()
        return method(self, *args, **kwargs)

    return wrapper


def plot_only(method):
    """Decorator making an env plotting helper a no-op when the env has no figure.

    ``EnvBase`` creates no ``EnvPlot`` in headless mode, so
    rendering, drawing, and saving helpers decorated with this simply return
    ``None`` in that case.
    """

    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        if self._env_plot is None:
            return None
        return method(self, *args, **kwargs)

    return wrapper


def normalize_actions(func):
    """
    Decorator to normalize (action, action_id) into an aligned actions list.

    The wrapped method must belong to a class that has a ``objects`` attribute.
    It receives ``actions``, a list aligned with ``self.objects`` (``None`` where
    nothing was given). ``action`` is one action, a list of actions, or a dict
    ``{name: action}``; ``action_id`` is an object id (``obj.id``) or name
    (``None`` for the first object), or a list of them. A single action is
    applied to every id; a list of actions is applied to the given ids, or to
    consecutive objects starting from the given id. Surplus actions (or ids)
    are dropped with a warning; an unknown id or name raises.
    """

    def wrapper(self, action=None, action_id=None, *args, **kwargs):
        objects = getattr(self, "objects", [])
        actions = [None] * len(objects)

        if isinstance(action, dict):  # {name: action}
            action_id, action = list(action), list(action.values())
        if action is not None:
            n_ids = len(action_id) if isinstance(action_id, list) else 1
            action_list = _as_action_list(action, n_ids)
            targets = _target_positions(objects, action_id, len(action_list))
            if len(action_list) != len(targets):
                self.logger.warning_once(
                    f"{len(action_list)} action(s) but {len(targets)} target object(s); "
                    "matched in order, the rest are ignored",
                    key="normalize_actions:surplus",
                )
            for pos, a in zip(targets, action_list, strict=False):
                actions[pos] = a

        return func(self, actions, 0, *args, **kwargs)

    return wrapper


def time_it2(name: str = "Function") -> Any:
    """
    Decorator to measure function execution time with instance attribute check.

    Args:
        name (str): Function name for logging (default "Function").

    Returns:
        function: Wrapped function with timing.
    """

    def decorator(func):
        def wrapper(self, *args, **kwargs):
            wrapper.count += 1
            start = time.time()
            result = func(self, *args, **kwargs)
            end = time.time()
            wrapper.func_count += 1
            if self.time_print:
                print(f"{name} execute time {(end - start):.6f} seconds")
            return result

        wrapper.count = 0
        wrapper.func_count = 0
        return wrapper

    return decorator
