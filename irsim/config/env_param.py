"""
Objects: A list of all objects in the environment
Logger: A logger object to log messages
Platform: The operating system platform
"""

import itertools
import platform
import sys
from collections.abc import Iterator
from dataclasses import dataclass, field, fields
from types import ModuleType
from typing import Any

import numpy as np

from irsim.world.object_base import ObjectBase


@dataclass
class EnvParam:
    """Mutable per-environment runtime state.

    Attributes:
        objects: Objects currently managed by the environment.
        logger: Environment logger instance.
        GeometryTree: Spatial index used for geometry queries, when available.
        platform_name: Name of the current operating system platform.
        id_iter: Counter handing out object ids; each environment numbers its
            objects from 0.
        env_id: Process-wide number of the owning environment (``None`` for the
            default instance bound before any environment exists).
        rng: Generator this environment draws from when it was created with a
            seed or reseeded; ``None`` means the shared default of
            ``irsim.util.random``.
    """

    objects: list[ObjectBase] = field(default_factory=list)
    logger: Any | None = None
    GeometryTree: Any | None = None
    platform_name: str = field(default_factory=platform.system)
    id_iter: Iterator[int] = field(default_factory=itertools.count)
    env_id: int | None = None
    rng: np.random.Generator | None = None


# Multi-env storage (default index 0)
_instances: list[EnvParam] = [EnvParam()]
_current = _instances[0]

_PARAM_FIELDS = frozenset(f.name for f in fields(EnvParam))


def bind(instance: EnvParam) -> None:
    """Bind instance to default index 0 and update current alias."""
    global _current
    if _instances:
        _instances[0] = instance
    else:
        _instances.append(instance)
    _current = instance


class _ParamModule(ModuleType):
    """Route param-field access on the module to the bound instance.

    PEP 562 lets a module define ``__getattr__`` only; plain assignment
    (``env_param.logger = x``) would create a real module attribute that
    permanently shadows the proxy. Swapping the module class makes
    attribute reads, writes, and index access all resolve against the
    currently bound :class:`EnvParam` instance.
    """

    def __getattr__(self, name: str):
        return getattr(_current, name)

    def __setattr__(self, name: str, value) -> None:
        if name in _PARAM_FIELDS:
            setattr(_current, name, value)
        else:
            super().__setattr__(name, value)

    def __getitem__(self, index: int) -> EnvParam:
        return _instances[index]

    def __setitem__(self, index: int, instance: EnvParam) -> None:
        """Assign an EnvParam at a specific index. Extends list if needed."""
        global _current
        if index < 0:
            raise IndexError("env_param index must be non-negative")
        if index >= len(_instances):
            _instances.extend(EnvParam() for _ in range(index - len(_instances) + 1))
        _instances[index] = instance
        if index == 0:
            _current = instance


sys.modules[__name__].__class__ = _ParamModule
