"""
World parameters.

Attributes:
    time: time elapse of the simulation
    control_mode: 'auto' (robot controlled automatically) or 'keyboard' (robot controlled by keyboard)
    collision_mode: 'stop' (default, all objects stop on collision), 'unobstructed' (no collision check),
        or 'unobstructed_obstacles' (only obstacles pass through each other)
    step_time: time of the simulation step, default is 0.1
    count: count of the simulation, time = count * step_time
"""

import sys
from dataclasses import dataclass, fields
from types import ModuleType


@dataclass
class WorldParam:
    """Mutable simulation-clock and control-mode parameters.

    Attributes:
        time: Elapsed simulation time.
        control_mode: ``auto`` for automatic control or ``keyboard`` for manual control.
        collision_mode: Collision handling mode.
        step_time: Simulation time step.
        count: Number of elapsed simulation steps.
    """

    time: float = 0.0
    control_mode: str = "auto"
    collision_mode: str = "stop"
    step_time: float = 0.1
    count: int = 0


# Multi-env storage (default index 0)
_instances: list[WorldParam] = [WorldParam()]
_current = _instances[0]

_PARAM_FIELDS = frozenset(f.name for f in fields(WorldParam))


def bind(instance: WorldParam) -> None:
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
    (``world_param.count = x``) would create a real module attribute that
    permanently shadows the proxy. Swapping the module class makes
    attribute reads, writes, and index access all resolve against the
    currently bound :class:`WorldParam` instance.
    """

    def __getattr__(self, name: str):
        return getattr(_current, name)

    def __setattr__(self, name: str, value) -> None:
        if name in _PARAM_FIELDS:
            setattr(_current, name, value)
        else:
            super().__setattr__(name, value)

    def __getitem__(self, index: int) -> WorldParam:
        return _instances[index]

    def __setitem__(self, index: int, instance: WorldParam) -> None:
        """Assign a WorldParam at a specific index. Extends list if needed."""
        global _current
        if index < 0:
            raise IndexError("world_param index must be non-negative")
        if index >= len(_instances):
            _instances.extend(WorldParam() for _ in range(index - len(_instances) + 1))
        _instances[index] = instance
        if index == 0:
            _current = instance


sys.modules[__name__].__class__ = _ParamModule
