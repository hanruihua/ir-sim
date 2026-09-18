"""
Default colors of everything IR-SIM draws.

The values are Okabe-Ito based (with a second green from Paul Tol's muted
scheme), so they stay distinguishable under common forms of colour-vision
deficiency and keep robots, sensors and obstacles apart in grayscale print.
Any color given in YAML or code overrides them. They are read when an object
or a plot is created, so changing them before ``irsim.make()`` restyles every
following scene::

    from irsim.config import palette_param

    palette_param.robot = "#0072B2"
    palette_param.cycle = ["#0072B2", "#D55E00", "#009E73"]

Attributes:
    robot: Robots with ``diff``, ``omni``, ``omni_angular`` or custom kinematics.
    robot_acker: Car-like (``acker``) robots.
    obstacle: Obstacles, grid maps and any object without a role color.
    arrow: Heading arrow drawn on top of a body.
    fov: Field-of-view fill.
    fov_edge: Field-of-view outline.
    lidar: Lidar beams.
    laser_highlight: Beams singled out with ``set_laser_color``.
    fmcw_zero_velocity: FMCW beams with no radial velocity.
    fmcw_positive_velocity: FMCW beams moving away.
    fmcw_negative_velocity: FMCW beams moving closer.
    marker: Points drawn with ``env.draw_points``.
    path: Lines drawn with ``env.draw_trajectory`` and ``env.draw_box``.
    quiver: Arrows drawn with ``env.draw_quiver``.
    cycle: Colors handed out in turn to the objects of a group configured with
        ``color: 'cycle'``; it starts with the robot color so a single robot
        looks the same either way.
"""

import sys
from dataclasses import dataclass, field, fields
from types import ModuleType

DEFAULT_CYCLE = (
    "#009E73",  # bluish green (the robot color)
    "#0072B2",  # blue
    "#E69F00",  # orange
    "#CC79A7",  # reddish purple
    "#56B4E9",  # sky blue
    "#D55E00",  # vermillion
    "#F0E442",  # yellow
    "#117733",  # dark green
)


@dataclass
class PaletteParam:
    """Default colors, one attribute per kind of drawing (see the module docstring)."""

    robot: str = "#009E73"
    robot_acker: str = "#117733"
    obstacle: str = "k"
    arrow: str = "#F0E442"
    fov: str = "#56B4E9"
    fov_edge: str = "#0072B2"
    lidar: str = "#CC3311"
    laser_highlight: str = "#56B4E9"
    fmcw_zero_velocity: str = "#56B4E9"
    fmcw_positive_velocity: str = "#CC3311"
    fmcw_negative_velocity: str = "#0072B2"
    marker: str = "#CC79A7"
    path: str = "#0072B2"
    quiver: str = "k"
    cycle: list[str] = field(default_factory=lambda: list(DEFAULT_CYCLE))

    def cycle_color(self, index: int) -> str:
        """Color of the ``index``-th object of a group drawn with ``color: 'cycle'``."""
        return self.cycle[index % len(self.cycle)]


# Multi-instance storage (default index 0), like the other config modules.
_instances: list[PaletteParam] = [PaletteParam()]
_current = _instances[0]

_PARAM_FIELDS = frozenset(f.name for f in fields(PaletteParam))


def bind(instance: PaletteParam) -> None:
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
    (``palette_param.robot = x``) would create a real module attribute that
    permanently shadows the proxy. Swapping the module class makes
    attribute reads, writes, and index access all resolve against the
    currently bound :class:`PaletteParam` instance.
    """

    def __getattr__(self, name: str):
        return getattr(_current, name)

    def __setattr__(self, name: str, value) -> None:
        if name in _PARAM_FIELDS:
            setattr(_current, name, value)
        else:
            super().__setattr__(name, value)

    def __getitem__(self, index: int) -> PaletteParam:
        return _instances[index]

    def __setitem__(self, index: int, instance: PaletteParam) -> None:
        """Assign a PaletteParam at a specific index. Extends list if needed."""
        global _current
        if index < 0:
            raise IndexError("palette_param index must be non-negative")
        if index >= len(_instances):
            _instances.extend(
                PaletteParam() for _ in range(index - len(_instances) + 1)
            )
        _instances[index] = instance
        if index == 0:
            _current = instance


sys.modules[__name__].__class__ = _ParamModule
