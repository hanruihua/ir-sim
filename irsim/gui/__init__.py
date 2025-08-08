import contextlib

from .mouse_control import MouseControl

with contextlib.suppress(ImportError):
    from .keyboard_control import KeyboardControl
