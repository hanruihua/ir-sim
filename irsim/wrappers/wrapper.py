"""Base environment wrapper."""

from __future__ import annotations

from typing import Any


class Wrapper:
    """Proxy an environment while allowing selected methods to be adapted.

    Wrappers are composed explicitly instead of being registered. This keeps
    construction parameters visible and allows wrappers to be nested.

    Args:
        env: Environment instance to wrap.
    """

    def __init__(self, env: Any) -> None:
        self.env = env

    def __getattr__(self, name: str) -> Any:
        """Delegate unknown attributes to the wrapped environment."""
        return getattr(self.env, name)

    @property
    def unwrapped(self) -> Any:
        """Return the environment below all IR-SIM wrappers."""
        env = self.env
        while isinstance(env, Wrapper):
            env = env.env
        return env
