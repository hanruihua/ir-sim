import importlib
import logging
from typing import Any, Optional

from irsim.config import world_param
from irsim.lib.behavior.behavior_registry import (
    group_behaviors_class_map,
    group_behaviors_map,
)
from irsim.world.object_base import ObjectBase

logger = logging.getLogger(__name__)


class GroupBehavior:
    """
    Group behavior facade.

    - If a class-based handler is registered, it is instantiated once and reused.
    - Otherwise, a function-based behavior is invoked each step.
    """

    def __init__(self, members: list[ObjectBase], **behavior_dict: Any) -> None:
        """Create a group behavior wrapper.

        Args:
            members: Current group members to control.
            **behavior_dict: Behavior configuration. Must include:
                - name: behavior action (e.g., "orca").
                - optional behavior-specific parameters.

        Notes:
            - If a class-based group behavior is registered for (kinematics, name),
              it will be instantiated once and reused.
            - Otherwise, a function-based behavior is used each step.
        """
        self.members = members
        self.behavior_dict = behavior_dict or {}
        self.kinematics: Optional[str] = members[0].kinematics if members else None
        self.name: Optional[str] = self.behavior_dict.get("name")

        self.load_group_behaviors()
        self._invoke_func = None
        self._init_group_behavior_class()

    def _init_group_behavior_class(self) -> None:
        """Initialize a class-based handler if one is registered for this behavior.

        Looks up a class in `group_behaviors_class_map` using
        (self.kinematics, self.name). If found, instantiates it and stores the
        resulting callable in `self._invoke_func`; otherwise leaves it as None.
        """
        if not self.behavior_dict or self.name is None or self.kinematics is None:
            self._invoke_func = None
            return

        key = (self.kinematics, self.name)
        init_cls = group_behaviors_class_map.get(key)
        if init_cls is None:
            self._invoke_func = None
            return

        try:
            self._invoke_func = init_cls(self.members, **self.behavior_dict)
        except Exception as e:
            logger.error(f"Failed to init group behavior class for {key}: {e!s}")
            self._invoke_func = None

    def update_members(self, members: list[ObjectBase]) -> None:
        """Update the current group members.

        Call this if the composition of the group changes at runtime. The
        class-based handler (if any) is expected to adapt on next call; no
        forced re-instantiation is performed here.
        """
        self.members = members

    def gen_group_vel(self) -> list[Any]:
        """Generate per-member actions for one step.

        Returns:
            list: A list of actions aligned with `members`. Each element is
                  behavior-specific (e.g., 2x1 numpy arrays for velocity).

        Behavior:
            - Uses a class-based handler if one was registered and initialized.
            - Otherwise looks up a function behavior in the registry.
            - If no behavior is configured, returns `[None]` (one sentinel
              element) and logs a warning periodically in auto mode.
        """
        if not self.behavior_dict or self.name is None or self.kinematics is None:
            if world_param.control_mode == "auto" and world_param.count % 20 == 0:
                logger.warning(
                    "Group behavior not defined. Auto control will be static. "
                    "Available behaviors: orca"
                )
            return [None]

        # Prefer class-based handler if initialized
        if callable(self._invoke_func):
            return self._invoke_func(self.members, **self.behavior_dict)

        # Fallback to function-based behavior
        key = (self.kinematics, self.name)
        func = group_behaviors_map.get(key)
        if not func:
            logger.error(
                f"No group behavior method found for category '{self.kinematics}' "
                f"and action '{self.name}'."
            )
            return [None]
        return func(self.members, **self.behavior_dict)

    def load_group_behaviors(self, group_behaviors: str = ".group_behavior_methods"):
        """Dynamically import user/group behavior methods.

        Args:
            group_behaviors: Dotted module path (relative to
                `irsim.lib.behavior`) that contains group behavior registrations.

        Note:
            Importing this module should register additional behaviors or
            behavior classes via the behavior registry decorators.
        """

        try:
            importlib.import_module(group_behaviors, package="irsim.lib.behavior")
        except ImportError as e:
            print(f"Failed to load module '{group_behaviors}': {e}")
