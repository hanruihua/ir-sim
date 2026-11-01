import importlib
from typing import Any

from irsim.lib.behavior.behavior_registry import (
    group_behaviors_class_map,
    group_behaviors_map,
)
from irsim.util.util import filter_by_role, log_error, log_warning_once
from irsim.world.object_base import ObjectBase


class GroupBehavior:
    """
    Group behavior facade.

    - If a class-based handler is registered, it is instantiated once and reused.
    - Otherwise, a function-based behavior is invoked each step.
    - ``target_roles`` is read here, as :class:`~irsim.lib.behavior.behavior.Behavior`
      does for one object: the handler receives the outside objects it selects,
      the obstacles unless set, as ``external_objects`` (:attr:`target_objects`).
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
        self.kinematics: str | None = members[0].kinematics if members else None
        self.name: str | None = self.behavior_dict.get("name")
        self.target_roles: str = self.behavior_dict.get("target_roles", "obstacle")

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
            log_error(f"Failed to init group behavior class for {key}: {e!s}")
            self._invoke_func = None

    def update_members(self, members: list[ObjectBase]) -> None:
        """Update the current group members.

        Call this if the composition of the group changes at runtime. The
        class-based handler (if any) is expected to adapt on next call; no
        forced re-instantiation is performed here.
        """
        self.members = members

    @property
    def target_objects(self) -> list[ObjectBase]:
        """The objects outside the group that ``target_roles`` selects.

        Handed to the handler as ``external_objects`` every step: what
        :attr:`ObjectBase.external_objects` is for one object, with the
        members left out and the role filter applied.
        """
        if not self.members:
            return []
        member_ids = {id(m) for m in self.members}
        outside = [
            obj for obj in self.members[0].external_objects if id(obj) not in member_ids
        ]
        return filter_by_role(outside, self.target_roles)

    def _no_actions(self) -> list[None]:
        """One ``None`` per member, so the result stays aligned with ``members``."""
        return [None] * len(self.members)

    def gen_group_vel(self) -> list[Any]:
        """Generate per-member actions for one step.

        Returns:
            list: A list of actions aligned with `members` (always
                  ``len(members)`` entries). Each element is behavior-specific
                  (e.g., 2x1 numpy arrays for velocity), or ``None`` when the
                  member gets no group action.

        Behavior:
            - Uses a class-based handler if one was registered and initialized.
            - Otherwise looks up a function behavior in the registry.
            - Either one is called with the members and, as the keyword
              ``external_objects``, the outside objects ``target_roles``
              selects (:attr:`target_objects`); a handler written as
              ``(members, **kwargs)`` keeps working.
            - If no behavior is configured, returns ``None`` for every member
              and logs a warning periodically in auto mode.
        """

        if not self.behavior_dict:
            return self._no_actions()

        if self.name is None or self.kinematics is None:
            # Access params via first member if available
            if self.members:
                wp = self.members[0]._world_param
                if wp.control_mode == "auto":
                    log_warning_once(
                        f"Group behavior not defined for the group of {self.members[0].name}. "
                        "Auto control will be static. Available behaviors: orca"
                    )
            return self._no_actions()

        # Prefer class-based handler if initialized
        if callable(self._invoke_func):
            return self._invoke_func(
                self.members,
                external_objects=self.target_objects,
                **self.behavior_dict,
            )

        # Fallback to function-based behavior
        key = (self.kinematics, self.name)
        func = group_behaviors_map.get(key)
        if not func:
            log_error(
                f"No group behavior method found for category '{self.kinematics}' "
                f"and action '{self.name}'."
            )
            return self._no_actions()
        return func(
            self.members, external_objects=self.target_objects, **self.behavior_dict
        )

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
            log_error(f"Failed to load module '{group_behaviors}': {e}")
