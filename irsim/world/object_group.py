from typing import Any, Optional

import numpy as np

from irsim.config import world_param
from irsim.world.object_base import ObjectBase


class ObjectGroup:
    def __init__(
        self,
        members: list[ObjectBase],
        group_id: int,
        group_behavior: Optional[dict] = None,
    ):
        """
        Define a group of objects with the same role and group id.

        Args:
            members: list[ObjectBase]
            group_id: group id
            group_behavior: group behavior dictionary

        Returns:
            None
        """

        self.members = members
        self.group_id = group_id
        self.role = members[0].role
        self.kinematics = members[0].kinematics
        self.number = len(members)

        self.group_behavior_dict = group_behavior

    def step(self, actions: list[any], sensor_step: bool = True):
        """
        Step the group of objects.

        Args:
            actions: robot actions for each member in the group
            sensor_step: whether to step the sensors of the members in the group

        Returns:
            None
        """

        actions = actions + [None] * (self.number - len(actions))

        [
            member.step(action, sensor_step)
            for member, action in zip(self.members, actions)
        ]

    def __hash__(self) -> int:
        return hash((self.role, self.group_id))

    def __len__(self) -> int:
        """
        Enable len(object_group) to return the number of members.

        Returns:
            int: number of members in the group
        """
        return self.number

    def __str__(self) -> str:
        return f"ObjectGroup(role='{self.role}', group_id={self.group_id}, number={self.number})"

    def __repr__(self) -> str:
        return f"ObjectGroup(role={self.role!r}, group_id={self.group_id}, number={self.number})"

    def __bool__(self) -> bool:
        return self.number > 0

    def __eq__(self, other) -> bool:
        if not isinstance(other, ObjectGroup):
            return NotImplemented
        return (self.role, self.group_id) == (other.role, other.group_id)

    def __iter__(self):
        return iter(self.members)

    def __getitem__(self, idx: int) -> ObjectBase:
        return self.members[idx]

    def __contains__(self, item) -> bool:
        if isinstance(item, ObjectBase):
            return item in self.members
        # allow membership check by id
        try:
            return any(m.id == int(item) for m in self.members)
        except (TypeError, ValueError):
            return False

    def gen_group_behavior_vel(self):
        if self.group_behavior is None or not self.group_behavior:
            if world_param.control_mode == "auto" and world_param.count % 20 == 0:
                self.logger.warning(
                    f"Group behavior not defined for group {self.group_id}. auto control will be static. Available behaviors: orca"
                )

            return np.zeros((2, 1))

        return self.invoke_group_behavior(
            self.kinematics,
            self.behavior_dict["name"],
            self.members,
            **self.group_behavior_dict,
        )

    def invoke_group_behavior(self, kinematics: str, action: str, **kwargs: Any):
        pass
