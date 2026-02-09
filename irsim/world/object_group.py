from irsim.config import env_param
from irsim.lib.behavior.group_behavior import GroupBehavior
from irsim.world.object_base import ObjectBase


class ObjectGroup:
    def __init__(
        self,
        members: list[ObjectBase],
        group_id: int,
    ):
        """
        Define a group of objects with the same role and group id.

        Args:
            members: list[ObjectBase]
            group_id: group id
        Returns:
            None
        """

        self.members = members
        self.group_id = group_id
        self.role = (
            self._delegate_member.role if self._delegate_member is not None else None
        )
        self.kinematics = (
            self._delegate_member.kinematics
            if self._delegate_member is not None
            else None
        )
        self.number = len(members)

        self.group_behavior = GroupBehavior(
            members,
            **self._delegate_member.group_behavior_dict
            if self._delegate_member is not None
            else {},
        )

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
            for member, action in zip(self.members, actions, strict=True)
        ]

    def gen_group_vel(self):
        return self.group_behavior.gen_group_vel()

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

    @property
    def _delegate_member(self) -> ObjectBase:
        if not self.members:
            return None

        return self.members[0]

    @property
    def logger(self):
        return env_param.logger
