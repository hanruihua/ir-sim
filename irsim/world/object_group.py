from irsim.world.object_base import ObjectBase


class ObjectGroup:
    def __init__(self, members: list[ObjectBase], group_id: int):
        self.members = members
        self.group_id = group_id
        self.role = members[0].role

    def step(self, actions: list[any], sensor_step: bool = True):
        actions = actions + [None] * (len(self.members) - len(actions))
        [
            member.step(action, sensor_step)
            for member, action in zip(self.members, actions)
        ]
