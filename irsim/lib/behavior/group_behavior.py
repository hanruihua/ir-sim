from typing import Optional

from irsim.world.object_base import ObjectBase


class GroupBehavior:
    def __init__(self, members: list[ObjectBase], behavior: Optional[dict] = None):
        self.members = members
        self.role = members[0].role
        self.number = len(members)

    def gen_vel(self):
        return

    def load_group_behavior(self, behaviors: str = ".behavior_methods"):
        return

    def invoke_group_behavior(self, kinematics: str, action: str, **kwargs):
        return
