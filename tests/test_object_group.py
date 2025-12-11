import unittest
from unittest.mock import Mock

from irsim.world.object_base import ObjectBase
from irsim.world.object_group import ObjectGroup


class TestObjectGroup(unittest.TestCase):
    def setUp(self):
        # Create mock members
        self.member1 = Mock(spec=ObjectBase)
        self.member1.role = "robot"
        self.member1.kinematics = "diff"
        self.member1.group_behavior_dict = {"name": "test_behavior"}
        self.member1.id = 1

        self.member2 = Mock(spec=ObjectBase)
        self.member2.role = "robot"
        self.member2.kinematics = "diff"
        self.member2.group_behavior_dict = {"name": "test_behavior"}
        self.member2.id = 2

        self.members = [self.member1, self.member2]
        self.group_id = 0

        # Initialize ObjectGroup
        self.object_group = ObjectGroup(self.members, self.group_id)

    def test_step(self):
        """Test step method delegates to members correctly"""
        actions = ["action1", "action2"]
        self.object_group.step(actions)

        self.member1.step.assert_called_with("action1", True)
        self.member2.step.assert_called_with("action2", True)

    def test_step_with_insufficient_actions(self):
        """Test step handles fewer actions than members"""
        actions = ["action1"]
        self.object_group.step(actions)

        self.member1.step.assert_called_with("action1", True)
        self.member2.step.assert_called_with(None, True)

    def test_step_without_sensor_update(self):
        """Test step handles sensor_step flag"""
        actions = ["action1", "action2"]
        self.object_group.step(actions, sensor_step=False)

        self.member1.step.assert_called_with("action1", False)
        self.member2.step.assert_called_with("action2", False)

    def test_len(self):
        """Test __len__ returns correct number of members"""
        assert len(self.object_group) == 2

    def test_getitem(self):
        """Test indexing works correctly"""
        assert self.object_group[0] == self.member1
        assert self.object_group[1] == self.member2

    def test_contains(self):
        """Test membership check"""
        assert 1 in self.object_group
        assert 99 not in self.object_group

        # Test checking by ID if needed (as per implementation details)
        assert 1 in self.object_group
        assert 99 not in self.object_group

    def test_equality(self):
        """Test equality check"""
        other_group = ObjectGroup(self.members, self.group_id)
        assert self.object_group == other_group

        different_group = ObjectGroup(self.members, self.group_id + 1)
        assert self.object_group != different_group

    def test_bool(self):
        """Test boolean evaluation"""
        assert self.object_group

        # Test empty group logic is tricky due to __init__ dependencies
        assert not ObjectGroup([], 0)

    def test_magic_methods(self):
        """Test magic methods"""
        assert (
            str(self.object_group) == "ObjectGroup(role='robot', group_id=0, number=2)"
        )
        assert (
            repr(self.object_group) == "ObjectGroup(role='robot', group_id=0, number=2)"
        )
        assert hash(self.object_group) == hash(
            (self.object_group.role, self.object_group.group_id)
        )
        assert len(self.object_group) == 2
        assert self.object_group[0] == self.member1
        assert self.object_group[1] == self.member2
        assert 1 in self.object_group
        assert 99 not in self.object_group


if __name__ == "__main__":
    unittest.main()
