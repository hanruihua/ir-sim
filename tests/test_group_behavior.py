import unittest
from unittest.mock import Mock, patch

from irsim.lib.behavior.group_behavior import GroupBehavior
from irsim.world.object_base import ObjectBase


class TestGroupBehavior(unittest.TestCase):
    def setUp(self):
        self.member1 = Mock(spec=ObjectBase)
        self.member1.kinematics = "diff"
        self.member2 = Mock(spec=ObjectBase)
        self.member2.kinematics = "diff"
        self.members = [self.member1, self.member2]

    def test_init_no_behavior(self):
        gb = GroupBehavior(self.members)
        assert gb.name is None
        assert gb.kinematics == "diff"
        assert gb.behavior_dict == {}
        assert gb._invoke_func is None

    def test_init_empty_members(self):
        gb = GroupBehavior([])
        assert gb.kinematics is None
        assert gb.behavior_dict == {}
        assert gb._invoke_func is None

    @patch("irsim.lib.behavior.group_behavior.group_behaviors_map")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_class_map")
    def test_init_function_behavior(self, mock_class_map, mock_func_map):
        # Setup registry mocks
        mock_class_map.get.return_value = None

        behavior_dict = {"name": "test_func_behavior", "param": 1}
        gb = GroupBehavior(self.members, **behavior_dict)

        assert gb.name == "test_func_behavior"
        assert gb.kinematics == "diff"
        assert gb._invoke_func is None  # Should remain None for function behaviors

    @patch("irsim.lib.behavior.group_behavior.group_behaviors_map")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_class_map")
    def test_init_class_behavior(self, mock_class_map, mock_func_map):
        # Setup registry mocks
        mock_handler_instance = Mock()
        mock_handler_class = Mock(return_value=mock_handler_instance)
        mock_class_map.get.return_value = mock_handler_class

        behavior_dict = {"name": "test_class_behavior", "param": 1}
        gb = GroupBehavior(self.members, **behavior_dict)

        # Verify class lookup and instantiation
        mock_class_map.get.assert_called_with(("diff", "test_class_behavior"))
        mock_handler_class.assert_called_with(self.members, **behavior_dict)

        assert gb._invoke_func == mock_handler_instance

    @patch("irsim.lib.behavior.group_behavior.group_behaviors_map")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_class_map")
    def test_init_class_behavior_failure(self, mock_class_map, mock_func_map):
        # Simulate exception during class instantiation
        mock_handler_class = Mock(side_effect=ValueError("Init failed"))
        mock_class_map.get.return_value = mock_handler_class

        behavior_dict = {"name": "test_class_behavior"}

        # Should catch exception and log error (not tested here), leaving _invoke_func as None
        gb = GroupBehavior(self.members, **behavior_dict)
        assert gb._invoke_func is None

    def test_gen_group_vel_no_config(self):
        gb = GroupBehavior(self.members)
        result = gb.gen_group_vel()
        assert result == [None]

    @patch("irsim.lib.behavior.group_behavior.group_behaviors_map")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_class_map")
    def test_gen_group_vel_function_behavior(self, mock_class_map, mock_func_map):
        mock_class_map.get.return_value = None

        mock_func = Mock(return_value=["action1", "action2"])
        mock_func_map.get.return_value = mock_func

        behavior_dict = {"name": "test_func", "p": 1}
        gb = GroupBehavior(self.members, **behavior_dict)

        result = gb.gen_group_vel()

        assert result == ["action1", "action2"]
        mock_func_map.get.assert_called_with(("diff", "test_func"))
        mock_func.assert_called_with(self.members, **behavior_dict)

    @patch("irsim.lib.behavior.group_behavior.group_behaviors_map")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_class_map")
    def test_gen_group_vel_class_behavior(self, mock_class_map, mock_func_map):
        mock_handler = Mock(return_value=["action1", "action2"])
        mock_class = Mock(return_value=mock_handler)
        mock_class_map.get.return_value = mock_class

        behavior_dict = {"name": "test_class"}
        gb = GroupBehavior(self.members, **behavior_dict)

        result = gb.gen_group_vel()

        assert result == ["action1", "action2"]
        mock_handler.assert_called_with(self.members, **behavior_dict)
        # Verify function map was NOT queried when class handler exists
        mock_func_map.get.assert_not_called()

    @patch("irsim.lib.behavior.group_behavior.group_behaviors_map")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_class_map")
    def test_gen_group_vel_missing_function(self, mock_class_map, mock_func_map):
        mock_class_map.get.return_value = None
        mock_func_map.get.return_value = None

        behavior_dict = {"name": "missing_behavior"}
        gb = GroupBehavior(self.members, **behavior_dict)

        with self.assertLogs("irsim.lib.behavior.group_behavior", level="ERROR") as cm:
            gb.gen_group_vel()

        assert (
            "No group behavior method found for category 'diff' and action 'missing_behavior'."
            in cm.output[0]
        )

    def test_update_members(self):
        gb = GroupBehavior(self.members)
        new_members = [self.member1]
        gb.update_members(new_members)
        assert gb.members == new_members


if __name__ == "__main__":
    unittest.main()
