"""
Tests for behavior functionality.

Covers individual behaviors, group behaviors, and custom behaviors.
"""

import types
from unittest.mock import Mock, patch

import numpy as np
import pytest

from irsim.lib.behavior.behavior import Behavior
from irsim.lib.behavior.group_behavior import GroupBehavior
from irsim.world.object_base import ObjectBase


class TestBehavior:
    """Tests for individual Behavior class."""

    def test_empty_behavior_dict(self, dummy_logger):
        """Test behavior with empty behavior dict returns zeros."""
        b = Behavior(
            object_info=types.SimpleNamespace(id=1, name="test_obj"),
            behavior_dict={},
        )
        vel = b.gen_vel(ego_object=None, external_objects=[])
        assert np.allclose(vel, np.zeros((2, 1)))

    def test_invoke_invalid_behavior(self, dummy_logger):
        """Test invoking invalid behavior raises ValueError."""
        b = Behavior(
            object_info=types.SimpleNamespace(id=1, name="test_obj"),
            behavior_dict={},
        )
        with pytest.raises(ValueError, match="No method found"):
            b.invoke_behavior(
                "diff", "unknown_action", ego_object=None, external_objects=[]
            )


class TestGroupBehavior:
    """Tests for GroupBehavior class."""

    @pytest.fixture
    def mock_members(self):
        """Create mock group members."""
        member1 = Mock(spec=ObjectBase)
        member1.kinematics = "diff"
        member2 = Mock(spec=ObjectBase)
        member2.kinematics = "diff"
        return [member1, member2]

    def test_init_no_behavior(self, mock_members):
        """Test GroupBehavior initialization without behavior."""
        gb = GroupBehavior(mock_members)
        assert gb.name is None
        assert gb.kinematics == "diff"
        assert gb.behavior_dict == {}
        assert gb._invoke_func is None

    def test_init_empty_members(self):
        """Test GroupBehavior with empty members list."""
        gb = GroupBehavior([])
        assert gb.kinematics is None
        assert gb.behavior_dict == {}
        assert gb._invoke_func is None

    @patch("irsim.lib.behavior.group_behavior.group_behaviors_map")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_class_map")
    def test_init_function_behavior(self, mock_class_map, mock_func_map, mock_members):
        """Test GroupBehavior with function-based behavior."""
        mock_class_map.get.return_value = None
        behavior_dict = {"name": "test_func_behavior", "param": 1}
        gb = GroupBehavior(mock_members, **behavior_dict)

        assert gb.name == "test_func_behavior"
        assert gb.kinematics == "diff"
        assert gb._invoke_func is None

    @patch("irsim.lib.behavior.group_behavior.group_behaviors_map")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_class_map")
    def test_init_class_behavior(self, mock_class_map, mock_func_map, mock_members):
        """Test GroupBehavior with class-based behavior."""
        mock_handler_instance = Mock()
        mock_handler_class = Mock(return_value=mock_handler_instance)
        mock_class_map.get.return_value = mock_handler_class

        behavior_dict = {"name": "test_class_behavior", "param": 1}
        gb = GroupBehavior(mock_members, **behavior_dict)

        mock_class_map.get.assert_called_with(("diff", "test_class_behavior"))
        mock_handler_class.assert_called_with(mock_members, **behavior_dict)
        assert gb._invoke_func == mock_handler_instance

    @patch("irsim.lib.behavior.group_behavior.group_behaviors_map")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_class_map")
    def test_init_class_behavior_failure(
        self, mock_class_map, mock_func_map, mock_members
    ):
        """Test GroupBehavior handles class instantiation failure."""
        mock_handler_class = Mock(side_effect=ValueError("Init failed"))
        mock_class_map.get.return_value = mock_handler_class

        behavior_dict = {"name": "test_class_behavior"}
        gb = GroupBehavior(mock_members, **behavior_dict)
        assert gb._invoke_func is None

    def test_gen_group_vel_no_config(self, mock_members):
        """Test gen_group_vel with no behavior configured."""
        gb = GroupBehavior(mock_members)
        result = gb.gen_group_vel()
        assert result == [None]

    @patch("irsim.lib.behavior.group_behavior.group_behaviors_map")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_class_map")
    def test_gen_group_vel_function_behavior(
        self, mock_class_map, mock_func_map, mock_members
    ):
        """Test gen_group_vel with function-based behavior."""
        mock_class_map.get.return_value = None
        mock_func = Mock(return_value=["action1", "action2"])
        mock_func_map.get.return_value = mock_func

        behavior_dict = {"name": "test_func", "p": 1}
        gb = GroupBehavior(mock_members, **behavior_dict)
        result = gb.gen_group_vel()

        assert result == ["action1", "action2"]
        mock_func_map.get.assert_called_with(("diff", "test_func"))
        mock_func.assert_called_with(mock_members, **behavior_dict)

    @patch("irsim.lib.behavior.group_behavior.group_behaviors_map")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_class_map")
    def test_gen_group_vel_class_behavior(
        self, mock_class_map, mock_func_map, mock_members
    ):
        """Test gen_group_vel with class-based behavior."""
        mock_handler = Mock(return_value=["action1", "action2"])
        mock_class = Mock(return_value=mock_handler)
        mock_class_map.get.return_value = mock_class

        behavior_dict = {"name": "test_class"}
        gb = GroupBehavior(mock_members, **behavior_dict)
        result = gb.gen_group_vel()

        assert result == ["action1", "action2"]
        mock_handler.assert_called_with(mock_members, **behavior_dict)
        mock_func_map.get.assert_not_called()

    @patch("irsim.lib.behavior.group_behavior.group_behaviors_map")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_class_map")
    def test_gen_group_vel_missing_function(
        self, mock_class_map, mock_func_map, mock_members, caplog
    ):
        """Test gen_group_vel returns None list for missing function."""
        mock_class_map.get.return_value = None
        mock_func_map.get.return_value = None

        behavior_dict = {"name": "missing_behavior"}
        gb = GroupBehavior(mock_members, **behavior_dict)

        # Returns [None] when behavior not found
        result = gb.gen_group_vel()
        assert result == [None]

    def test_update_members(self, mock_members):
        """Test updating group members."""
        gb = GroupBehavior(mock_members)
        new_members = [mock_members[0]]
        gb.update_members(new_members)
        assert gb.members == new_members


class TestCustomBehavior:
    """Tests for custom behavior loading."""

    @pytest.mark.parametrize("projection", ["2d", "3d"])
    def test_custom_behavior(self, env_factory, projection):
        """Test loading and using custom behavior."""
        env = env_factory("custom_behavior.yaml", projection=projection)
        env.load_behavior("custom_behavior_methods")

        for _ in range(10):
            env.step()
            env.render(0.01)


class TestFOVDetection:
    """Tests for field of view detection."""

    @pytest.mark.parametrize("projection", ["2d", "3d"])
    def test_fov_detection(self, env_factory, projection):
        """Test field of view object detection."""
        env = env_factory("test_fov_world.yaml", projection=projection)
        env.obstacle_list[0].get_fov_detected_objects()

        iterations = 30 if projection == "2d" else 10
        for _ in range(iterations):
            detected = [obs.fov_detect_object(env.robot) for obs in env.obstacle_list]
            env.step()
            env.render(
                0.01,
                fov_color="red",
                fov_alpha=0.2,
                fov_edge_color="red",
                fov_zorder=2,
            )
        assert isinstance(detected, list)
