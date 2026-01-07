"""
Tests for object-related functionality.

Covers ObjectBase, ObjectGroup, ObjectFactory, and obstacle types.
"""

from unittest.mock import Mock

import numpy as np
import pytest

from irsim.world.object_base import ObjectBase
from irsim.world.object_factory import ObjectFactory
from irsim.world.object_group import ObjectGroup
from irsim.world.obstacles.obstacle_acker import ObstacleAcker


class TestObjectGroup:
    """Tests for ObjectGroup class."""

    @pytest.fixture
    def mock_group(self):
        """Create a mock object group."""
        member1 = Mock(spec=ObjectBase)
        member1.role = "robot"
        member1.kinematics = "diff"
        member1.group_behavior_dict = {"name": "test_behavior"}
        member1.id = 1

        member2 = Mock(spec=ObjectBase)
        member2.role = "robot"
        member2.kinematics = "diff"
        member2.group_behavior_dict = {"name": "test_behavior"}
        member2.id = 2

        members = [member1, member2]
        group = ObjectGroup(members, group_id=0)
        return group, members

    def test_step(self, mock_group):
        """Test step method delegates to members correctly."""
        group, members = mock_group
        actions = ["action1", "action2"]
        group.step(actions)

        members[0].step.assert_called_with("action1", True)
        members[1].step.assert_called_with("action2", True)

    def test_step_insufficient_actions(self, mock_group):
        """Test step handles fewer actions than members."""
        group, members = mock_group
        actions = ["action1"]
        group.step(actions)

        members[0].step.assert_called_with("action1", True)
        members[1].step.assert_called_with(None, True)

    def test_step_without_sensor_update(self, mock_group):
        """Test step handles sensor_step flag."""
        group, members = mock_group
        actions = ["action1", "action2"]
        group.step(actions, sensor_step=False)

        members[0].step.assert_called_with("action1", False)
        members[1].step.assert_called_with("action2", False)

    def test_len(self, mock_group):
        """Test __len__ returns correct number of members."""
        group, _ = mock_group
        assert len(group) == 2

    def test_getitem(self, mock_group):
        """Test indexing works correctly."""
        group, members = mock_group
        assert group[0] == members[0]
        assert group[1] == members[1]

    def test_contains(self, mock_group):
        """Test membership check."""
        group, _ = mock_group
        assert 1 in group
        assert 99 not in group

    def test_equality(self, mock_group):
        """Test equality check."""
        group, members = mock_group
        other_group = ObjectGroup(members, group_id=0)
        assert group == other_group

        different_group = ObjectGroup(members, group_id=1)
        assert group != different_group

    def test_bool(self, mock_group):
        """Test boolean evaluation."""
        group, _ = mock_group
        assert group
        assert not ObjectGroup([], 0)

    def test_str_repr(self, mock_group):
        """Test string representation."""
        group, _ = mock_group
        assert str(group) == "ObjectGroup(role='robot', group_id=0, number=2)"
        assert repr(group) == "ObjectGroup(role='robot', group_id=0, number=2)"

    def test_hash(self, mock_group):
        """Test hash computation."""
        group, _ = mock_group
        assert hash(group) == hash((group.role, group.group_id))


class TestObjectFactory:
    """Tests for ObjectFactory class."""

    @pytest.fixture
    def factory(self):
        """Create an ObjectFactory instance."""
        return ObjectFactory()

    def test_create_from_map_none(self, factory):
        """Test create_from_map with None points."""
        result = factory.create_from_map(None)
        assert result == []

    def test_create_from_map_points(self, factory):
        """Test create_from_map with valid points."""
        points = np.array([[0, 1], [0, 1]])
        result = factory.create_from_map(points, reso=0.1)
        assert len(result) == 1

    def test_generate_state_list_manual(self, factory):
        """Test generate_state_list with manual distribution."""
        state_list, goal_list = factory.generate_state_list(
            number=2,
            distribution={"name": "manual"},
            state=[1, 1, 0],
            goal=[9, 9, 0],
        )
        assert len(state_list) == 2
        assert len(goal_list) == 2

    def test_generate_state_list_random(self, factory):
        """Test generate_state_list with random distribution."""
        state_list, goal_list = factory.generate_state_list(
            number=2,
            distribution={"name": "random"},
        )
        assert len(state_list) == 2
        assert len(goal_list) == 2

    def test_generate_state_list_circle(self, factory):
        """Test generate_state_list with circle distribution."""
        state_list, goal_list = factory.generate_state_list(
            number=4,
            distribution={"name": "circle", "radius": 3, "center": [5, 5, 0]},
        )
        assert len(state_list) == 4
        assert len(goal_list) == 4

    def test_3d_distribution_not_implemented(self, factory):
        """Test 3D distribution raises NotImplementedError."""
        with pytest.raises(
            NotImplementedError, match="3D state generation is not yet implemented"
        ):
            factory.create_object(
                obj_type="robot",
                number=1,
                distribution={"name": "manual", "3d": True},
            )

    def test_uniform_distribution_not_implemented(self, factory):
        """Test uniform distribution raises NotImplementedError."""
        with pytest.raises(
            NotImplementedError, match="'uniform' distribution is not yet implemented"
        ):
            factory.generate_state_list(
                number=1,
                distribution={"name": "uniform"},
            )

    def test_unknown_distribution_raises(self, factory):
        """Test unknown distribution raises ValueError."""
        with pytest.raises(ValueError, match="Unknown distribution name"):
            factory.generate_state_list(
                number=1,
                distribution={"name": "unknown_distribution"},
            )


class TestObstacleAcker:
    """Tests for ObstacleAcker class."""

    def test_instantiation(self):
        """Test ObstacleAcker instantiation."""
        obstacle = ObstacleAcker()
        assert obstacle is not None
