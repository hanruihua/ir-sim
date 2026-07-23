"""
Tests for object-related functionality.

Covers ObjectBase, ObjectGroup, ObjectFactory, and obstacle types.
"""

from collections import deque
from unittest.mock import Mock

import matplotlib.pyplot as plt
import numpy as np
import pytest
import shapely

from irsim.lib.handler.geometry_handler import GeometryFactory
from irsim.world.object_base import ObjectBase
from irsim.world.object_factory import ObjectFactory
from irsim.world.object_group import ObjectGroup
from irsim.world.obstacles.obstacle_acker import ObstacleAcker
from irsim.world.obstacles.obstacle_diff import ObstacleDiff
from irsim.world.obstacles.obstacle_omni import ObstacleOmni
from irsim.world.robots.robot_acker import RobotAcker
from irsim.world.robots.robot_diff import RobotDiff
from irsim.world.robots.robot_omni import RobotOmni


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

    def test_iter(self, mock_group):
        """Iterating a group yields its members in order."""
        group, members = mock_group
        assert list(group) == members

    def test_eq_not_implemented(self, mock_group):
        """Comparing with a non-ObjectGroup returns NotImplemented."""
        group, _ = mock_group
        assert group.__eq__("not a group") is NotImplemented

    def test_contains_object_base(self, mock_group):
        """Membership check accepts ObjectBase instances as well as ids."""
        group, members = mock_group
        assert members[0] in group

        other_member = Mock(spec=ObjectBase)
        other_member.id = 99
        assert other_member not in group

    def test_contains_invalid_type(self, mock_group):
        """Membership check is False for unsupported types."""
        group, _ = mock_group
        assert "not_an_id" not in group
        assert None not in group

    def test_logger_property(self, mock_group, dummy_logger):
        """Group exposes the environment logger."""
        group, _ = mock_group
        assert group.logger is not None

    def test_delegate_member_empty(self):
        """An empty group has no delegate member, role, or kinematics."""
        group = ObjectGroup([], 0)
        assert group._delegate_member is None
        assert group.role is None
        assert group.kinematics is None


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

    def test_create_from_map_empty(self, factory):
        """Test create_from_map with empty array."""
        result = factory.create_from_map(np.array([]))
        assert result == []

    def test_create_from_map_points(self, factory):
        """Test create_from_map with valid points."""
        points = np.array([[0, 1], [0, 1]])
        result = factory.create_from_map(points, reso=np.array([[0.1], [0.1]]))
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
        """Test ObstacleAcker instantiation emits deprecation warning."""
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        with pytest.warns(DeprecationWarning, match="ObstacleAcker is deprecated"):
            obstacle = ObstacleAcker()
        assert obstacle is not None
        env.end()


# ---------------------------------------------------------------------------
# Coverage-targeted tests for ObjectFactory
# ---------------------------------------------------------------------------


class TestObjectFactoryCreateFromParse:
    """Tests for ObjectFactory.create_from_parse with dict input (line 56)."""

    def test_create_from_parse_dict(self):
        """create_from_parse with dict input (line 56)."""
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        factory = ObjectFactory()
        parse = {
            "number": 1,
            "shape": {"name": "circle", "radius": 0.5},
        }
        result = factory.create_from_parse("obstacle", parse)
        assert isinstance(result, list)
        env.end()


class TestObjectFactoryKinematics:
    """Tests for create_robot and create_obstacle with various kinematics."""

    def test_create_robot_static(self):
        """create_robot with static kinematics (line 178-179)."""
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        factory = ObjectFactory()
        robot = factory.create_robot(
            kinematics={"name": "static"},
            shape={"name": "circle", "radius": 0.5},
        )
        assert robot is not None
        env.end()

    def test_create_robot_none_kinematics(self):
        """create_robot with None kinematics (lines 168-169, 178-179)."""
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        factory = ObjectFactory()
        robot = factory.create_robot(
            kinematics=None,
            shape={"name": "circle", "radius": 0.5},
        )
        assert robot is not None
        env.end()

    def test_create_obstacle_acker(self):
        """create_obstacle with acker kinematics (line 203-204)."""
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        factory = ObjectFactory()
        obs = factory.create_obstacle(kinematics={"name": "acker"})
        assert obs is not None
        env.end()

    def test_create_obstacle_omni(self):
        """create_obstacle with omni kinematics (line 205-206)."""
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        factory = ObjectFactory()
        obs = factory.create_obstacle(kinematics={"name": "omni"})
        assert obs is not None
        env.end()

    def test_create_robot_unsupported_kinematics(self):
        """create_robot with unsupported kinematics raises NotImplementedError (line 182)."""
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        factory = ObjectFactory()
        with pytest.raises(NotImplementedError, match="not implemented"):
            factory.create_robot(kinematics={"name": "nonexistent_kin"})
        env.end()

    def test_create_obstacle_unsupported_kinematics(self):
        """create_obstacle with unsupported kinematics raises NotImplementedError (line 209)."""
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        factory = ObjectFactory()
        with pytest.raises(NotImplementedError, match="not implemented"):
            factory.create_obstacle(kinematics={"name": "nonexistent_kin"})
        env.end()


# ---------------------------------------------------------------------------
# Deprecated subclass deprecation-warning tests
# ---------------------------------------------------------------------------


class TestDeprecatedSubclasses:
    """Ensure deprecated robot/obstacle subclasses emit DeprecationWarning."""

    def test_robot_diff_deprecation(self):
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        with pytest.warns(DeprecationWarning, match="RobotDiff is deprecated"):
            robot = RobotDiff(
                kinematics={"name": "diff"}, shape={"name": "circle", "radius": 0.2}
            )
        assert robot is not None

        # The shim's _init_plot keeps the legacy arrow/goal defaults.
        fig, ax = plt.subplots()
        robot._init_plot(ax)
        assert robot.arrow_patch is not None
        plt.close(fig)
        env.end()

    def test_robot_omni_deprecation(self):
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        with pytest.warns(DeprecationWarning, match="RobotOmni is deprecated"):
            robot = RobotOmni(
                kinematics={"name": "omni"}, shape={"name": "circle", "radius": 0.2}
            )
        assert robot is not None

        # The shim's _init_plot keeps the legacy goal default; omni draws no arrow.
        fig, ax = plt.subplots()
        robot._init_plot(ax)
        assert getattr(robot, "arrow_patch", None) is None
        plt.close(fig)
        env.end()

    def test_robot_acker_deprecation(self):
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        with pytest.warns(DeprecationWarning, match="RobotAcker is deprecated"):
            robot = RobotAcker(kinematics={"name": "acker"})
        assert robot is not None
        env.end()

    def test_obstacle_diff_deprecation(self):
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        with pytest.warns(DeprecationWarning, match="ObstacleDiff is deprecated"):
            obs = ObstacleDiff(kinematics={"name": "diff"})
        assert obs is not None
        env.end()

    def test_obstacle_omni_deprecation(self):
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        with pytest.warns(DeprecationWarning, match="ObstacleOmni is deprecated"):
            obs = ObstacleOmni(kinematics={"name": "omni"})
        assert obs is not None
        env.end()


# ---------------------------------------------------------------------------
# ObjectBase fallback property tests (kf is None)
# ---------------------------------------------------------------------------


class TestObjectBaseNoKinematics:
    """Cover fallback paths in ObjectBase properties when kf is None."""

    def _make_no_kf_object(self):
        """Create an ObjectBase with kf=None by passing kinematics=None."""
        obj = ObjectBase(role="obstacle", state=[1, 2, 0.5])
        assert obj.kf is None
        return obj

    def test_velocity_xy_no_kf(self):
        """velocity_xy returns zeros when kf is None."""
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        obj = self._make_no_kf_object()
        result = obj.velocity_xy
        np.testing.assert_array_equal(result, np.zeros((2, 1)))
        env.end()

    def test_max_speed_no_kf(self):
        """max_speed returns 0 when kf is None."""
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        obj = self._make_no_kf_object()
        assert obj.max_speed == 0
        env.end()

    def test_heading_no_kf(self):
        """heading falls back to state[2,0] when kf is None."""
        import irsim

        env = irsim.make("test_collision_world.yaml", save_ani=False, display=False)
        obj = self._make_no_kf_object()
        assert obj.heading == pytest.approx(0.5)
        env.end()


# ---------------------------------------------------------------------------
# Robot API (ObjectBase through a live environment)
# ---------------------------------------------------------------------------


def _goal_covered(goal, obstacle_list, goal_check_radius):
    """True if a circle of ``goal_check_radius`` at ``goal`` hits any obstacle."""
    shape = {"name": "circle", "radius": goal_check_radius}
    gf = GeometryFactory.create_geometry(**shape)
    geometry = gf.step(np.c_[goal])
    return any(shapely.intersects(geometry, obj._geometry) for obj in obstacle_list)


class TestRobotProperties:
    """Tests for robot property accessors."""

    def test_robot_properties(self, env_factory):
        """Test basic robot properties."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        robot = env.robot

        assert robot.abbr is not None
        assert robot.name is not None
        assert robot.velocity is not None
        assert robot.heading is not None
        assert robot.orientation is not None

    def test_robot_arrive(self, env_factory):
        """Test robot arrive property."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        arrive = env.robot.arrive
        assert isinstance(arrive, bool)

    def test_robot_collision(self, env_factory):
        """Test robot collision property."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        collision = env.robot.collision
        assert isinstance(collision, bool)

    def test_robot_max_speed(self, env_factory):
        """Test robot max_speed property."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        max_speed = env.robot.max_speed
        assert max_speed > 0
        # Test second robot too
        if len(env.robot_list) > 1:
            assert env.robot_list[1].max_speed > 0

    def test_robot_vertices(self, env_factory):
        """Test robot vertices properties."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        robot = env.robot
        assert robot.goal_vertices is not None
        assert robot.original_vertices is not None


class TestRobotState:
    """Tests for robot state management."""

    def test_set_state_list(self, env_factory):
        """Test setting robot state from list."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env.robot.set_state([1, 1, 0])
        state = env.robot.state
        assert state[0, 0] == pytest.approx(1.0)
        assert state[1, 0] == pytest.approx(1.0)

    def test_set_state_array(self, env_factory):
        """Test setting robot state from numpy array."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env.robot.set_state(np.array([2, 2, 0]).reshape(3, 1))
        state = env.robot.state
        assert state[0, 0] == pytest.approx(2.0)
        assert state[1, 0] == pytest.approx(2.0)


class TestRobotVelocity:
    """Tests for robot velocity management."""

    def test_set_velocity_list(self, env_factory):
        """Test setting robot velocity from list."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env.robot.set_velocity([1, 1])

    def test_set_velocity_list_init(self, env_factory):
        """Test setting robot velocity with init=True."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env.robot.set_velocity([1, 1], init=True)

    def test_set_velocity_array(self, env_factory):
        """Test setting robot velocity from numpy array."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env.robot.set_velocity(np.array([1, 1]).reshape(2, 1), init=True)

    def test_get_desired_omni_vel(self, env_factory):
        """Test getting desired omnidirectional velocity."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        vel = env.robot.get_desired_omni_vel()
        assert vel is not None

    def test_get_desired_omni_vel_normalized(self, env_factory):
        """Test getting normalized desired omnidirectional velocity."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        vel = env.robot.get_desired_omni_vel(normalized=True)
        assert vel is not None

    def test_get_desired_omni_vel_no_goal(self, env_factory):
        """Test getting desired velocity with no goal set."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env.robot.set_goal(None)
        vel = env.robot.get_desired_omni_vel()
        assert vel is not None

    def test_get_desired_omni_vel_threshold(self, env_factory):
        """Test getting desired velocity with goal threshold."""
        env = env_factory("test_multi_objects_world.yaml")
        env.robot.get_desired_omni_vel(goal_threshold=1000)


class TestRobotGoal:
    """Tests for robot goal management."""

    def test_set_goal_list(self, env_factory):
        """Test setting robot goal from list."""
        env = env_factory("test_all_objects.yaml")
        env.robot.set_goal([9, 9, 0])

    def test_set_goal_multiple(self, env_factory):
        """Test setting multiple robot goals."""
        env = env_factory("test_collision_world.yaml")
        env.robot.set_goal([[5, 10, 0], [5, 9, 0], [5, 8, 0]], init=True)

    def test_set_goal_none(self, env_factory):
        """Test setting robot goal to None."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env.robot.set_goal(None)

    def test_set_random_goal(self, env_factory):
        """Sampled random goals avoid obstacles within the check radius."""
        env = env_factory("test_collision_world.yaml")
        goal_check_radius = 0.4

        # Sanity-check the coverage helper against known positions first.
        env.robot.set_goal([5, 10, 0], init=True)
        assert not _goal_covered(
            env.robot._goal[0], env.obstacle_list, goal_check_radius
        )
        env.robot.set_goal([5, 5, 0], init=True)
        assert _goal_covered(env.robot._goal[0], env.obstacle_list, goal_check_radius)

        for _ in range(20):
            env.robot.set_random_goal(
                env.obstacle_list, goal_check_radius=goal_check_radius
            )
            goal = env.robot._goal[0]
            assert len(goal) == 3
            assert not _goal_covered(goal, env.obstacle_list, goal_check_radius)

    def test_set_random_goal_with_limits(self, env_factory):
        """Random goals with range limits stay in range and avoid obstacles."""
        env = env_factory("test_collision_world.yaml")
        env.robot.set_goal([[5, 10, 0], [5, 9, 0], [5, 8, 0]], init=True)

        for _ in range(20):
            env.robot.set_random_goal(
                env.obstacle_list,
                goal_check_radius=0.4,
                range_limits=[[3, 3, -np.pi], [7, 7, np.pi]],
            )
            goals = env.robot._goal
            for goal in goals:
                assert not _goal_covered(goal, env.obstacle_list, 0.4)
            assert all(3 < point[0] < 7 for point in goals)
            assert all(3 < point[1] < 7 for point in goals)


class TestRobotLidar:
    """Tests for robot lidar functionality."""

    def test_get_lidar_points(self, env_factory):
        """Test getting lidar points."""
        env = env_factory("test_all_objects.yaml")
        points = env.robot.get_lidar_points()
        # Points may be None if robot has no lidar
        if points is not None:
            env.draw_points(points)

    def test_get_lidar_scan(self, env_factory):
        """Test getting lidar scan."""
        env = env_factory("test_all_objects.yaml")
        scan = env.robot.get_lidar_scan()
        assert scan is not None

    def test_get_lidar_offset(self, env_factory):
        """Test getting lidar offset."""
        env = env_factory("test_all_objects.yaml")
        offset = env.robot.get_lidar_offset()
        assert offset is not None

    def test_set_laser_color(self, env_factory):
        """Test setting laser color for specific beams."""
        env = env_factory("test_grid_map.yaml")
        env.robot.set_laser_color(
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9], laser_color="blue", alpha=0.2
        )


class TestRobotGh:
    """Tests for robot G/h matrix functionality."""

    def test_get_init_gh(self, env_factory):
        """Test getting initial G/h matrices."""
        env = env_factory("test_all_objects.yaml")
        gh = env.robot.get_init_Gh()
        assert gh is not None

    def test_get_gh(self, env_factory):
        """Test getting G/h matrices."""
        env = env_factory("test_all_objects.yaml")
        gh = env.robot.get_Gh()
        assert gh is not None


class TestRobotInfo:
    """Tests for robot info management."""

    def test_add_property(self, env_factory):
        """Test adding property to robot info."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env.robot.info.add_property("test", 1)
        assert env.robot.info.test == 1

    def test_get_obstacle_info(self, env_factory):
        """Test getting obstacle info from robot's perspective."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        obs_info = env.robot.get_obstacle_info()
        obs_info.add_property("test", 2)
        assert obs_info.test == 2


class TestRobotComparison:
    """Tests for robot comparison operations."""

    def test_robot_equality(self, env_factory):
        """Test robot equality comparison."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        assert env.robot_list[0] == env.robot_list[0]
        if len(env.robot_list) > 1:
            assert env.robot_list[0] != env.robot_list[1]
        assert env.robot_list[0] != env.robot_list

    def test_robot_hash(self, env_factory):
        """Test robot hash."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        h = hash(env.robot)
        assert isinstance(h, int)

    def test_robot_str(self, env_factory):
        """Test robot string representation."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        s = str(env.robot)
        assert isinstance(s, str)
        assert len(s) > 0


class TestRobotRemove:
    """Tests for robot removal."""

    def test_remove_robot(self, env_factory):
        """Test removing robot from environment."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env.robot.remove()
        # Robot is marked for removal but list update happens elsewhere


class TestRobotHeading:
    """Tests for robot heading with different kinematics."""

    def test_heading_diff(self, env_factory):
        """Test heading for differential drive robot."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        heading = env.robot.heading
        assert heading is not None

    def test_heading_second_robot(self, env_factory):
        """Test heading for second robot."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        if len(env.robot_list) > 1:
            heading = env.robot_list[1].heading
            assert heading is not None


# ---------------------------------------------------------------------------
# Coverage-targeted tests for object_base.py
# ---------------------------------------------------------------------------


class TestSetGoalEdgeCases:
    """Tests for set_goal edge cases (lines 912-964)."""

    def test_set_goal_deque(self, env_factory):
        """set_goal with deque input (lines 928-934)."""
        env = env_factory("test_collision_world.yaml")
        goal_deque = deque([[5, 5, 0], [8, 8, 0]])
        env.robot.set_goal(goal_deque)
        assert env.robot._goal is goal_deque

    def test_set_goal_deque_with_init(self, env_factory):
        """set_goal with deque and init=True (lines 931-932)."""
        env = env_factory("test_collision_world.yaml")
        goal_deque = deque([[5, 5, 0], [8, 8, 0]])
        env.robot.set_goal(goal_deque, init=True)
        assert env.robot._init_goal is not None

    def test_set_goal_none_with_init(self, env_factory):
        """set_goal(None) with init=True (lines 912-918)."""
        env = env_factory("test_collision_world.yaml")
        env.robot.set_goal(None, init=True)
        assert env.robot._goal is None
        assert env.robot._init_goal is None

    def test_set_goal_2d_list_with_init(self, env_factory):
        """set_goal with 2D list and init=True (lines 920-926)."""
        env = env_factory("test_collision_world.yaml")
        env.robot.set_goal([[5, 5, 0], [8, 8, 0]], init=True)
        assert isinstance(env.robot._goal, deque)
        assert env.robot._init_goal is not None

    def test_set_goal_numpy_array(self, env_factory):
        """set_goal with numpy array (lines 942-946)."""
        env = env_factory("test_collision_world.yaml")
        goal = np.array([[5], [5], [0]])
        env.robot.set_goal(goal)
        assert env.robot._goal is not None

    def test_set_goal_single_list_with_init(self, env_factory):
        """set_goal with single list and init=True (lines 936-953)."""
        env = env_factory("test_collision_world.yaml")
        env.robot.set_goal([5, 5, 0], init=True)
        assert env.robot._init_goal is not None


class TestSetLaserColorNoLidar:
    """Test set_laser_color on object without lidar (line 981)."""

    def test_set_laser_color_no_lidar(self, env_factory):
        """set_laser_color warns when no lidar (line 981)."""
        env = env_factory("test_collision_world.yaml")
        robot = env.robot
        if robot.lidar is None:
            robot.set_laser_color([0, 1, 2])  # Should just warn


class TestGenBehaviorVelNoBehavior:
    """Test gen_behavior_vel with no behavior (lines 601-606)."""

    def test_gen_behavior_vel_no_behavior(self, env_factory):
        """gen_behavior_vel without behavior returns zeros (lines 600-606)."""
        env = env_factory("test_collision_world.yaml")
        robot = env.robot
        # Temporarily remove behavior config
        original_beh = robot.obj_behavior.behavior_dict
        robot.obj_behavior.behavior_dict = {}
        robot.group_behavior_dict = {}
        vel = robot.gen_behavior_vel(None)
        assert np.allclose(vel, 0)
        robot.obj_behavior.behavior_dict = original_beh


class TestObjectBaseProperties:
    """Tests for centroid and init_state properties (lines 2177, 2210)."""

    def test_centroid(self, env_factory):
        """centroid property (line 2177)."""
        env = env_factory("test_collision_world.yaml")
        centroid = env.robot.centroid
        assert centroid is not None
        assert centroid.shape[0] == 2

    def test_init_state(self, env_factory):
        """init_state property (line 2210)."""
        env = env_factory("test_collision_world.yaml")
        init_state = env.robot.init_state
        assert init_state is not None


class TestMaxSpeedAndHeadingUnsupported:
    """Test max_speed and heading for unsupported kinematics (lines 2534-2537, 2624)."""

    def test_max_speed_static(self, env_factory):
        """max_speed on static object returns 0 (lines 2534-2537)."""
        env = env_factory("test_all_objects.yaml")
        for obs in env.obstacle_list:
            if obs.kinematics == "static":
                speed = obs.max_speed
                assert speed == 0
                break


class TestSetStateEdgeCases:
    """Tests for set_state with None state (line 791)."""

    def test_set_state_none(self, env_factory):
        """set_state(None) uses default [0,0,0] (line 791)."""
        env = env_factory("test_collision_world.yaml")
        robot = env.robot
        robot.set_state(None)
        assert robot.state is not None


class TestAppendGoal:
    """Tests for append_goal (lines 960-964)."""

    def test_append_goal_list(self, env_factory):
        """append_goal with list input (line 961)."""
        env = env_factory("test_collision_world.yaml")
        robot = env.robot
        robot.set_goal([5, 5, 0])
        robot.append_goal([8, 8, 0])
        assert len(robot._goal) == 2

    def test_append_goal_numpy(self, env_factory):
        """append_goal with numpy array (lines 963-964)."""
        env = env_factory("test_collision_world.yaml")
        robot = env.robot
        robot.set_goal([5, 5, 0])
        robot.append_goal(np.array([[8], [8], [0]]))
        assert len(robot._goal) == 2


class TestGenBehaviorVelNoBehConfig:
    """Test gen_behavior_vel when beh_config is None (lines 600-606)."""

    def test_gen_behavior_vel_no_beh_config(self, env_factory):
        """gen_behavior_vel returns zeros when beh_config is None (lines 600-606)."""
        env = env_factory("test_collision_world.yaml")
        robot = env.robot
        # Make beh_config return None: set both to None
        original_beh = robot.obj_behavior.behavior_dict
        original_group = robot.group_behavior_dict
        robot.obj_behavior.behavior_dict = None
        robot.group_behavior_dict = None
        vel = robot.gen_behavior_vel(None)
        assert np.allclose(vel, 0)
        robot.obj_behavior.behavior_dict = original_beh
        robot.group_behavior_dict = original_group


class TestObjectBaseUnknownKwargs:
    """ObjectBase kwargs validation."""

    def test_object_base_unknown_kwarg_warns(self, dummy_logger):
        """ObjectBase.__init__ warns about unknown kwargs."""
        warnings_collected = []
        dummy_logger.warning = lambda msg, *a, **kw: warnings_collected.append(msg)

        ObjectBase(shape={"name": "circle", "radius": 0.2}, colr="red")  # typo

        assert any("colr" in w for w in warnings_collected)


class TestCollisionModes:
    """check_status behavior across collision modes."""

    def test_collision_mode_reactive(self, env_factory):
        """Reactive mode currently passes through without stopping objects."""
        env = env_factory("test_collision_world.yaml")
        env._world_param.collision_mode = "reactive"
        for _ in range(3):
            env.step()
        assert env.robot.stop_flag is False

    def test_collision_mode_undefined_warns(self, env_factory):
        """An unknown collision mode warns and behaves as unobstructed."""
        warnings_collected = []

        env = env_factory("test_collision_world.yaml")
        env._env_param.logger = Mock(
            warning=lambda msg, *a, **kw: warnings_collected.append(msg)
        )
        env._world_param.collision_mode = "unknown_mode"
        env._world_param.count = 50
        env.robot.check_status()

        assert env.robot.stop_flag is False
        assert any("unknown_mode" in w for w in warnings_collected)


class TestDesiredOmniVel:
    """desired_omni_vel near and away from the goal."""

    def test_desired_omni_vel_zero_at_goal(self, env_factory):
        """Within the goal threshold the desired velocity is zero."""
        env = env_factory("test_collision_world.yaml")
        robot = env.robot
        robot.set_goal(robot.state[:3].flatten().tolist())
        assert np.allclose(robot.desired_omni_vel, 0)
