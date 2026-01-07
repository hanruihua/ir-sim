"""
Tests for robot functionality.

Covers robot properties, state management, velocity, lidar, and behaviors.
"""

import numpy as np
import pytest


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
        """Test setting random goal that avoids obstacles."""
        env = env_factory("test_collision_world.yaml")
        goal_check_radius = 0.4
        env_objects = env.obstacle_list

        for _ in range(10):
            env.robot.set_random_goal(env_objects, goal_check_radius=goal_check_radius)
            goal = env.robot._goal[0]
            assert len(goal) == 3

    def test_set_random_goal_with_limits(self, env_factory):
        """Test setting random goal with range limits."""
        env = env_factory("test_collision_world.yaml")
        env.robot.set_goal([[5, 10, 0], [5, 9, 0], [5, 8, 0]], init=True)

        for _ in range(10):
            env.robot.set_random_goal(
                env.obstacle_list,
                goal_check_radius=0.4,
                range_limits=[[3, 3, -3.141592653589793], [7, 7, 3.141592653589793]],
            )
            goals = env.robot._goal
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
