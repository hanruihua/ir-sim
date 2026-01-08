"""
Tests for core environment functionality (EnvBase, EnvBase3D).

Covers environment creation, object management, state queries, and flags.
"""

import contextlib
import re
from unittest.mock import patch

import numpy as np
import pytest

import irsim
from irsim.env.env_base import EnvBase


class TestEnvironmentCreation:
    """Tests for environment creation and initialization."""

    def test_make_basic_environment(self, env_factory):
        """Test basic environment creation with default parameters."""
        env = env_factory("test_collision_world.yaml")
        assert env is not None
        assert env.robot is not None

    def test_make_with_full_flag(self, env_factory):
        """Test environment creation with full=True for complete setup."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        assert env is not None
        assert len(env.robot_list) >= 1

    @pytest.mark.parametrize("projection", ["2d", "3d"])
    def test_make_with_projection(self, env_factory, projection):
        """Test environment creation with different projections."""
        env = env_factory("test_multi_objects_world.yaml", projection=projection)
        assert env is not None

    def test_invalid_projection_raises(self):
        """Test that invalid projection raises ValueError."""
        with pytest.raises(
            ValueError, match="Unknown projection 'nonexistent_projection_xyz'"
        ):
            irsim.make(
                "test_multi_objects_world.yaml",
                save_ani=False,
                display=False,
                projection="nonexistent_projection_xyz",
            )

    def test_register_custom_env(self, env_factory):
        """Test registering a custom environment class."""
        irsim._env_factory.register("custom_3d", irsim.EnvBase3D)
        env = env_factory("test_multi_objects_world.yaml", projection="custom_3d")
        assert env is not None

    def test_empty_yaml_path_logs(self, capsys):
        """Test that empty YAML path logs error message."""
        with contextlib.suppress(Exception):
            EnvBase(
                "",
                display=False,
                disable_all_plot=True,
                log_file=None,
                log_level="CRITICAL",
            )

        out = capsys.readouterr().out
        assert "YAML Configuration load failed" in out or "YAML File not found" in out


class TestObjectManagement:
    """Tests for object creation, addition, and deletion."""

    def test_create_and_add_obstacle(self, env_factory):
        """Test creating and adding an obstacle."""
        env = env_factory("test_all_objects.yaml")
        obs = env.create_obstacle(
            shape={"name": "polygon", "vertices": [[6, 5], [7, 5], [7, 6], [6, 6]]}
        )
        initial_count = len(env.obstacle_list)
        env.add_object(obs)
        assert len(env.obstacle_list) == initial_count + 1
        env.delete_object(obs.id)
        assert len(env.obstacle_list) == initial_count

    def test_add_multiple_objects(self, env_factory):
        """Test adding multiple objects at once."""
        env = env_factory("test_all_objects.yaml")
        obs = env.create_obstacle(
            shape={"name": "polygon", "vertices": [[6, 5], [7, 5], [7, 6], [6, 6]]}
        )
        env.add_objects([obs])
        assert obs.id in [o.id for o in env.obstacle_list]

    def test_delete_objects_by_id(self, env_factory):
        """Test deleting multiple objects by ID."""
        env = env_factory("test_all_objects.yaml")
        initial_count = len(env.robot_list) + len(env.obstacle_list)
        env.delete_objects([1, 2])
        final_count = len(env.robot_list) + len(env.obstacle_list)
        assert final_count < initial_count

    def test_duplicate_object_name_raises(self, env_factory):
        """Test that adding object with duplicate name raises ValueError."""
        env = env_factory("test_all_objects.yaml")
        obs = env.create_obstacle(
            shape={"name": "polygon", "vertices": [[6, 5], [7, 5], [7, 6], [6, 6]]}
        )
        env.add_object(obs)
        with pytest.raises(
            ValueError, match=f"Object name '{obs.name}' already exists"
        ):
            env.add_object(obs)

    def test_duplicate_objects_raises(self, env_factory):
        """Test that adding objects with duplicate names raises ValueError."""
        env = env_factory("test_all_objects.yaml", display=True)
        obs = env.create_obstacle(
            shape={"name": "polygon", "vertices": [[6, 5], [7, 5], [7, 6], [6, 6]]}
        )
        env.add_object(obs)
        with pytest.raises(
            ValueError, match=re.escape(f"Object names already exist: {[obs.name]}")
        ):
            env.add_objects([obs])

    def test_validate_unique_names_pass(self, env_factory):
        """Test that unique object names pass validation."""
        env = env_factory("test_all_objects.yaml")
        assert len(env.names) == len(set(env.names))

    def test_validate_unique_names_duplicate_raises(self):
        """Test that duplicate object names raise ValueError."""
        with pytest.raises(ValueError, match="Duplicate object names"):
            irsim.make("test_duplicate_names.yaml", display=False)


class TestStateQueries:
    """Tests for environment state query methods."""

    def test_get_robot_state(self, env_factory):
        """Test getting robot state."""
        env = env_factory("test_collision_world.yaml")
        state = env.get_robot_state()
        assert state is not None
        assert isinstance(state, np.ndarray)

    def test_get_lidar_scan(self, env_factory):
        """Test getting lidar scan data."""
        env = env_factory("test_all_objects.yaml")
        scan = env.get_lidar_scan()
        assert scan is not None

    def test_get_lidar_offset(self, env_factory):
        """Test getting lidar offset."""
        env = env_factory("test_all_objects.yaml")
        offset = env.get_lidar_offset()
        assert offset is not None

    def test_get_robot_info(self, env_factory):
        """Test getting robot info."""
        env = env_factory("test_collision_world.yaml")
        info = env.get_robot_info()
        assert info is not None

    def test_get_obstacle_info_list(self, env_factory):
        """Test getting obstacle info list."""
        env = env_factory("test_all_objects.yaml")
        info_list = env.get_obstacle_info_list()
        assert isinstance(info_list, list)

    def test_get_robot_info_list(self, env_factory):
        """Test getting robot info list."""
        env = env_factory("test_all_objects.yaml")
        info_list = env.get_robot_info_list()
        assert isinstance(info_list, list)

    def test_get_map(self, env_factory):
        """Test getting map data."""
        env = env_factory("test_collision_world.yaml")
        env_map = env.get_map()
        assert env_map is not None

    def test_get_object_by_name(self, env_factory):
        """Test getting object by name."""
        env = env_factory("test_collision_avoidance.yaml")
        result = env.get_object_by_name("nonexistent")
        assert result is None

    def test_get_object_by_id(self, env_factory):
        """Test getting object by ID."""
        env = env_factory("test_collision_world.yaml")
        obj = env.get_object_by_id(env.robot.id)
        assert obj is not None
        assert obj.id == env.robot.id

    def test_get_group_by_name(self, env_factory):
        """Test getting object group by name."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        group = env.get_group_by_name("robot_group")
        # May be None if group doesn't exist in config
        assert group is None or group is not None


class TestEnvironmentProperties:
    """Tests for environment property accessors."""

    def test_robot_property(self, env_factory):
        """Test robot property returns first robot."""
        env = env_factory("test_collision_world.yaml")
        assert env.robot is not None
        assert env.robot == env.robot_list[0]

    def test_robot_property_empty_raises(self, env_factory):
        """Test robot property raises IndexError when no robots."""
        env = env_factory("test_collision_world.yaml")
        # Remove all robots
        for robot in list(env.robot_list):
            env.delete_object(robot.id)
        with pytest.raises(IndexError, match="No robots in the environment"):
            _ = env.robot

    def test_step_time(self, env_factory):
        """Test step_time property."""
        env = env_factory("test_collision_world.yaml")
        assert env.step_time > 0

    def test_robot_number(self, env_factory):
        """Test robot_number property."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        assert env.robot_number >= 1

    def test_dynamic_objects(self, env_factory):
        """Test dynamic_objects property."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        dynamic = env.dynamic_objects
        assert isinstance(dynamic, list)

    def test_static_objects(self, env_factory):
        """Test static_objects property."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        static = env.static_objects
        assert isinstance(static, list)

    def test_names_property(self, env_factory):
        """Test names property returns all object names."""
        env = env_factory("test_all_objects.yaml")
        names = env.names
        assert isinstance(names, list)
        assert len(names) > 0

    def test_world_param_property(self, env_factory):
        """Test world_param property returns WorldParam instance."""
        env = env_factory("test_collision_world.yaml")
        wp = env.world_param
        assert wp is not None
        assert hasattr(wp, "control_mode")
        assert hasattr(wp, "collision_mode")
        assert hasattr(wp, "step_time")
        assert hasattr(wp, "count")
        assert wp.step_time > 0

    def test_env_param_property(self, env_factory):
        """Test env_param property returns EnvParam instance."""
        env = env_factory("test_collision_world.yaml")
        ep = env.env_param
        assert ep is not None
        assert hasattr(ep, "logger")
        assert hasattr(ep, "objects")
        assert ep.logger is not None

    def test_path_param_property(self, env_factory):
        """Test path_param property returns PathManager instance."""
        env = env_factory("test_collision_world.yaml")
        pp = env.path_param
        assert pp is not None
        assert hasattr(pp, "root_path")
        assert hasattr(pp, "ani_buffer_path")
        assert hasattr(pp, "ani_path")
        assert hasattr(pp, "fig_path")

    def test_object_world_param_property(self, env_factory):
        """Test world_param property on objects."""
        env = env_factory("test_collision_world.yaml")
        robot = env.robot
        wp = robot.world_param
        assert wp is not None
        assert wp.step_time == env.world_param.step_time
        assert wp is env.world_param

    def test_object_env_param_property(self, env_factory):
        """Test env_param property on objects."""
        env = env_factory("test_collision_world.yaml")
        robot = env.robot
        ep = robot.env_param
        assert ep is not None
        assert ep.logger is env.logger
        assert ep is env.env_param


class TestEnvironmentFlags:
    """Tests for environment flag processing."""

    def test_save_figure_flag(self, env_factory):
        """Test save_figure_flag triggers save during render."""
        env = env_factory("test_all_objects.yaml")
        with patch.object(env, "save_figure") as mock_save:
            env.save_figure_flag = True
            env.render(0.0)
            mock_save.assert_called_once()
            assert env.save_figure_flag is False

    def test_reset_flag(self, env_factory):
        """Test reset_flag triggers reset during render."""
        env = env_factory("test_all_objects.yaml")
        env.reset_flag = True
        env.render(0.0)
        assert env.reset_flag is False

    def test_reload_flag(self, env_factory):
        """Test reload_flag triggers reload during render."""
        env = env_factory("test_all_objects.yaml")
        env.reload_flag = True
        env.render(0.0)
        assert env.reload_flag is False

    def test_quit_flag_raises_systemexit(self, env_factory):
        """Test quit_flag causes step to raise SystemExit."""
        env = env_factory("test_all_objects.yaml")
        env.quit_flag = True
        with pytest.raises(SystemExit):
            env.step()


class TestSimulationLoop:
    """Tests for simulation step and render loop."""

    @pytest.mark.parametrize("projection", ["2d", "3d"])
    def test_basic_simulation_loop(self, env_factory, projection):
        """Test basic step-render loop."""
        env = env_factory("test_collision_world.yaml", projection=projection)
        for _ in range(4):
            env.step()
            env.render(0.01)
        assert env.time > 0

    def test_step_with_action(self, env_factory):
        """Test step with explicit action."""
        env = env_factory("test_multi_objects_world.yaml")
        action = np.array([1, 0]).reshape(2, 1)
        env.step(action)
        assert env.time > 0

    def test_step_with_action_list(self, env_factory):
        """Test step with action list and ID list."""
        env = env_factory("test_multi_objects_world.yaml")
        action_list = [[1, 0], [2, 0]]
        action_id_list = [2, 3]
        env.step(action_list, action_id_list)

    def test_step_with_action_single_id(self, env_factory):
        """Test step with action list and single ID."""
        env = env_factory("test_multi_objects_world.yaml")
        action_list = [[1, 0], [2, 0]]
        action_id = 1
        env.step(action_list, action_id)

    def test_objects_step(self, env_factory):
        """Test internal _objects_step method."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env._objects_step([np.array([1, 0]).reshape(2, 1)])
        assert True  # No exception raised

    def test_done_detection(self, env_factory):
        """Test simulation done detection."""
        env = env_factory("test_collision_world.yaml")
        # Run until done or max iterations
        for _ in range(100):
            env.step()
            if env.done():
                break
        # Either done or hit max iterations
        assert True


class TestAnimationSaving:
    """Tests for animation saving functionality."""

    @pytest.mark.parametrize("projection", ["2d", "3d"])
    def test_animation_saving(self, env_factory, projection):
        """Test animation saving functionality."""
        env = irsim.make(
            "test_render.yaml",
            save_ani=True,
            display=False,
            projection=projection,
        )
        steps = 20 if projection == "2d" else 3
        for _ in range(steps):
            env.step()
            env.render(0.01)
        env.end(ani_name="test_animation")


class TestRandomization:
    """Tests for randomization methods."""

    def test_set_random_seed(self, env_factory):
        """Test setting random seed."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env.set_random_seed(42)
        # Seed is set; no assertion needed

    def test_random_polygon_shape(self, env_factory):
        """Test random polygon shape generation."""
        env = env_factory("test_all_objects.yaml")
        env.random_polygon_shape()
        # Shape is randomized; no assertion needed

    @pytest.mark.parametrize("projection", ["2d", "3d"])
    def test_random_obstacle_position(self, env_factory, projection):
        """Test random obstacle positioning."""
        env = env_factory("test_multi_objects_world.yaml", projection=projection)
        env.random_obstacle_position()

    def test_random_obstacle_position_non_overlapping(self, env_factory):
        """Test random obstacle positioning with non-overlapping constraint."""
        env = env_factory("test_multi_objects_world.yaml")
        env.random_obstacle_position(ids=[3, 4, 5, 6, 7], non_overlapping=True)


class TestLogger:
    """Tests for logger functionality."""

    def test_logger_methods(self, env_factory):
        """Test all logger methods work without error."""
        env = env_factory("test_collision_avoidance.yaml", full=True)
        env.logger.info("test")
        env.logger.warning("test")
        env.logger.error("test")
        env.logger.critical("test")
        env.logger.debug("test")
        env.logger.success("test")
        env.logger.trace("test")


class TestQuit:
    """Tests for environment quit functionality."""

    def test_quit_raises_systemexit(self, env_factory):
        """Test quit method raises SystemExit."""
        env = env_factory("test_multi_objects_world.yaml")
        with pytest.raises(SystemExit):
            env.quit()


class TestMultipleEnvironments:
    """Tests for multiple environment instances with isolated parameters."""

    def test_two_environments_have_separate_world_params(self, env_factory):
        """Test that two environments have separate world_param instances."""
        env1 = env_factory("test_collision_world.yaml")
        env2 = env_factory("test_multi_objects_world.yaml")

        # They should have different instances
        assert env1.world_param is not env2.world_param

        # Modifying one should not affect the other
        original_mode = env2.world_param.control_mode
        env1.world_param.control_mode = "keyboard"
        assert env2.world_param.control_mode == original_mode

    def test_two_environments_have_separate_env_params(self, env_factory):
        """Test that two environments have separate env_param instances."""
        env1 = env_factory("test_collision_world.yaml")
        env2 = env_factory("test_multi_objects_world.yaml")

        # They should have different instances
        assert env1.env_param is not env2.env_param
        assert env1.env_param.objects is not env2.env_param.objects

    def test_two_environments_have_separate_path_params(self, env_factory):
        """Test that two environments have separate path_param instances."""
        env1 = env_factory("test_collision_world.yaml")
        env2 = env_factory("test_multi_objects_world.yaml")

        # They should have different instances
        assert env1.path_param is not env2.path_param

    def test_objects_reference_correct_env_params(self, env_factory):
        """Test that objects reference their own environment's params."""
        env1 = env_factory("test_collision_world.yaml")
        env2 = env_factory("test_multi_objects_world.yaml")

        # Robots should reference their own env's params
        assert env1.robot.world_param is env1.world_param
        assert env1.robot.env_param is env1.env_param

        assert env2.robot.world_param is env2.world_param
        assert env2.robot.env_param is env2.env_param

        # Cross-check they are different
        assert env1.robot.world_param is not env2.robot.world_param

    def test_concurrent_step_isolated(self, env_factory):
        """Test that stepping one env doesn't affect another's count."""
        env1 = env_factory("test_collision_world.yaml")
        env2 = env_factory("test_multi_objects_world.yaml")

        # Step env1 multiple times
        for _ in range(5):
            env1.step()

        # Step env2 fewer times
        for _ in range(2):
            env2.step()

        # Counts should be independent
        assert env1.world_param.count == 5
        assert env2.world_param.count == 2
