"""
Tests for behavior functionality.

Covers individual behaviors, group behaviors, and custom behaviors.
"""

import types
from unittest.mock import Mock, patch

import numpy as np
import pytest

from irsim.config import env_param, world_param
from irsim.lib.behavior.behavior import Behavior
from irsim.lib.behavior.behavior_registry import behaviors_class_map
from irsim.lib.behavior.group_behavior import GroupBehavior
from irsim.world.object_base import ObjectBase


def _install_dummy_logger():
    class _Logger:
        def info(self, *_args, **_kwargs):
            pass

        def warning(self, *_args, **_kwargs):
            pass

        def error(self, *_args, **_kwargs):
            pass

    env_param.logger = _Logger()


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


class TestBehaviorCoverage:
    """Additional tests to cover remaining lines in behavior.py"""

    def test_behavior_warning_no_behavior(self, dummy_logger):
        """Test warning when behavior not defined in auto mode (lines 56-59)."""
        _install_dummy_logger()
        world_param.control_mode = "auto"
        world_param.count = 20

        info = types.SimpleNamespace(id=1, name="test_obj", kinematics="diff")
        b = Behavior(object_info=info, behavior_dict=None)

        vel = b.gen_vel(ego_object=None, external_objects=[])
        assert np.allclose(vel, np.zeros((2, 1)))

        world_param.control_mode = "keyboard"

    def test_behavior_class_invoke(self, dummy_logger):
        """Test class-based behavior invocation (line 72)."""
        _install_dummy_logger()

        mock_behavior_func = Mock(return_value=np.array([[1.0], [0.0]]))

        with patch.dict(
            behaviors_class_map,
            {("diff", "test_class_beh"): Mock(return_value=mock_behavior_func)},
        ):
            info = types.SimpleNamespace(id=1, name="test_obj", kinematics="diff")
            b = Behavior(
                object_info=info,
                behavior_dict={"name": "test_class_beh"},
            )

            if callable(b._invoke_func):
                ego = Mock()
                b.gen_vel(ego_object=ego, external_objects=[])
                mock_behavior_func.assert_called()

    def test_behavior_class_init_exception(self, dummy_logger):
        """Test exception handling in _init_behavior_class (lines 109-114)."""
        _install_dummy_logger()

        mock_class = Mock(side_effect=ValueError("Init failed"))

        with patch.dict(behaviors_class_map, {("diff", "failing_beh"): mock_class}):
            info = types.SimpleNamespace(id=1, name="test_obj", kinematics="diff")
            b = Behavior(
                object_info=info,
                behavior_dict={"name": "failing_beh"},
            )
            assert b._invoke_func is None

    def test_behavior_filter_by_role(self, dummy_logger):
        """Test behavior filters external objects by role (lines 65-68)."""
        _install_dummy_logger()

        info = types.SimpleNamespace(id=1, name="test_obj", kinematics="diff")
        b = Behavior(
            object_info=info,
            behavior_dict={"name": "dash", "target_roles": "robot"},
        )

        robot_obj = Mock()
        robot_obj.role = "robot"

        obstacle_obj = Mock()
        obstacle_obj.role = "obstacle"

        ego = Mock()
        ego.state = np.array([[0], [0], [0]])
        ego.goal = np.array([[1], [1]])
        ego.goal_threshold = 0.1
        ego.vel_max = np.array([[1.0], [1.0]])
        ego.info = Mock()
        ego.info.acce = np.array([[1.0], [1.0]])
        ego.get_vel_range = Mock(
            return_value=(np.array([[-1.0], [-1.0]]), np.array([[1.0], [1.0]]))
        )
        ego._world_param.step_time = 0.1
        ego.max_speed = 1.0

        # This should filter out the obstacle and keep only the robot
        _ = b.gen_vel(ego_object=ego, external_objects=[robot_obj, obstacle_obj])

    def test_load_behavior_import_error(self, dummy_logger):
        """Test ImportError handling in load_behavior (lines 126-127)."""
        _install_dummy_logger()

        info = types.SimpleNamespace(id=1, name="test_obj", kinematics="diff")
        b = Behavior(object_info=info, behavior_dict={})

        b.load_behavior(".non_existent_module_xyz")


class TestBehaviorMethodsGoalNone:
    """Tests for behavior methods when goal is None."""

    def test_diff_rvo_goal_none(self, dummy_logger):
        """Test diff rvo behavior returns zeros when goal is None."""
        from irsim.lib.behavior.behavior_methods import beh_diff_rvo

        _install_dummy_logger()

        ego = Mock()
        ego.goal = None
        ego._world_param = Mock()
        ego._world_param.count = 10  # divisible by 10 to trigger warning
        ego.logger = Mock()

        result = beh_diff_rvo(ego, [])
        assert np.allclose(result, np.zeros((2, 1)))
        ego.logger.warning.assert_called_once()

    def test_omni_dash_goal_none(self, dummy_logger):
        """Test omni dash behavior returns zeros when goal is None."""
        from irsim.lib.behavior.behavior_methods import beh_omni_dash

        _install_dummy_logger()

        ego = Mock()
        ego.goal = None
        ego._world_param = Mock()
        ego._world_param.count = 10  # divisible by 10 to trigger warning
        ego.logger = Mock()

        result = beh_omni_dash(ego, [])
        assert np.allclose(result, np.zeros((2, 1)))
        ego.logger.warning.assert_called_once()

    def test_omni_angular_dash_goal_none(self, dummy_logger):
        """Test omni_angular dash behavior returns zeros when goal is None."""
        from irsim.lib.behavior.behavior_methods import beh_omni_angular_dash

        _install_dummy_logger()

        ego = Mock()
        ego.goal = None
        ego._world_param = Mock()
        ego._world_param.count = 10
        ego.logger = Mock()

        result = beh_omni_angular_dash(ego, [])
        assert np.allclose(result, np.zeros((3, 1)))
        ego.logger.warning.assert_called_once()

    def test_omni_angular_dash_with_goal(self, dummy_logger):
        """Test omni_angular dash behavior with valid goal."""
        from irsim.lib.behavior.behavior_methods import beh_omni_angular_dash

        _install_dummy_logger()

        ego = Mock()
        ego.state = np.array([[0.0], [0.0], [0.0]])
        ego.goal = np.array([[5.0], [3.0], [1.0]])
        ego.goal_threshold = 0.3
        ego.vel_max = np.array([[1.0], [1.0], [0.5]])
        ego.info = Mock()
        ego.info.acce = np.array([[1.0], [1.0], [3.0]])
        ego.get_vel_range = Mock(
            return_value=(
                np.array([[-1.0], [-1.0], [-0.5]]),
                np.array([[1.0], [1.0], [0.5]]),
            )
        )
        ego._world_param = Mock()
        ego._world_param.step_time = 0.1
        ego.logger = Mock()

        result = beh_omni_angular_dash(ego, [])
        assert result.shape == (3, 1)

    def test_omni_angular_dash_yaw_deceleration(self, dummy_logger):
        """OmniAngularDash decel ramp: caps yaw below max, mirrors CCW/CW, inf → bang-bang."""
        from irsim.lib.behavior.behavior_methods import OmniAngularDash

        state = np.array([[0.0], [0.0], [0.0]])
        goal_pos = np.array([[5.0], [0.0], [0.3]])  # goal heading 0.3 rad away
        max_vel = np.array([[1.0], [1.0], [3.0]])
        dt = 0.1

        res = OmniAngularDash(
            state,
            goal_pos,
            max_vel,
            goal_threshold=6.0,
            angle_tolerance=0.05,
            angular_acce=3.0,
            dt=dt,
        )
        assert abs(res[2, 0]) < 3.0, "decel ramp should limit yaw rate"

        # Discrete-time ramp: ω·dt + ω²/(2a) ≤ remaining → no overshoot in one step
        remaining = 0.3 - 0.05
        a, omega = 3.0, abs(res[2, 0])
        assert omega * dt + omega**2 / (2 * a) <= remaining + 1e-9, (
            "overshoot in one step"
        )

        # CW goal must produce same magnitude as CCW
        goal_cw = np.array([[5.0], [0.0], [-0.3]])
        res_cw = OmniAngularDash(
            state,
            goal_cw,
            max_vel,
            goal_threshold=6.0,
            angle_tolerance=0.05,
            angular_acce=3.0,
            dt=dt,
        )
        assert abs(res[2, 0]) == pytest.approx(abs(res_cw[2, 0]))
        assert res_cw[2, 0] < 0

        # inf acce → bang-bang (full yaw)
        res_inf = OmniAngularDash(
            state,
            goal_pos,
            max_vel,
            goal_threshold=6.0,
            angle_tolerance=0.05,
        )
        assert res_inf[2, 0] == pytest.approx(3.0)

    def test_omni_angular_dash_yaw_capped_at_max_vel(self, dummy_logger):
        """Large heading error: the decel ramp is capped at max_vel, not exceeded."""
        from irsim.lib.behavior.behavior_methods import OmniAngularDash

        # At goal position, but goal heading is ~pi away -> large remaining, so the
        # uncapped decel ramp (~3.9) would exceed the 3.0 yaw-rate limit.
        state = np.array([[0.0], [0.0], [0.0]])
        goal = np.array([[0.0], [0.0], [3.0]])
        max_vel = np.array([[1.0], [1.0], [3.0]])

        res = OmniAngularDash(
            state,
            goal,
            max_vel,
            goal_threshold=0.5,
            angle_tolerance=0.05,
            angular_acce=3.0,
            dt=0.1,
        )
        assert res[2, 0] == pytest.approx(3.0)

    def test_diff_dash_angular_capped_at_max_vel(self, dummy_logger):
        """Large heading error: DiffDash decel ramp is capped at max_vel."""
        from irsim.lib.behavior.behavior_methods import DiffDash

        # Goal directly behind -> heading error ~pi, so the uncapped ramp (~4.0)
        # would exceed the 2.0 angular limit.
        state = np.array([[0.0], [0.0], [0.0]])
        goal = np.array([[-5.0], [0.0]])
        max_vel = np.array([[1.0], [2.0]])

        res = DiffDash(
            state,
            goal,
            max_vel,
            goal_threshold=0.1,
            angle_tolerance=0.05,
            angular_acce=3.0,
            dt=0.1,
        )
        assert abs(res[1, 0]) == pytest.approx(2.0)


class TestBehaviorMethodsFunctions:
    """Tests for standalone behavior method functions."""

    def test_omni_rvo_neighbor_none(self):
        """Test OmniRVO with neighbor_list=None (line 245)."""
        from irsim.lib.behavior.behavior_methods import OmniRVO

        # State: (x, y, theta, vx, vy, goal_x, goal_y)
        state_tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0)
        result = OmniRVO(state_tuple, neighbor_list=None)
        assert result.shape == (2, 1)

    def test_diff_rvo_neighbor_none(self):
        """Test DiffRVO with neighbor_list=None (line 290)."""
        from irsim.lib.behavior.behavior_methods import DiffRVO

        # State: (x, y, theta, vx, vy, goal_x, goal_y)
        state_tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0)
        result = DiffRVO(state_tuple, neighbor_list=None)
        assert result.shape == (2, 1)

    def test_diff_rvo_goal_behind_rotates(self):
        """DiffRVO must turn around instead of freezing when the goal is behind.

        Holonomic RVO collapses toward zero velocity when the goal lies behind
        the robot (it cannot encode a reversal). With a tight acceleration
        window the converted command would be all-zeros and a diff robot would
        freeze forever facing away from its goal. With no neighbors in range the
        behavior should instead rotate in place toward the desired heading.
        """
        from irsim.lib.behavior.behavior_methods import DiffRVO

        # rvo_state = [x, y, vx, vy, radius, vx_des, vy_des, theta]
        # robot faces +x (theta=0) but the desired velocity points behind it.
        state_tuple = [0.0, 0.0, 0.0, 0.0, 0.5, -1.0, 0.0, 0.0]
        # Tight acce window forces the RVO solution below the deadband.
        result = DiffRVO(state_tuple, neighbor_list=[], acce=0.01)

        assert result.shape == (2, 1)
        assert result[0, 0] == 0.0  # no forward creep while turning around
        assert abs(result[1, 0]) > 0.1  # rotates toward the goal instead of freezing

    def test_omni_dash_at_goal(self):
        """Test OmniDash when at goal."""
        from irsim.lib.behavior.behavior_methods import OmniDash

        state = np.array([[5.0], [5.0], [0.0]])
        goal = np.array([[5.0], [5.0]])  # Same position as state
        max_vel = np.array([[1.0], [1.0]])

        result = OmniDash(state, goal, max_vel, goal_threshold=0.5)
        assert np.allclose(result, np.zeros((2, 1)))

    def test_omni_dash_moving_body_frame(self):
        """Test OmniDash produces body-frame velocity toward goal."""
        from irsim.lib.behavior.behavior_methods import OmniDash

        # Robot at origin facing +x, goal ahead
        state = np.array([[0.0], [0.0], [0.0]])
        goal = np.array([[5.0], [0.0]])
        max_vel = np.array([[1.0], [1.0]])

        result = OmniDash(state, goal, max_vel, goal_threshold=0.1)
        assert result.shape == (2, 1)
        assert result[0, 0] > 0  # forward component positive

    def test_omni_dash_with_heading(self):
        """Test OmniDash body-frame velocity with non-zero heading."""
        from irsim.lib.behavior.behavior_methods import OmniDash

        # Robot at origin facing +y (pi/2), goal at (5, 0) = to the right
        state = np.array([[0.0], [0.0], [np.pi / 2]])
        goal = np.array([[5.0], [0.0]])
        max_vel = np.array([[1.0], [1.0]])

        result = OmniDash(state, goal, max_vel, goal_threshold=0.1)
        assert result.shape == (2, 1)
        # Goal is to the right of heading, so lateral should be negative
        assert result[1, 0] < 0

    def test_omni_angular_dash_moving(self):
        """Test OmniAngularDash drives toward goal with yaw."""
        from irsim.lib.behavior.behavior_methods import OmniAngularDash

        state = np.array([[0.0], [0.0], [0.0]])
        goal = np.array([[5.0], [3.0], [1.0]])
        max_vel = np.array([[1.0], [1.0], [0.5]])

        result = OmniAngularDash(state, goal, max_vel, goal_threshold=0.3)
        assert result.shape == (3, 1)
        # Should be moving forward and turning toward goal
        assert result[0, 0] != 0 or result[1, 0] != 0
        assert result[2, 0] != 0  # yaw rate active

    def test_omni_angular_dash_at_goal_rotate_in_place(self):
        """Test OmniAngularDash rotates in place when at goal position."""
        from irsim.lib.behavior.behavior_methods import OmniAngularDash

        state = np.array([[5.0], [5.0], [0.0]])
        goal = np.array([[5.0], [5.0], [1.5]])  # same position, different angle
        max_vel = np.array([[1.0], [1.0], [0.5]])

        result = OmniAngularDash(state, goal, max_vel, goal_threshold=0.5)
        assert result.shape == (3, 1)
        assert result[0, 0] == 0  # no forward
        assert result[1, 0] == 0  # no lateral
        assert result[2, 0] > 0  # rotating toward goal angle

    def test_omni_angular_dash_at_goal_aligned(self):
        """Test OmniAngularDash stops when at goal and angle aligned."""
        from irsim.lib.behavior.behavior_methods import OmniAngularDash

        state = np.array([[5.0], [5.0], [1.0]])
        goal = np.array([[5.0], [5.0], [1.0]])  # same position and angle
        max_vel = np.array([[1.0], [1.0], [0.5]])

        result = OmniAngularDash(
            state, goal, max_vel, goal_threshold=0.5, angle_tolerance=0.1
        )
        assert np.allclose(result, np.zeros((3, 1)))

    def test_omni_angular_dash_goal_no_theta(self):
        """Test OmniAngularDash with 2D goal (no theta component)."""
        from irsim.lib.behavior.behavior_methods import OmniAngularDash

        state = np.array([[5.0], [5.0], [0.5]])
        goal = np.array([[5.0], [5.0]])  # 2x1 goal, no theta
        max_vel = np.array([[1.0], [1.0], [0.5]])

        result = OmniAngularDash(
            state, goal, max_vel, goal_threshold=0.5, angle_tolerance=0.1
        )
        assert result.shape == (3, 1)
        # At goal position, goal has no theta so target_angle = theta => no rotation
        assert result[2, 0] == 0

    def test_diff_dash_at_goal_and_aligned(self):
        """Test DiffDash at goal and angle aligned (lines 398, 401)."""
        from irsim.lib.behavior.behavior_methods import DiffDash

        state = np.array([[5.0], [5.0], [0.0]])
        goal = np.array([[5.0], [5.0]])
        max_vel = np.array([[1.0], [0.5]])

        result = DiffDash(state, goal, max_vel, goal_threshold=0.5, angle_tolerance=0.2)
        assert np.allclose(result, np.zeros((2, 1)))

    def test_diff_dash_angle_within_tolerance(self):
        """Test DiffDash when angle is within tolerance (line 398)."""
        from irsim.lib.behavior.behavior_methods import DiffDash

        # Robot facing roughly toward goal (within tolerance)
        state = np.array([[0.0], [0.0], [0.1]])  # theta = 0.1
        goal = np.array([[5.0], [0.1]])  # Goal roughly in front
        max_vel = np.array([[1.0], [0.5]])

        result = DiffDash(state, goal, max_vel, goal_threshold=0.1, angle_tolerance=0.2)
        assert result.shape == (2, 1)
        # Should be moving forward, not at goal
        assert result[0, 0] > 0


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
