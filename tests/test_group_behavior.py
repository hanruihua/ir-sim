import unittest
from unittest.mock import Mock, patch

import numpy as np

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

    @patch("irsim.lib.behavior.group_behavior.log_error")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_map")
    @patch("irsim.lib.behavior.group_behavior.group_behaviors_class_map")
    def test_gen_group_vel_missing_function(
        self, mock_class_map, mock_func_map, mock_log_error
    ):
        mock_class_map.get.return_value = None
        mock_func_map.get.return_value = None

        behavior_dict = {"name": "missing_behavior"}
        gb = GroupBehavior(self.members, **behavior_dict)

        # The error is reported through the env logger (util.log_error), which
        # respects log_level, not a stdlib logging.getLogger.
        gb.gen_group_vel()
        mock_log_error.assert_called_once()
        assert (
            "No group behavior method found for category 'diff' and action 'missing_behavior'."
            in mock_log_error.call_args[0][0]
        )

    def test_update_members(self):
        gb = GroupBehavior(self.members)
        new_members = [self.member1]
        gb.update_members(new_members)
        assert gb.members == new_members


class TestGroupBehaviorCoverage:
    """Additional tests to cover remaining lines in group_behavior.py"""

    def test_gen_group_vel_warning_auto_mode(self):
        """Test warning in gen_group_vel when name/kinematics is None (lines 96-101)."""
        from unittest.mock import patch

        member = Mock(spec=ObjectBase)
        member.kinematics = None
        # gen_group_vel reads the member's world param, so the auto-mode warning
        # condition must be set there (not on the global world_param).
        member._world_param.control_mode = "auto"
        member._world_param.count = 20

        gb = GroupBehavior([member], name=None)
        with patch("irsim.lib.behavior.group_behavior.log_warning") as mock_log:
            result = gb.gen_group_vel()
        assert result == [None]
        mock_log.assert_called_once()

    def test_load_group_behaviors_import_error(self):
        """Test ImportError handling in load_group_behaviors (lines 132-133)."""
        member = Mock(spec=ObjectBase)
        member.kinematics = "diff"

        gb = GroupBehavior([member])

        gb.load_group_behaviors(".non_existent_group_behavior_xyz")


class TestOrcaGroupBehaviorCoverage:
    """Tests for OrcaGroupBehavior in group_behavior_methods.py"""

    def test_orca_ensure_pyrvo_import_error(self):
        """Test ImportError when pyrvo not installed (lines 47-50)."""
        from unittest.mock import patch

        import pytest

        with (
            patch.dict("sys.modules", {"pyrvo": None}),
            patch(
                "irsim.lib.behavior.group_behavior_methods.OrcaGroupBehavior._ensure_pyrvo",
                side_effect=ImportError("pyrvo not installed"),
            ),
        ):
            from irsim.lib.behavior.group_behavior_methods import OrcaGroupBehavior

            member = Mock(spec=ObjectBase)
            member.kinematics = "omni"
            member.state = np.array([[0], [0]])
            member.radius = 0.5
            member.max_speed = 1.0

            with pytest.raises(ImportError, match="pyrvo"):
                OrcaGroupBehavior([member])

    def test_orca_rebuild_sim_on_mismatch(self):
        """Test rebuild sim when agent count mismatches (lines 96-99)."""
        import pytest

        pytest.importorskip("pyrvo")
        from irsim.lib.behavior.group_behavior_methods import OrcaGroupBehavior

        member1 = Mock(spec=ObjectBase)
        member1.kinematics = "omni"
        member1.state = np.array([[0], [0]])
        member1.radius = 0.5
        member1.max_speed = 1.0
        member1.get_desired_omni_vel = Mock(return_value=np.array([[1], [0]]))
        member1._world_param = Mock()
        member1._world_param.step_time = 0.1

        orca = OrcaGroupBehavior([member1])

        member2 = Mock(spec=ObjectBase)
        member2.kinematics = "omni"
        member2.state = np.array([[1], [1]])
        member2.radius = 0.5
        member2.max_speed = 1.0
        member2.get_desired_omni_vel = Mock(return_value=np.array([[0], [1]]))
        member2._world_param = Mock()
        member2._world_param.step_time = 0.1

        result = orca([member1, member2])
        assert len(result) == 2

    def test_orca_diff_outputs_unicycle_velocity(self):
        """diff ORCA maps the holonomic plan to (linear, angular)."""
        import pytest

        pytest.importorskip("pyrvo")
        from irsim.lib.behavior.group_behavior_methods import OrcaGroupBehavior

        def make_member(theta):
            m = Mock(spec=ObjectBase)
            m.kinematics = "diff"
            m.state = np.array([[0.0], [0.0], [theta]])
            m.goal = np.array([[10.0], [0.0], [0.0]])
            m.radius = 0.2
            m.max_speed = 1.5
            m.vel_max = np.array([[1.5], [1.0]])
            m._world_param = Mock()
            m._world_param.step_time = 0.1
            return m

        # Single isolated robot already facing its goal: the holonomic plan
        # passes straight through, so it drives forward with no turn.
        facing = make_member(0.0)
        orca = OrcaGroupBehavior([facing])
        result = orca([facing])
        assert len(result) == 1
        assert result[0].shape == (2, 1)
        assert result[0][0, 0] == pytest.approx(1.5, abs=1e-6)
        assert result[0][1, 0] == pytest.approx(0.0, abs=1e-6)

        # Robot facing away from its goal turns in place instead of reversing.
        backward = make_member(np.pi)
        orca = OrcaGroupBehavior([backward])
        result = orca([backward])
        assert result[0][0, 0] == pytest.approx(0.0, abs=1e-6)
        assert abs(result[0][1, 0]) > 0.0

    def test_orca_diff_zero_when_no_goal(self):
        """diff preferred velocity is zero when a member has no goal."""
        import pytest

        pytest.importorskip("pyrvo")
        from irsim.lib.behavior.group_behavior_methods import OrcaGroupBehavior

        member = Mock(spec=ObjectBase)
        member.kinematics = "diff"
        member.state = np.array([[0.0], [0.0], [0.0]])
        member.goal = None
        member.radius = 0.2
        member.max_speed = 1.5
        member.vel_max = np.array([[1.5], [1.0]])
        member._world_param = Mock()
        member._world_param.step_time = 0.1

        orca = OrcaGroupBehavior([member])
        assert orca._pref_velocity(member) == [0.0, 0.0]

    def test_orca_call_refreshes_kinematics_from_members(self):
        """__call__ re-reads kinematics from the call-time members, so a reused
        or rebuilt handler maps the ORCA output for the current members."""
        import pytest

        pytest.importorskip("pyrvo")
        from irsim.lib.behavior.group_behavior_methods import OrcaGroupBehavior

        member = Mock(spec=ObjectBase)
        member.kinematics = "diff"
        member.state = np.array([[0.0], [0.0], [0.0]])
        member.goal = np.array([[10.0], [0.0], [0.0]])
        member.radius = 0.2
        member.max_speed = 1.5
        member.vel_max = np.array([[1.5], [1.0]])
        member._world_param = Mock()
        member._world_param.step_time = 0.1

        orca = OrcaGroupBehavior([member])
        # Simulate a stale cached kinematics (e.g. handler reused after the
        # group's members changed); __call__ must correct it.
        orca._kinematics = "omni"
        result = orca([member])

        assert orca._kinematics == "diff"
        assert result[0].shape == (2, 1)
        # diff mapping: aligned with the goal -> forward, no turn.
        assert result[0][0, 0] == pytest.approx(1.5, abs=1e-6)
        assert result[0][1, 0] == pytest.approx(0.0, abs=1e-6)

    def test_orca_diff_integration_via_make(self, tmp_path):
        """End-to-end: a diff + orca scenario dispatches to ``beh_diff_orca``
        through the registry and drives the robots toward their goals."""
        import pytest

        pytest.importorskip("pyrvo")
        import irsim

        config = tmp_path / "orca_diff.yaml"
        config.write_text(
            "world:\n"
            "  height: 20\n"
            "  width: 20\n"
            "  step_time: 0.1\n"
            "robot:\n"
            "  - number: 3\n"
            "    kinematics: {name: 'diff'}\n"
            "    shape: {name: 'circle', radius: 0.3}\n"
            "    distribution: {name: 'circle', radius: 5}\n"
            "    vel_max: [2, 1.5]\n"
            "    group_behavior: {name: 'orca', neighborDist: 8.0, safe_radius: 0.2}\n"
        )

        env = irsim.make(str(config), display=False, save_ani=False)
        try:
            start = np.array([r.state[:2, 0] for r in env.robot_list])
            for _ in range(30):
                env.step()
            end = np.array([r.state[:2, 0] for r in env.robot_list])
        finally:
            env.end()

        # ORCA produced non-zero diff velocities, so the robots moved.
        assert np.linalg.norm(end - start, axis=1).max() > 0.1


if __name__ == "__main__":
    unittest.main()
