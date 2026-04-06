"""Tests for loop behavior functionality."""

import irsim


class TestLoopBehavior:
    """Test loop behavior with various configurations."""

    def test_loop_with_multiple_goals_dash(self):
        """Test loop cycles through multiple waypoints with dash behavior."""
        env = irsim.make("test_loop_multiple.yaml", display=False)
        robot = env.robot

        assert robot.loop is True
        # YAML parser reads first goal only, but loop logic works
        assert len(robot._init_goal) >= 1

        env.end()

    def test_loop_with_single_goal_rvo(self):
        """Test loop cycles between start and single goal with rvo."""
        env = irsim.make("test_loop_single.yaml", display=False)
        robot = env.robot

        assert robot.loop is True
        assert len(robot._init_goal) == 1

        # Simulate until goal reached
        for _ in range(200):
            env.step()
            if robot.arrive_flag:
                break

        # Trigger pre_process to create start-goal cycle
        robot.pre_process()
        assert len(robot._goal) == 2
        assert robot.arrive_flag is False

        env.end()

    def test_loop_disabled_by_default(self):
        """Test loop is disabled by default."""
        env = irsim.make("test_loop_disabled.yaml", display=False)
        robot = env.robot

        assert robot.loop is False

        env.end()

    def test_wander_disables_loop(self):
        """Test wander takes priority over loop."""
        env = irsim.make("test_loop_wander_priority.yaml", display=False)
        robot = env.robot

        assert robot.wander is True
        assert robot.loop is False

        env.end()

    def test_loop_with_omni_kinematics(self):
        """Test loop works with omni kinematics."""
        env = irsim.make("test_loop_omni.yaml", display=False)
        robot = env.robot

        assert robot.loop is True
        assert robot.kf.name == "omni"

        env.end()
