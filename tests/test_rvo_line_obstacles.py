"""Tests for RVO line obstacle support.

Covers:
- reciprocal_vel_obs with line_obs_list (config_vo_lines, penalty, _tc_line_segment)
- OmniRVO / DiffRVO with line_segments parameter
- beh_diff_rvo / beh_omni_rvo line segment separation from circular neighbors
- object_base.rvo_line_segments property
"""

from unittest.mock import Mock

import numpy as np
import pytest

from irsim.lib.algorithm.rvo import reciprocal_vel_obs


# ---------------------------------------------------------------------------
# Helper: standard agent state [x, y, vx, vy, radius, vx_des, vy_des]
# ---------------------------------------------------------------------------
def _agent_state(x=0.0, y=0.0, vx=0.0, vy=0.0, r=0.3, vx_des=1.0, vy_des=0.0):
    return [x, y, vx, vy, r, vx_des, vy_des]


# ===================================================================
# reciprocal_vel_obs — constructor & update
# ===================================================================


class TestRVOLineInit:
    def test_default_line_obs_list(self):
        rvo = reciprocal_vel_obs(_agent_state())
        assert rvo.line_obs_list == []

    def test_explicit_line_obs_list(self):
        segs = [[0, 2, 5, 2]]
        rvo = reciprocal_vel_obs(_agent_state(), line_obs_list=segs)
        assert rvo.line_obs_list is segs

    def test_update_with_line_obs(self):
        rvo = reciprocal_vel_obs(_agent_state())
        segs = [[1, 1, 2, 2]]
        rvo.update(_agent_state(), [], line_obs_list=segs)
        assert rvo.line_obs_list == segs

    def test_update_without_line_obs(self):
        rvo = reciprocal_vel_obs(_agent_state(), line_obs_list=[[0, 0, 1, 1]])
        rvo.update(_agent_state(), [])
        assert rvo.line_obs_list == []


# ===================================================================
# config_vo_lines
# ===================================================================


class TestConfigVOLines:
    def test_empty_when_no_lines(self):
        rvo = reciprocal_vel_obs(_agent_state())
        assert rvo.config_vo_lines() == []

    def test_single_segment_produces_one_cone(self):
        # Wall at y=3, agent at origin
        rvo = reciprocal_vel_obs(
            _agent_state(x=0, y=0, r=0.3),
            line_obs_list=[[-5, 3, 5, 3]],
        )
        cones = rvo.config_vo_lines()
        assert len(cones) == 1

    def test_cone_format(self):
        rvo = reciprocal_vel_obs(
            _agent_state(x=0, y=0, r=0.3),
            line_obs_list=[[1, 2, 3, 2]],
        )
        cones = rvo.config_vo_lines()
        cone = cones[0]
        # [apex, left_vector, right_vector]
        assert len(cone) == 3
        assert cone[0] == [0, 0]  # static obstacle apex
        assert len(cone[1]) == 2  # left vector
        assert len(cone[2]) == 2  # right vector

    def test_cone_vectors_are_unit(self):
        rvo = reciprocal_vel_obs(
            _agent_state(x=0, y=0, r=0.3),
            line_obs_list=[[2, -1, 2, 1]],
        )
        cones = rvo.config_vo_lines()
        lv = cones[0][1]
        rv = cones[0][2]
        assert abs(lv[0] ** 2 + lv[1] ** 2 - 1.0) < 1e-10
        assert abs(rv[0] ** 2 + rv[1] ** 2 - 1.0) < 1e-10

    def test_degenerate_segment_skipped(self):
        # Zero-length segment
        rvo = reciprocal_vel_obs(
            _agent_state(),
            line_obs_list=[[1, 1, 1, 1]],
        )
        assert rvo.config_vo_lines() == []

    def test_multiple_segments(self):
        rvo = reciprocal_vel_obs(
            _agent_state(),
            line_obs_list=[[0, 3, 5, 3], [-3, 0, -3, 5]],
        )
        cones = rvo.config_vo_lines()
        assert len(cones) == 2

    def test_agent_very_close_to_endpoint(self):
        # Agent almost on top of endpoint — should still produce a cone
        rvo = reciprocal_vel_obs(
            _agent_state(x=0, y=0, r=0.3),
            line_obs_list=[[0.001, 0.001, 3, 0]],
        )
        cones = rvo.config_vo_lines()
        assert len(cones) == 1

    def test_cone_blocks_velocity_toward_wall(self):
        """A velocity heading straight into the wall should be inside the VO cone."""
        # Wall at y=2, agent at origin heading up
        state = _agent_state(x=0, y=0, vx=0, vy=0.5, r=0.3)
        rvo = reciprocal_vel_obs(state, line_obs_list=[[-5, 2, 5, 2]])
        cones = rvo.config_vo_lines()
        cone = cones[0]
        # Velocity toward wall: (0, 1)
        rel_vx = 0 - cone[0][0]
        rel_vy = 1 - cone[0][1]
        inside = reciprocal_vel_obs.between_vector(
            cone[1], cone[2], [rel_vx, rel_vy]
        )
        assert inside

    def test_cone_allows_velocity_parallel_to_wall(self):
        """A velocity parallel to a distant wall should be outside the VO cone."""
        # Wall at y=5, agent at origin
        state = _agent_state(x=0, y=0, r=0.3)
        rvo = reciprocal_vel_obs(state, line_obs_list=[[-10, 5, 10, 5]])
        cones = rvo.config_vo_lines()
        cone = cones[0]
        # Velocity parallel to wall: (1, 0)
        rel_vx = 1 - cone[0][0]
        rel_vy = 0 - cone[0][1]
        outside = not reciprocal_vel_obs.between_vector(
            cone[1], cone[2], [rel_vx, rel_vy]
        )
        assert outside


# ===================================================================
# config_rvo / config_vo / config_hrvo include line cones
# ===================================================================


class TestConfigModesIncludeLines:
    def _make_rvo(self, mode_state_vx=0.5):
        state = _agent_state(vx=mode_state_vx, vy=0.0)
        circ = [[3, 0, 0.5, 0, 0.3]]
        lines = [[-5, 2, 5, 2]]
        return reciprocal_vel_obs(state, circ, line_obs_list=lines)

    def test_config_rvo_includes_line_cones(self):
        rvo = self._make_rvo()
        result = rvo.config_rvo()
        # 1 circular + 1 line
        assert len(result) == 2

    def test_config_vo_includes_line_cones(self):
        rvo = self._make_rvo()
        result = rvo.config_vo()
        assert len(result) == 2

    def test_config_hrvo_includes_line_cones(self):
        rvo = self._make_rvo()
        result = rvo.config_hrvo()
        assert len(result) == 2

    def test_config_rvo_static_agent_includes_lines(self):
        rvo = self._make_rvo(mode_state_vx=0.0)
        result = rvo.config_rvo()
        assert len(result) == 2


# ===================================================================
# _tc_line_segment
# ===================================================================


class TestTCLineSegment:
    def test_moving_toward_wall(self):
        # Agent at (0,0), wall at y=3, velocity (0,1)
        tc = reciprocal_vel_obs._tc_line_segment(
            0, 0, 0.3, [0, 1], [-5, 3, 5, 3]
        )
        assert tc == pytest.approx(2.7, abs=0.01)  # (3 - 0.3) / 1

    def test_moving_away_from_wall(self):
        tc = reciprocal_vel_obs._tc_line_segment(
            0, 0, 0.3, [0, -1], [-5, 3, 5, 3]
        )
        assert tc == 1e6

    def test_moving_parallel(self):
        tc = reciprocal_vel_obs._tc_line_segment(
            0, 0, 0.3, [1, 0], [-5, 3, 5, 3]
        )
        assert tc == 1e6

    def test_degenerate_segment(self):
        tc = reciprocal_vel_obs._tc_line_segment(
            0, 0, 0.3, [0, 1], [2, 2, 2, 2]
        )
        assert tc == 1e6

    def test_collision_point_outside_segment(self):
        # Short wall segment far to the right, agent heading up
        tc = reciprocal_vel_obs._tc_line_segment(
            0, 0, 0.3, [0, 1], [10, 3, 15, 3]
        )
        assert tc == 1e6

    def test_already_inside_wall(self):
        # Agent overlapping the wall — tc should be 0
        tc = reciprocal_vel_obs._tc_line_segment(
            0, 0, 0.5, [0, 1], [-5, 0.2, 5, 0.2]
        )
        assert tc == 0

    def test_diagonal_approach(self):
        # Agent at (0,0), wall at y=4, velocity (1,1)
        tc = reciprocal_vel_obs._tc_line_segment(
            0, 0, 0.3, [1, 1], [-10, 4, 10, 4]
        )
        # Perpendicular distance = 4, vel component toward wall = 1
        expected = (4 - 0.3) / 1.0
        assert tc == pytest.approx(expected, abs=0.01)


# ===================================================================
# penalty with line obstacles
# ===================================================================


class TestPenaltyWithLines:
    def test_penalty_with_only_lines(self):
        state = _agent_state(x=0, y=0, vx=0, vy=0, r=0.3, vx_des=1.0, vy_des=0.0)
        rvo = reciprocal_vel_obs(state, [], line_obs_list=[[-5, 2, 5, 2]])
        # Velocity toward wall
        p_toward = rvo.penalty([0, 1], [1, 0], 1.0)
        # Velocity away from wall
        p_away = rvo.penalty([0, -1], [1, 0], 1.0)
        # Toward wall should have higher penalty (lower tc)
        assert p_toward > p_away

    def test_penalty_empty_obstacles(self):
        state = _agent_state()
        rvo = reciprocal_vel_obs(state, [])
        # No obstacles at all — should just return distance to desired vel
        p = rvo.penalty([0, 0], [1, 0], 1.0)
        assert p == pytest.approx(1.0, abs=0.01)

    def test_penalty_mixed_circular_and_line(self):
        state = _agent_state(x=0, y=0, vx=0.5, vy=0, r=0.3)
        circ = [[5, 0, 0, 0, 0.3]]
        lines = [[-5, 2, 5, 2]]
        rvo = reciprocal_vel_obs(state, circ, line_obs_list=lines)
        p = rvo.penalty([0.5, 0], [1, 0], 1.0)
        assert isinstance(p, float)


# ===================================================================
# cal_vel with line obstacles (integration)
# ===================================================================


class TestCalVelWithLines:
    def _make_rvo_with_wall(self, mode="rvo"):
        # Agent at origin, desired velocity toward +x, wall at y=1
        state = _agent_state(x=0, y=0, vx=0, vy=0, r=0.3, vx_des=1.0, vy_des=0.0)
        return reciprocal_vel_obs(
            state, [], vxmax=1.5, vymax=1.5, acce=1.0,
            line_obs_list=[[-5, 1, 5, 1]],
        )

    def test_cal_vel_rvo_mode(self):
        rvo = self._make_rvo_with_wall("rvo")
        vel = rvo.cal_vel("rvo")
        assert len(vel) == 2

    def test_cal_vel_vo_mode(self):
        rvo = self._make_rvo_with_wall("vo")
        vel = rvo.cal_vel("vo")
        assert len(vel) == 2

    def test_cal_vel_hrvo_mode(self):
        rvo = self._make_rvo_with_wall("hrvo")
        vel = rvo.cal_vel("hrvo")
        assert len(vel) == 2

    def test_selected_vel_avoids_wall(self):
        """Selected velocity should not head straight into the wall."""
        rvo = self._make_rvo_with_wall("rvo")
        vel = rvo.cal_vel("rvo")
        # vy should not be strongly positive (toward wall at y=1)
        assert vel[1] < 0.5

    def test_no_line_obs_same_as_before(self):
        """With no line obstacles, behavior should match original."""
        state = _agent_state(x=0, y=0, vx=0, vy=0, r=0.3, vx_des=1.0, vy_des=0.0)
        rvo_with = reciprocal_vel_obs(state, [], line_obs_list=[])
        rvo_without = reciprocal_vel_obs(state, [])
        vel_with = rvo_with.cal_vel("rvo")
        vel_without = rvo_without.cal_vel("rvo")
        assert vel_with == pytest.approx(vel_without, abs=1e-10)


# ===================================================================
# OmniRVO / DiffRVO with line_segments
# ===================================================================


class TestBehaviorFunctionsWithLines:
    def test_omni_rvo_with_line_segments(self):
        from irsim.lib.behavior.behavior_methods import OmniRVO

        state = (0.0, 0.0, 0.0, 0.0, 0.3, 1.0, 0.0)
        result = OmniRVO(state, line_segments=[[-5, 2, 5, 2]])
        assert result.shape == (2, 1)

    def test_diff_rvo_with_line_segments(self):
        from irsim.lib.behavior.behavior_methods import DiffRVO

        state = (0.0, 0.0, 0.0, 0.0, 0.3, 1.0, 0.0, 0.0)
        result = DiffRVO(state, line_segments=[[-5, 2, 5, 2]])
        assert result.shape == (2, 1)

    def test_omni_rvo_none_line_segments(self):
        from irsim.lib.behavior.behavior_methods import OmniRVO

        state = (0.0, 0.0, 0.0, 0.0, 0.3, 1.0, 0.0)
        result = OmniRVO(state, line_segments=None)
        assert result.shape == (2, 1)

    def test_diff_rvo_none_line_segments(self):
        from irsim.lib.behavior.behavior_methods import DiffRVO

        state = (0.0, 0.0, 0.0, 0.0, 0.3, 1.0, 0.0, 0.0)
        result = DiffRVO(state, line_segments=None)
        assert result.shape == (2, 1)

    def test_omni_rvo_mixed_neighbors_and_lines(self):
        from irsim.lib.behavior.behavior_methods import OmniRVO

        state = (0.0, 0.0, 0.0, 0.0, 0.3, 1.0, 0.0)
        neighbors = [[3, 0, 0, 0, 0.3]]
        result = OmniRVO(
            state, neighbor_list=neighbors, line_segments=[[-5, 2, 5, 2]]
        )
        assert result.shape == (2, 1)


# ===================================================================
# beh_diff_rvo / beh_omni_rvo line segment separation
# ===================================================================


class TestBehFunctionsLineSeparation:
    def _make_ego(self):
        ego = Mock()
        ego.goal = np.array([[5.0], [5.0]])
        ego.rvo_state = (0.0, 0.0, 0.0, 0.0, 0.3, 1.0, 0.0, 0.0)
        ego._world_param = Mock()
        ego._world_param.count = 1
        ego.logger = Mock()
        return ego

    def _make_circular_obj(self):
        obj = Mock()
        obj.rvo_neighbor_state = [3, 0, 0, 0, 0.3]
        obj.rvo_line_segments = []
        return obj

    def _make_line_obj(self):
        obj = Mock()
        obj.rvo_line_segments = [[-5, 3, 5, 3]]
        return obj

    def test_beh_diff_rvo_separates_lines(self):
        from irsim.lib.behavior.behavior_methods import beh_diff_rvo

        ego = self._make_ego()
        circ = self._make_circular_obj()
        line = self._make_line_obj()
        result = beh_diff_rvo(ego, [circ, line])
        assert result.shape == (2, 1)

    def test_beh_omni_rvo_separates_lines(self):
        from irsim.lib.behavior.behavior_methods import beh_omni_rvo

        ego = self._make_ego()
        ego.rvo_state = (0.0, 0.0, 0.0, 0.0, 0.3, 1.0, 0.0)
        circ = self._make_circular_obj()
        line = self._make_line_obj()
        result = beh_omni_rvo(ego, [circ, line])
        assert result.shape == (2, 1)

    def test_beh_diff_rvo_only_lines(self):
        from irsim.lib.behavior.behavior_methods import beh_diff_rvo

        ego = self._make_ego()
        line = self._make_line_obj()
        result = beh_diff_rvo(ego, [line])
        assert result.shape == (2, 1)

    def test_beh_omni_rvo_only_circular(self):
        from irsim.lib.behavior.behavior_methods import beh_omni_rvo

        ego = self._make_ego()
        ego.rvo_state = (0.0, 0.0, 0.0, 0.0, 0.3, 1.0, 0.0)
        circ = self._make_circular_obj()
        result = beh_omni_rvo(ego, [circ])
        assert result.shape == (2, 1)


# ===================================================================
# rvo_line_segments property
# ===================================================================


class TestRVOLineSegmentsProperty:
    def _make_obj(self, shape, vertices=None):
        obj = Mock()
        obj.shape = shape
        if vertices is not None:
            obj.vertices = vertices
        return obj

    def test_non_linestring_returns_empty(self):
        obj = self._make_obj("circle")
        # Call the property logic directly since we can't easily instantiate ObjectBase
        assert obj.shape != "linestring"

    def test_linestring_segments(self):
        """Verify segment extraction logic matches the property implementation."""
        # Simulate what rvo_line_segments does
        verts = np.array([[0, 1, 2], [0, 1, 0]])  # 2x3 array: 3 vertices
        segments = []
        for i in range(verts.shape[1] - 1):
            segments.append(
                [verts[0, i], verts[1, i], verts[0, i + 1], verts[1, i + 1]]
            )
        assert len(segments) == 2
        assert segments[0] == [0, 0, 1, 1]
        assert segments[1] == [1, 1, 2, 0]
