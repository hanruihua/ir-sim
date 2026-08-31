"""The vectorized RVO candidate evaluation must match the original loops exactly."""

from math import sqrt

import numpy as np
import pytest

from irsim.lib.algorithm.rvo import reciprocal_vel_obs
from irsim.util.util import dist_hypot


def _loop_candidates(obj, rvo_list):
    """Reference implementation: the original nested loops."""
    vo_outside, vo_inside = [], []
    cur_vx, cur_vy = obj.state[2], obj.state[3]
    for new_vx in np.arange(
        max(cur_vx - obj.acce, -obj.vxmax), min(cur_vx + obj.acce, obj.vxmax), 0.05
    ):
        for new_vy in np.arange(
            max(cur_vy - obj.acce, -obj.vymax), min(cur_vy + obj.acce, obj.vymax), 0.05
        ):
            if obj.vo_out(new_vx, new_vy, rvo_list):
                vo_outside.append([new_vx, new_vy])
            else:
                vo_inside.append([new_vx, new_vy])
    return vo_outside, vo_inside


def _agent(rng):
    obj = reciprocal_vel_obs.__new__(reciprocal_vel_obs)
    obj.state = [0.0, 0.0, *rng.uniform(-1, 1, 2), 0.3, *rng.uniform(-1.5, 1.5, 2)]
    obj.acce, obj.vxmax, obj.vymax, obj.factor = 1.0, 1.5, 1.5, 1.0
    return obj


def _cones(rng, n):
    cones = []
    for _ in range(n):
        apex = rng.uniform(-1, 1, 2)
        theta, half = rng.uniform(-np.pi, np.pi), rng.uniform(0.1, 1.2)
        left = [np.cos(theta + half), np.sin(theta + half)]
        right = [np.cos(theta - half), np.sin(theta - half)]
        cones.append([apex.tolist(), left, right])
    return cones


@pytest.mark.parametrize("seed", range(5))
def test_vectorized_candidates_match_loops(seed):
    rng = np.random.default_rng(seed)
    obj, cones = _agent(rng), _cones(rng, rng.integers(1, 6))

    outside, inside = obj.vel_candidate(cones)
    ref_outside, ref_inside = _loop_candidates(obj, cones)

    assert outside == ref_outside
    assert inside == ref_inside
    assert len(outside) + len(inside) > 0


@pytest.mark.parametrize("seed", range(5))
def test_vectorized_selection_matches_min(seed):
    rng = np.random.default_rng(seed)
    obj, cones = _agent(rng), _cones(rng, 3)
    outside, inside = obj.vel_candidate(cones)
    if not outside:
        pytest.skip("no feasible velocity for this seed")
    vel_des = [obj.state[5], obj.state[6]]

    expected = min(
        outside, key=lambda v: dist_hypot(v[0], v[1], vel_des[0], vel_des[1])
    )
    assert obj.vel_select(outside, inside) == expected


# The 2.10.2 scalar implementation, kept verbatim as the regression oracle.
def _scalar_penalty(obj, vel, vel_des, factor):
    """Compute fallback cost from desired-velocity error and collision time."""
    tc_list = []

    for moving in obj.obs_state_list:
        distance = dist_hypot(moving[0], moving[1], obj.state[0], obj.state[1])
        diff = distance**2 - (obj.state[4] + moving[4]) ** 2

        if diff < 0:
            diff = 0

        dis_vel = np.sqrt(diff)
        vel_trans = [
            2 * vel[0] - obj.state[2] - moving[2],
            2 * vel[1] - obj.state[3] - moving[3],
        ]
        vel_trans_speed = np.sqrt(vel_trans[0] ** 2 + vel_trans[1] ** 2) + 1e-7

        tc = dis_vel / vel_trans_speed

        tc_list.append(tc)

    # Time-to-collision with line segments
    x = obj.state[0]
    y = obj.state[1]
    r = obj.state[4]

    for seg in obj.line_obs_list:
        tc = _scalar_tc_line_segment(x, y, r, vel, seg)
        tc_list.append(tc)

    if not tc_list:
        return dist_hypot(vel_des[0], vel_des[1], vel[0], vel[1])

    tc_min = min(tc_list)

    if tc_min == 0:
        tc_min = 0.0001

    return factor * (1 / tc_min) + dist_hypot(vel_des[0], vel_des[1], vel[0], vel[1])


def _scalar_tc_line_segment(x, y, r, vel, seg):
    """Compute time-to-collision between agent and a line segment.

    Uses ray-segment intersection with the segment expanded by agent radius.
    """
    x1, y1, x2, y2 = seg
    # Segment direction and normal
    dx_seg = x2 - x1
    dy_seg = y2 - y1
    seg_len = sqrt(dx_seg * dx_seg + dy_seg * dy_seg)

    if seg_len < 1e-12:
        return 1e6

    # Outward normal (toward agent side)
    nx = -dy_seg / seg_len
    ny = dx_seg / seg_len

    # Make sure normal points toward agent
    if nx * (x - x1) + ny * (y - y1) < 0:
        nx, ny = -nx, -ny

    # Perpendicular distance from agent to segment line
    perp_dist = nx * (x - x1) + ny * (y - y1)

    # Velocity component toward the segment
    vel_toward = -(nx * vel[0] + ny * vel[1])

    if vel_toward <= 1e-7:
        # Moving away or parallel; no collision.
        return 1e6

    # Time to reach the expanded segment (offset by radius)
    tc = (perp_dist - r) / vel_toward

    if tc < 0:
        tc = 0

    # Check if the collision point is within the segment bounds
    # Project collision point onto segment axis
    col_x = x + vel[0] * tc
    col_y = y + vel[1] * tc
    t_proj = ((col_x - x1) * dx_seg + (col_y - y1) * dy_seg) / (seg_len * seg_len)

    # Allow some margin beyond endpoints for the agent radius
    margin = r / seg_len
    if t_proj < -margin or t_proj > 1 + margin:
        return 1e6

    return tc


@pytest.mark.parametrize("seed", range(6))
def test_vectorized_penalties_match_scalar(seed):
    rng = np.random.default_rng(seed)
    obj = _agent(rng)
    obj.obs_state_list = [
        [*rng.uniform(-3, 3, 2), *rng.uniform(-1, 1, 2), 0.3]
        for _ in range(rng.integers(0, 5))
    ]
    obj.line_obs_list = [
        rng.uniform(-3, 3, 4).tolist() for _ in range(rng.integers(0, 3))
    ]
    vels = rng.uniform(-1.5, 1.5, (40, 2))
    vel_des = [obj.state[5], obj.state[6]]

    expected = [_scalar_penalty(obj, v.tolist(), vel_des, obj.factor) for v in vels]
    np.testing.assert_allclose(
        obj.penalties(vels, vel_des, obj.factor), expected, rtol=1e-12, atol=0
    )
