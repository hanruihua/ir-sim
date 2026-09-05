"""Tests for social_force_model_vec — the NumPy-vectorized SFM implementation.

Strategy
--------
* Every test runs the SAME inputs through both the scalar ``social_force_model``
  and the vectorized ``social_force_model_vec`` and asserts the outputs agree
  to ``ATOL=1e-10`` (well inside numerical noise for double-precision trig).
* All edge-case branches that exist in the scalar version are exercised so any
  divergence in the vectorized logic is caught before the scalar code is touched.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from irsim.lib.algorithm.social_force_model import (
    social_force_model,
    social_force_model_vec,
)

ATOL = 1e-10

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _state(x=0.0, y=0.0, vx=0.0, vy=0.0, r=0.3, vx_des=1.0, vy_des=0.0, theta=0.0):
    return [x, y, vx, vy, r, vx_des, vy_des, theta]


def _both(state, **kw):
    """Return (scalar_sfm, vec_sfm) built from the same arguments."""
    return (
        social_force_model(state=state, **kw),
        social_force_model_vec(state=state, **kw),
    )


def _close(a: list, b: list) -> bool:
    return np.allclose(a, b, atol=ATOL)


# ---------------------------------------------------------------------------
# desired_force  (inherited — just a sanity check that vec inherits it intact)
# ---------------------------------------------------------------------------


class TestVecDesiredForceInherited:
    def test_same_as_scalar(self):
        s, v = _both(_state(vx=0.5, vx_des=1.2, vy_des=0.3))
        assert _close(s.desired_force(), v.desired_force())


# ---------------------------------------------------------------------------
# social_force — equivalence with scalar, all branches
# ---------------------------------------------------------------------------


class TestVecSocialForceEquivalence:
    def test_empty_neighbors(self):
        s, v = _both(_state(), neighbor_list=[])
        assert _close(s.social_force(), v.social_force())
        assert v.social_force() == [0.0, 0.0]

    def test_single_neighbor_in_front(self):
        nb = [[1.0, 0.0, 0.0, 0.0, 0.3]]
        s, v = _both(_state(vx=1.0, vx_des=1.0), neighbor_list=nb)
        assert _close(s.social_force(), v.social_force())

    def test_single_neighbor_behind(self):
        nb = [[-1.5, 0.0, 0.0, 0.0, 0.3]]
        s, v = _both(_state(vx=1.0, vx_des=1.0), neighbor_list=nb)
        assert _close(s.social_force(), v.social_force())

    def test_neighbor_out_of_range_skipped(self):
        """Both implementations must return zero for a far-away neighbor."""
        nb = [[100.0, 0.0, 0.0, 0.0, 0.3]]
        s, v = _both(_state(vx=1.0), neighbor_list=nb, neighbor_range=10.0)
        assert _close(s.social_force(), v.social_force())
        assert v.social_force() == [0.0, 0.0]

    def test_collocated_neighbor_skipped(self):
        """dist < 1e-6 branch — both must skip and return zero."""
        nb = [[0.0, 0.0, 0.0, 0.0, 0.3]]
        s, v = _both(_state(x=0.0, y=0.0), neighbor_list=nb)
        assert _close(s.social_force(), v.social_force())
        assert v.social_force() == [0.0, 0.0]

    def test_zero_interaction_vector_skipped(self):
        """t_norm < 1e-9 branch: ``t = lambda * dv + d_hat`` collapses to zero."""
        # With state vx=-0.5, neighbor at (1,0) vx=vy=0, lambda=2:
        #   dv = (-0.5, 0), d_hat = (1, 0)
        #   t  = 2*(-0.5,0) + (1,0) = (0, 0)
        nb = [[1.0, 0.0, 0.0, 0.0, 0.3]]
        s, v = _both(
            _state(x=0.0, y=0.0, vx=-0.5, vy=0.0),
            neighbor_list=nb,
            lambda_importance=2.0,
        )
        assert _close(s.social_force(), v.social_force())
        assert v.social_force() == [0.0, 0.0]

    def test_multiple_neighbors_mixed(self):
        """Three neighbors: one in range, one collocated, one out of range."""
        nb = [
            [1.5, 0.0, 0.0, 0.0, 0.3],  # in range
            [0.0, 0.0, 0.0, 0.0, 0.3],  # collocated
            [50.0, 0.0, 0.0, 0.0, 0.3],  # out of range
        ]
        s, v = _both(_state(vx=1.0), neighbor_list=nb, neighbor_range=10.0)
        assert _close(s.social_force(), v.social_force())

    @pytest.mark.parametrize("seed", [0, 1, 7, 42, 99])
    def test_random_large_crowd(self, seed):
        """N=50 neighbors with random positions and velocities."""
        rng = np.random.default_rng(seed)
        nb = rng.uniform(-5.0, 5.0, (50, 5)).tolist()
        s, v = _both(
            _state(x=0.0, y=0.0, vx=0.5, vy=0.2),
            neighbor_list=nb,
            neighbor_range=10.0,
            gamma=0.35,
            n_velocity=3.0,
            n_angular=2.0,
            lambda_importance=2.0,
            safety_radius=0.1,
        )
        assert _close(s.social_force(), v.social_force()), (
            f"seed={seed}: scalar={s.social_force()} vec={v.social_force()}"
        )

    @pytest.mark.parametrize("n_nb", [1, 5, 20, 100, 200])
    def test_various_crowd_sizes(self, n_nb):
        """Force sums match across many crowd densities."""
        rng = np.random.default_rng(n_nb)
        nb = rng.uniform(-3.0, 3.0, (n_nb, 5)).tolist()
        s, v = _both(_state(vx=0.8, vy=0.0), neighbor_list=nb, neighbor_range=8.0)
        assert _close(s.social_force(), v.social_force())

    def test_anisotropy_preserved(self):
        """Neighbor in front must produce a larger-magnitude force than one behind."""
        state = _state(vx=1.0, vx_des=1.0)
        _, v = _both(state, neighbor_list=[[1.0, 0.0, 0.0, 0.0, 0.3]])
        _, v_back = _both(state, neighbor_list=[[-1.0, 0.0, 0.0, 0.0, 0.3]])
        front_mag = math.hypot(*v.social_force())
        behind_mag = math.hypot(*v_back.social_force())
        assert front_mag > behind_mag


# ---------------------------------------------------------------------------
# obstacle_force — equivalence with scalar, all branches
# ---------------------------------------------------------------------------


class TestVecObstacleForceEquivalence:
    def test_no_obstacles(self):
        s, v = _both(_state(), line_obs_list=[])
        assert _close(s.obstacle_force(), v.obstacle_force())
        assert v.obstacle_force() == [0.0, 0.0]

    def test_single_wall_push_negative_y(self):
        """Wall at y=+1, agent at origin -> force must point in -y direction."""
        seg = [[-1.0, 1.0, 1.0, 1.0]]
        s, v = _both(_state(x=0.0, y=0.0), line_obs_list=seg)
        assert _close(s.obstacle_force(), v.obstacle_force())
        assert v.obstacle_force()[1] < 0.0

    def test_out_of_range_segment_zero(self):
        """Segment well beyond 5*sigma_obstacle must contribute nothing."""
        seg = [[-1.0, 100.0, 1.0, 100.0]]
        s, v = _both(_state(), line_obs_list=seg, sigma_obstacle=0.5)
        assert _close(s.obstacle_force(), v.obstacle_force())
        assert v.obstacle_force() == [0.0, 0.0]

    def test_overlap_branch_fires(self):
        """Closest point on segment coincides with agent → scalar adds 1.0 to fx."""
        seg = [[-1.0, 0.0, 1.0, 0.0]]
        s, v = _both(_state(x=0.0, y=0.0, r=0.3), line_obs_list=seg)
        sf, vf = s.obstacle_force(), v.obstacle_force()
        assert _close(sf, vf)
        assert vf[0] == pytest.approx(1.0)
        assert vf[1] == pytest.approx(0.0)

    def test_multiple_overlapping_segments(self):
        """Two segments both centred on agent → fx == 2.0, fy == 0.0."""
        segs = [
            [-1.0, 0.0, 1.0, 0.0],  # horizontal, passes through origin
            [0.0, -1.0, 0.0, 1.0],  # vertical, passes through origin
        ]
        s, v = _both(_state(x=0.0, y=0.0, r=0.3), line_obs_list=segs)
        sf, vf = s.obstacle_force(), v.obstacle_force()
        assert _close(sf, vf)
        assert vf[0] == pytest.approx(2.0)
        assert vf[1] == pytest.approx(0.0)

    def test_closer_wall_larger_force(self):
        """Force magnitude grows as the agent approaches the wall."""
        s_near, v_near = _both(_state(), line_obs_list=[[-1, 0.5, 1, 0.5]])
        s_far, v_far = _both(_state(), line_obs_list=[[-1, 2.0, 1, 2.0]])
        mag_near = math.hypot(*v_near.obstacle_force())
        mag_far = math.hypot(*v_far.obstacle_force())
        assert mag_near > mag_far
        # And both agree with scalar
        assert _close(s_near.obstacle_force(), v_near.obstacle_force())
        assert _close(s_far.obstacle_force(), v_far.obstacle_force())

    def test_degenerate_segment_endpoint(self):
        """Zero-length segment (both endpoints coincide) must not crash."""
        seg = [[2.0, 3.0, 2.0, 3.0]]  # degenerate
        s, v = _both(_state(x=0.0, y=0.0, r=0.1), line_obs_list=seg)
        assert _close(s.obstacle_force(), v.obstacle_force())

    def test_mixed_in_range_and_out_of_range(self):
        """One near segment + one far segment: only near contributes."""
        segs = [
            [-1.0, 0.5, 1.0, 0.5],  # near — in range
            [-1.0, 200.0, 1.0, 200.0],  # far — out of range
        ]
        s, v = _both(_state(), line_obs_list=segs)
        assert _close(s.obstacle_force(), v.obstacle_force())

    @pytest.mark.parametrize("seed", [0, 3, 17, 42, 123])
    def test_random_segments(self, seed):
        """Random segment collections match scalar output."""
        rng = np.random.default_rng(seed)
        n = int(rng.integers(5, 40))
        # Generate segments as random short walls inside [-5, 5]
        centers = rng.uniform(-4.0, 4.0, (n, 2))
        angles = rng.uniform(0, math.pi, n)
        lengths = rng.uniform(0.1, 2.0, n)
        hv = (lengths[:, None] / 2) * np.column_stack([np.cos(angles), np.sin(angles)])
        segs = np.column_stack([centers - hv, centers + hv]).tolist()
        s, v = _both(
            _state(x=0.0, y=0.0, r=0.3),
            line_obs_list=segs,
            sigma_obstacle=0.8,
        )
        assert _close(s.obstacle_force(), v.obstacle_force()), (
            f"seed={seed}: scalar={s.obstacle_force()} vec={v.obstacle_force()}"
        )

    def test_overlap_and_nonoverlap_mixed(self):
        """One overlapping segment + one normal segment — sums must match."""
        # Segment 0: passes through origin → overlap
        # Segment 1: wall at y=1.5 → normal
        segs = [
            [-1.0, 0.0, 1.0, 0.0],
            [-1.0, 1.5, 1.0, 1.5],
        ]
        s, v = _both(_state(x=0.0, y=0.0, r=0.3), line_obs_list=segs)
        assert _close(s.obstacle_force(), v.obstacle_force())


# ---------------------------------------------------------------------------
# cal_vel end-to-end — vec must produce the same trajectory
# ---------------------------------------------------------------------------


class TestVecCalVelEquivalence:
    def test_cal_vel_matches_scalar(self):
        nb = [[1.5, 0.2, 0.1, -0.1, 0.3], [0.8, -0.5, -0.2, 0.0, 0.25]]
        segs = [[-2.0, 2.0, 2.0, 2.0]]
        s, v = _both(
            _state(x=0.0, y=0.0, vx=0.6, vy=0.1, vx_des=1.0, vy_des=0.0),
            neighbor_list=nb,
            line_obs_list=segs,
            vmax=1.5,
            step_time=0.1,
        )
        assert _close(s.cal_vel(), v.cal_vel())

    def test_vmax_clipping_still_applies(self):
        """vec must clip the output to vmax just like scalar does."""
        s, v = _both(
            _state(vx=0.0, vx_des=20.0),  # huge desired → integrates past vmax
            vmax=1.0,
            step_time=1.0,
        )
        vx, vy = v.cal_vel()
        speed = math.hypot(vx, vy)
        assert speed <= 1.0 + 1e-9
        # And it agrees with scalar
        assert _close([vx, vy], s.cal_vel())

    def test_update_then_cal_vel(self):
        """update() followed by cal_vel() must give same output from both."""
        s = social_force_model(state=_state(x=0.0))
        v = social_force_model_vec(state=_state(x=0.0))
        new_state = _state(x=2.0, vx=0.5)
        nb = [[3.0, 0.0, 0.0, 0.0, 0.3]]
        s.update(new_state, nb)
        v.update(new_state, nb)
        assert _close(s.cal_vel(), v.cal_vel())

    @pytest.mark.parametrize("seed", range(10))
    def test_random_end_to_end(self, seed):
        rng = np.random.default_rng(seed)
        nb = rng.uniform(-3.0, 3.0, (rng.integers(0, 30), 5)).tolist()
        n_segs = int(rng.integers(0, 15))
        segs = []
        for _ in range(n_segs):
            c = rng.uniform(-2.0, 2.0, 2)
            ang = rng.uniform(0, math.pi)
            L = rng.uniform(0.2, 1.5)
            hv = L / 2 * np.array([math.cos(ang), math.sin(ang)])
            segs.append([*(c - hv), *(c + hv)])
        state = _state(
            x=float(rng.uniform(-1, 1)),
            y=float(rng.uniform(-1, 1)),
            vx=float(rng.uniform(-0.5, 0.5)),
            vy=float(rng.uniform(-0.5, 0.5)),
            vx_des=float(rng.uniform(0.3, 1.0)),
        )
        s = social_force_model(state=state, neighbor_list=nb, line_obs_list=segs)
        v = social_force_model_vec(state=state, neighbor_list=nb, line_obs_list=segs)
        assert _close(s.cal_vel(), v.cal_vel()), (
            f"seed={seed}: scalar={s.cal_vel()} vec={v.cal_vel()}"
        )


# ---------------------------------------------------------------------------
# Constructor / error handling  (inherited from scalar — smoke test)
# ---------------------------------------------------------------------------


class TestVecConstructor:
    def test_invalid_relaxation_time(self):
        with pytest.raises(ValueError, match="relaxation_time must be > 0"):
            social_force_model_vec(state=_state(), relaxation_time=0.0)

    def test_invalid_gamma(self):
        with pytest.raises(ValueError, match="gamma must be > 0"):
            social_force_model_vec(state=_state(), gamma=-1.0)

    def test_invalid_sigma_obstacle(self):
        with pytest.raises(ValueError, match="sigma_obstacle must be > 0"):
            social_force_model_vec(state=_state(), sigma_obstacle=0.0)
