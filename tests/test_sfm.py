"""Tests for the Social Force Model behavior.

Covers:
- social_force_model algorithm: desired/social/obstacle forces, vmax clipping.
- SocialForceModelBatch: parity with the scalar solver, social-group forces.
- ("diff", "sfm") and ("omni", "sfm") behavior registration, per-object and group.
- SfmGroupBehavior: outside objects, kinematics mapping, role filtering.
- End-to-end env step with `behavior: sfm` and `group_behavior: sfm` via a temp YAML.
"""

import contextlib
import importlib
import os
import tempfile
from unittest.mock import Mock

import numpy as np
import pytest
import yaml

import irsim
from irsim.lib.algorithm.social_force_model import (
    SocialForceModelBatch,
    social_force_model,
)
from irsim.lib.behavior.behavior_registry import (
    behaviors_map,
    group_behaviors_class_map,
)
from irsim.lib.behavior.group_behavior_methods import SfmGroupBehavior
from irsim.world.object_base import ObjectBase

# Trigger the @register_behavior decorators in behavior_methods.py.
importlib.import_module("irsim.lib.behavior.behavior_methods")


# ---------------------------------------------------------------------------
# Helper: agent state matches rvo_state convention
#   [x, y, vx, vy, radius, vx_des, vy_des, theta]
# ---------------------------------------------------------------------------
def _state(x=0.0, y=0.0, vx=0.0, vy=0.0, r=0.3, vx_des=1.0, vy_des=0.0, theta=0.0):
    return [x, y, vx, vy, r, vx_des, vy_des, theta]


# ===================================================================
# social_force_model algorithm
# ===================================================================


class TestSFMDesiredForce:
    def test_pulls_toward_desired_velocity(self):
        sfm = social_force_model(state=_state(vx=0.0, vx_des=1.0))
        fx, fy = sfm.desired_force()
        assert fx > 0.0
        assert fy == 0.0

    def test_zero_when_at_desired_velocity(self):
        sfm = social_force_model(state=_state(vx=1.0, vx_des=1.0))
        fx, fy = sfm.desired_force()
        assert fx == 0.0
        assert fy == 0.0

    def test_relaxation_time_scales_force(self):
        s = _state(vx=0.0, vx_des=1.0)
        f_short = social_force_model(state=s, relaxation_time=0.25).desired_force()
        f_long = social_force_model(state=s, relaxation_time=1.0).desired_force()
        assert f_short[0] > f_long[0] > 0.0


class TestSFMSocialForce:
    def test_empty_neighbors_zero_force(self):
        sfm = social_force_model(state=_state(), neighbor_list=[])
        assert sfm.social_force() == [0.0, 0.0]

    def test_neighbor_in_front_pushes_back(self):
        """Neighbor directly ahead should produce a force with negative x component."""
        sfm = social_force_model(
            state=_state(x=0.0, y=0.0, vx=1.0, vx_des=1.0),
            neighbor_list=[[1.0, 0.0, 0.0, 0.0, 0.3]],
        )
        fx, _ = sfm.social_force()
        assert fx < 0.0

    def test_neighbor_far_outside_range_no_force(self):
        sfm = social_force_model(
            state=_state(vx=1.0, vx_des=1.0),
            neighbor_list=[[50.0, 0.0, 0.0, 0.0, 0.3]],
            neighbor_range=10.0,
        )
        assert sfm.social_force() == [0.0, 0.0]

    def test_anisotropy_neighbor_behind_weaker_than_front(self):
        """Same distance, different angle: neighbor in front matters more."""
        state = _state(vx=1.0, vx_des=1.0)
        front = social_force_model(
            state=state, neighbor_list=[[1.0, 0.0, 0.0, 0.0, 0.3]]
        ).social_force()
        behind = social_force_model(
            state=state, neighbor_list=[[-1.0, 0.0, 0.0, 0.0, 0.3]]
        ).social_force()
        front_mag = (front[0] ** 2 + front[1] ** 2) ** 0.5
        behind_mag = (behind[0] ** 2 + behind[1] ** 2) ** 0.5
        assert front_mag > behind_mag


class TestSFMObstacleForce:
    def test_no_obstacles_zero_force(self):
        sfm = social_force_model(state=_state(), line_obs_list=[])
        assert sfm.obstacle_force() == [0.0, 0.0]

    def test_closer_wall_larger_force(self):
        s = _state(x=0.0, y=0.0)
        near = social_force_model(state=s, line_obs_list=[[-1, 0.5, 1, 0.5]])
        far = social_force_model(state=s, line_obs_list=[[-1, 2.0, 1, 2.0]])
        near_mag = sum(c**2 for c in near.obstacle_force()) ** 0.5
        far_mag = sum(c**2 for c in far.obstacle_force()) ** 0.5
        assert near_mag > far_mag

    def test_force_points_away_from_wall(self):
        # wall at y=+1, agent at origin → force should point in -y
        sfm = social_force_model(
            state=_state(x=0.0, y=0.0), line_obs_list=[[-1, 1.0, 1, 1.0]]
        )
        _, fy = sfm.obstacle_force()
        assert fy < 0.0


class TestSFMIntegration:
    def test_cal_vel_clipped_to_vmax(self):
        sfm = social_force_model(
            state=_state(vx=0.0, vx_des=10.0),  # huge desired velocity
            vmax=1.0,
            step_time=1.0,
        )
        vx, vy = sfm.cal_vel()
        speed = (vx * vx + vy * vy) ** 0.5
        assert speed <= 1.0 + 1e-9

    def test_update_refreshes_state(self):
        sfm = social_force_model(state=_state(x=0.0))
        sfm.update(_state(x=5.0), neighbor_list=[[1, 1, 0, 0, 0.3]])
        assert sfm.state[0] == 5.0
        assert len(sfm.neighbor_list) == 1


# ===================================================================
# SocialForceModelBatch: parity with the scalar solver
# ===================================================================


def _random_scene(seed=0, n=10, m=4, k=5):
    """Random members, outside neighbours and walls in a 6 m box."""
    rng = np.random.default_rng(seed)
    states = np.column_stack(
        [
            rng.uniform(-3, 3, n),
            rng.uniform(-3, 3, n),
            rng.uniform(-1, 1, n),
            rng.uniform(-1, 1, n),
            rng.uniform(0.2, 0.4, n),
            rng.uniform(-1, 1, n),
            rng.uniform(-1, 1, n),
            rng.uniform(-3, 3, n),
        ]
    )
    neighbors = np.column_stack(
        [
            rng.uniform(-3, 3, m),
            rng.uniform(-3, 3, m),
            rng.uniform(-1, 1, m),
            rng.uniform(-1, 1, m),
            rng.uniform(0.2, 0.5, m),
        ]
    )
    segments = rng.uniform(-4, 4, (k, 4))
    return states, neighbors, segments


_PARITY_PARAMS = {
    "vmax": 1.2,
    "step_time": 0.1,
    "relaxation_time": 0.4,
    "lambda_importance": 1.3,
    "gamma": 0.5,
    "n_angular": 1.5,
    "n_velocity": 2.5,
    "sigma_obstacle": 0.6,
    "safety_radius": 0.1,
    "force_factor_desired": 0.9,
    "force_factor_social": 4.0,
    "force_factor_obstacle": 6.0,
}


class TestSFMBatchMatchesScalar:
    def _scalar_for(self, i, states, neighbors, segments, neighbor_range):
        """Scalar solver for member ``i`` fed exactly what the per-object
        behavior feeds it: other members and outside neighbours within
        ``neighbor_range``."""
        x, y = states[i, 0], states[i, 1]
        others = [s[:5].tolist() for j, s in enumerate(states) if j != i]
        others += neighbors.tolist()
        others = [
            o for o in others if (x - o[0]) ** 2 + (y - o[1]) ** 2 < neighbor_range**2
        ]
        return social_force_model(
            states[i].tolist(),
            others,
            segments.tolist(),
            neighbor_range=neighbor_range,
            **_PARITY_PARAMS,
        )

    @pytest.mark.parametrize("seed", [0, 1, 2])
    def test_every_force_and_velocity_match(self, seed):
        states, neighbors, segments = _random_scene(seed)
        # A point segment and a wall through member 0's centre exercise the
        # degenerate-segment and overlap branches in both solvers.
        segments = np.vstack(
            [
                segments,
                [1.0, 1.0, 1.0, 1.0],
                [states[0, 0] - 1, states[0, 1], states[0, 0] + 1, states[0, 1]],
            ]
        )
        neighbor_range = 4.0
        batch = SocialForceModelBatch(
            states, neighbors, segments, neighbor_range=neighbor_range, **_PARITY_PARAMS
        )
        fd, fs, fo, vel = (
            batch.desired_force(),
            batch.social_force(),
            batch.obstacle_force(),
            batch.cal_vel(),
        )
        assert vel.shape == (len(states), 2)
        for i in range(len(states)):
            scalar = self._scalar_for(i, states, neighbors, segments, neighbor_range)
            np.testing.assert_allclose(fd[i], scalar.desired_force(), atol=1e-12)
            np.testing.assert_allclose(fs[i], scalar.social_force(), atol=1e-12)
            np.testing.assert_allclose(fo[i], scalar.obstacle_force(), atol=1e-12)
            np.testing.assert_allclose(vel[i], scalar.cal_vel(), atol=1e-12)

    def test_outside_neighbors_push_but_are_not_moved(self):
        # One member walking +x, one outside neighbour straight ahead.
        states = np.array([[0.0, 0.0, 1.0, 0.0, 0.3, 1.0, 0.0, 0.0]])
        batch = SocialForceModelBatch(
            states, neighbor_states=[[1.0, 0.0, 0.0, 0.0, 0.3]]
        )
        fs = batch.social_force()
        assert fs.shape == (1, 2)
        assert fs[0, 0] < 0.0
        assert batch.cal_vel().shape == (1, 2)

    def test_self_interaction_and_range_are_excluded(self):
        states = np.array(
            [
                [0.0, 0.0, 1.0, 0.0, 0.3, 1.0, 0.0, 0.0],
                [50.0, 0.0, -1.0, 0.0, 0.3, -1.0, 0.0, 0.0],
            ]
        )
        batch = SocialForceModelBatch(states, neighbor_range=10.0)
        np.testing.assert_array_equal(batch.social_force(), np.zeros((2, 2)))

    def test_vmax_clipping(self):
        states = np.array([[0.0, 0.0, 0.0, 0.0, 0.3, 10.0, 0.0, 0.0]])
        vel = SocialForceModelBatch(states, vmax=1.0, step_time=1.0).cal_vel()
        assert np.hypot(*vel[0]) <= 1.0 + 1e-9

    def test_empty_batch(self):
        batch = SocialForceModelBatch(
            np.zeros((0, 8)), [[1, 1, 0, 0, 0.3]], [[0, 0, 1, 1]]
        )
        assert batch.cal_vel().shape == (0, 2)
        assert batch.social_force().shape == (0, 2)
        assert batch.obstacle_force().shape == (0, 2)

    def test_non_positive_divisors_rejected(self):
        states = np.zeros((1, 8))
        for kwargs in (
            {"relaxation_time": 0.0},
            {"gamma": -0.5},
            {"sigma_obstacle": 0.0},
            {"group_repulsion_threshold": -0.1},
            {"gaze_vision_angle": 0.0},
            {"gaze_vision_angle": 181.0},
        ):
            param = next(iter(kwargs))
            with pytest.raises(ValueError, match=f"^{param} must be"):
                SocialForceModelBatch(states, **kwargs)


# ===================================================================
# SocialForceModelBatch: social-group forces (Moussaid et al. 2010)
# ===================================================================


def _row(x, y, vx=0.0, vy=0.0, r=0.3):
    return [x, y, vx, vy, r, vx, vy, 0.0]


class TestSFMBatchGroupForces:
    def test_coherence_pulls_straggler_toward_centre(self):
        # Members at x = 0 and x = 3: the centre of mass is at 1.5, well
        # beyond the (2 - 1) / 2 = 0.5 m comfort distance, so both are
        # pulled toward it and the straggler feels the larger pull.
        batch = SocialForceModelBatch([_row(0, 0), _row(3, 0)], social_groups=[[0, 1]])
        f = batch.coherence_force()
        assert f[0, 0] > 0.0
        assert f[1, 0] < 0.0
        assert f[0, 1] == f[1, 1] == 0.0

    def test_coherence_fades_when_members_are_close(self):
        near = SocialForceModelBatch([_row(0, 0), _row(0.2, 0)], social_groups=[[0, 1]])
        far = SocialForceModelBatch([_row(0, 0), _row(3.0, 0)], social_groups=[[0, 1]])
        assert np.abs(near.coherence_force()).max() < 0.05
        assert np.abs(far.coherence_force()).max() > 1.0

    def test_repulsion_only_inside_threshold_and_points_apart(self):
        close = SocialForceModelBatch(
            [_row(0, 0), _row(0.2, 0)],
            social_groups=[[0, 1]],
            group_repulsion_threshold=0.55,
        )
        f = close.group_repulsive_force()
        assert f[0, 0] < 0.0
        assert f[1, 0] > 0.0
        np.testing.assert_allclose(f[0], -f[1])

        apart = SocialForceModelBatch(
            [_row(0, 0), _row(1.0, 0)],
            social_groups=[[0, 1]],
            group_repulsion_threshold=0.55,
        )
        np.testing.assert_array_equal(apart.group_repulsive_force(), np.zeros((2, 2)))

    def test_repulsion_default_threshold_is_sum_of_radii(self):
        # radii 0.3 + 0.3: discs touch at 0.6 m centre distance
        touching = SocialForceModelBatch(
            [_row(0, 0, r=0.3), _row(0.55, 0, r=0.3)], social_groups=[[0, 1]]
        )
        assert touching.group_repulsive_force()[0, 0] < 0.0
        clear = SocialForceModelBatch(
            [_row(0, 0, r=0.3), _row(0.65, 0, r=0.3)], social_groups=[[0, 1]]
        )
        np.testing.assert_array_equal(clear.group_repulsive_force(), np.zeros((2, 2)))

    def test_gaze_brakes_the_member_ahead_only(self):
        # All walk +x; member 1 is 3 m ahead so the others' centre of mass
        # sits directly behind it (180 deg > 90 deg vision): it is braked by
        # (pi - pi/2) * v. Members 0 and 2 see the centre ahead: no force.
        batch = SocialForceModelBatch(
            [_row(0, 0, vx=1.0), _row(3, 0, vx=1.0), _row(0.2, 0, vx=1.0)],
            social_groups=[[0, 1, 2]],
        )
        f = batch.gaze_force()
        np.testing.assert_allclose(f[1], [-np.pi / 2, 0.0], atol=1e-9)
        np.testing.assert_array_equal(f[0], [0.0, 0.0])
        np.testing.assert_array_equal(f[2], [0.0, 0.0])

    def test_gaze_partial_rotation_scales_with_speed(self):
        # Partner directly to the side and slightly behind: the centre of
        # mass is a few degrees past 90, so the brake is small and grows
        # linearly with the walking speed.
        slow = SocialForceModelBatch(
            [_row(0, 0, vx=0.5), _row(-0.1, 0.6, vx=0.5)], social_groups=[[0, 1]]
        ).gaze_force()
        fast = SocialForceModelBatch(
            [_row(0, 0, vx=1.0), _row(-0.1, 0.6, vx=1.0)], social_groups=[[0, 1]]
        ).gaze_force()
        assert slow[0, 0] < 0.0
        assert fast[0, 0] == pytest.approx(2.0 * slow[0, 0])
        assert np.abs(slow[0, 0]) < np.pi / 2 * 0.5

    def test_gaze_zero_when_stationary(self):
        batch = SocialForceModelBatch([_row(0, 0), _row(3, 0)], social_groups=[[0, 1]])
        np.testing.assert_array_equal(batch.gaze_force(), np.zeros((2, 2)))

    def test_group_forces_enter_cal_vel_only_with_groups(self):
        rows = [_row(0, 0, vx=1.0), _row(3, 0, vx=1.0)]
        alone = SocialForceModelBatch(rows).cal_vel()
        grouped = SocialForceModelBatch(rows, social_groups=[[0, 1]]).cal_vel()
        assert not np.allclose(alone, grouped)
        # coherence pulls the straggler forward and the leader back
        assert grouped[0, 0] > alone[0, 0]
        assert grouped[1, 0] < alone[1, 0]

    def test_singletons_and_empty_groups_are_ignored(self):
        batch = SocialForceModelBatch(
            [_row(0, 0), _row(3, 0)], social_groups=[[0], [1]]
        )
        assert batch.social_groups == []
        batch = SocialForceModelBatch([_row(0, 0), _row(3, 0)], social_groups=[])
        assert batch.social_groups == []

    def test_invalid_groups_raise(self):
        rows = [_row(0, 0), _row(3, 0), _row(6, 0)]
        with pytest.raises(ValueError, match="out of range"):
            SocialForceModelBatch(rows, social_groups=[[0, 5]])
        with pytest.raises(ValueError, match="out of range"):
            SocialForceModelBatch(rows, social_groups=[[-1, 0]])
        with pytest.raises(ValueError, match="more than one social group"):
            SocialForceModelBatch(rows, social_groups=[[0, 1], [1, 2]])
        with pytest.raises(ValueError, match="more than one social group"):
            SocialForceModelBatch(rows, social_groups=[[0, 0]])

    def test_update_keeps_groups_unless_replaced(self):
        rows = [_row(0, 0), _row(3, 0)]
        batch = SocialForceModelBatch(rows, social_groups=[[0, 1]])
        batch.update(rows)
        assert len(batch.social_groups) == 1
        batch.update(rows, social_groups=[])
        assert batch.social_groups == []
        # Shrinking the batch below a group index is rejected on update.
        batch.update(rows, social_groups=[[0, 1]])
        with pytest.raises(ValueError, match="out of range"):
            batch.update(rows[:1])


# ===================================================================
# SfmGroupBehavior (class-based group handler) with mock members
# ===================================================================


def _mock_member(
    kin="diff",
    x=0.0,
    y=0.0,
    theta=0.0,
    vx=0.0,
    vy=0.0,
    vx_des=1.0,
    vy_des=0.0,
    r=0.3,
    externals=None,
):
    m = Mock(spec=ObjectBase)
    m.kinematics = kin
    m.role = "robot"
    m.state = np.array([[x], [y], [theta]])
    m.rvo_state = [x, y, vx, vy, r, vx_des, vy_des, theta]
    m.rvo_neighbor_state = [x, y, vx, vy, r]
    m.rvo_line_segments = []
    m.external_objects = list(externals) if externals is not None else []
    m.get_vel_range = Mock(
        return_value=(np.array([[-1.5], [-3.0]]), np.array([[1.5], [3.0]]))
    )
    m._world_param = Mock()
    m._world_param.step_time = 0.1
    return m


def _mock_wall(segments, role="obstacle"):
    o = Mock(spec=ObjectBase)
    o.role = role
    o.rvo_line_segments = segments
    o.rvo_neighbor_state = [0.0, 0.0, 0.0, 0.0, 0.0]
    return o


def _mock_circle(x, y, vx=0.0, vy=0.0, r=0.3, role="robot"):
    o = Mock(spec=ObjectBase)
    o.role = role
    o.rvo_line_segments = []
    o.rvo_neighbor_state = [x, y, vx, vy, r]
    return o


class TestSfmGroupBehavior:
    def test_registered_for_omni_and_diff(self):
        # Other test modules may evict and re-import the methods module, so
        # look the class up on the module that is currently registered.
        module = importlib.import_module("irsim.lib.behavior.group_behavior_methods")
        assert ("omni", "sfm") in group_behaviors_class_map
        assert ("diff", "sfm") in group_behaviors_class_map
        handler = group_behaviors_class_map[("diff", "sfm")]([_mock_member()])
        assert isinstance(handler, module.SfmGroupBehavior)

    def test_diff_isolated_member_accelerates_forward(self):
        member = _mock_member("diff")
        out = SfmGroupBehavior([member], vmax=1.0)([member])
        assert len(out) == 1
        assert out[0].shape == (2, 1)
        # from rest: v = dt * (v_des - 0) / tau = 0.1 * 1.0 / 0.5
        assert out[0][0, 0] == pytest.approx(0.2, abs=1e-9)
        assert out[0][1, 0] == pytest.approx(0.0, abs=1e-9)

    def test_desired_velocity_renormalised_to_vmax(self):
        # rvo_state carries an anisotropically scaled desired velocity;
        # only its direction should survive, at speed ``vmax``.
        member = _mock_member("diff", vx_des=3.0, vy_des=0.0)
        beh = SfmGroupBehavior([member], vmax=1.0)
        states, _, _ = beh._collect_inputs([member])
        np.testing.assert_allclose(states[0, 5:7], [1.0, 0.0])

    def test_omni_output_is_in_body_frame(self):
        # Facing +y while the plan says +x world: body-frame lateral command.
        member = _mock_member("omni", theta=np.pi / 2)
        out = SfmGroupBehavior([member], vmax=1.0)([member])
        assert out[0].shape == (2, 1)
        assert out[0][0, 0] == pytest.approx(0.0, abs=1e-9)
        assert out[0][1, 0] == pytest.approx(-0.2, abs=1e-9)

    def test_unknown_kinematics_passes_world_velocity_through(self):
        member = _mock_member("custom")
        out = SfmGroupBehavior([member], vmax=1.0)([member])
        np.testing.assert_allclose(out[0], [[0.2], [0.0]], atol=1e-9)

    def test_outside_wall_and_robot_push_member(self):
        wall = _mock_wall([[-1.0, 0.5, 1.0, 0.5]])
        other = _mock_circle(1.0, 0.0)
        member = _mock_member("diff", vx=1.0, externals=[wall, other])
        beh = SfmGroupBehavior([member], vmax=2.0)
        states, circles, segments = beh._collect_inputs([member])
        assert circles == [other.rvo_neighbor_state]
        assert segments == [[-1.0, 0.5, 1.0, 0.5]]
        beh.sfm.update(states, circles, segments)
        assert beh.sfm.obstacle_force()[0, 1] < 0.0  # wall above pushes down
        assert beh.sfm.social_force()[0, 0] < 0.0  # robot ahead pushes back

    def test_members_are_not_double_counted_as_externals(self):
        a = _mock_member("diff", x=0.0)
        b = _mock_member("diff", x=1.0)
        a.external_objects = [b]
        b.external_objects = [a]
        beh = SfmGroupBehavior([a, b])
        _, circles, segments = beh._collect_inputs([a, b])
        assert circles == []
        assert segments == []

    def test_target_roles_filters_outside_objects(self):
        wall = _mock_wall([[-1.0, 0.5, 1.0, 0.5]], role="obstacle")
        other = _mock_circle(1.0, 0.0, role="robot")
        member = _mock_member("diff", externals=[wall, other])
        _, circles, segments = SfmGroupBehavior(
            [member], target_roles="robot"
        )._collect_inputs([member])
        assert circles == [other.rvo_neighbor_state]
        assert segments == []
        _, circles, segments = SfmGroupBehavior(
            [member], target_roles="obstacle"
        )._collect_inputs([member])
        assert circles == []
        assert segments == [[-1.0, 0.5, 1.0, 0.5]]

    def test_member_without_goal_stands_but_yields(self):
        # No goal -> zero desired velocity (rvo_state convention). Alone it
        # brakes to rest; with a neighbour bearing down it is pushed aside.
        alone = _mock_member("omni", vx=1.0, vx_des=0.0, vy_des=0.0)
        out = SfmGroupBehavior([alone], vmax=2.0)([alone])
        assert out[0][0, 0] < 1.0
        pushed = _mock_member(
            "omni",
            vx_des=0.0,
            vy_des=0.0,
            externals=[_mock_circle(0.6, 0.05, vx=-1.0)],
        )
        out = SfmGroupBehavior([pushed], vmax=2.0)([pushed])
        assert np.abs(out[0]).max() > 0.0

    def test_call_refreshes_kinematics_from_members(self):
        member = _mock_member("diff")
        beh = SfmGroupBehavior([member])
        beh._kinematics = "omni"
        out = beh([member])
        assert beh._kinematics == "diff"
        assert out[0].shape == (2, 1)

    def test_member_count_change_revalidates_groups(self):
        a, b = _mock_member("diff", x=0.0), _mock_member("diff", x=1.0)
        beh = SfmGroupBehavior([a, b], social_groups=[[0, 1]])
        assert len(beh([a, b])) == 2
        with pytest.raises(ValueError, match="out of range"):
            beh([a])

    def test_invalid_social_groups_raise_at_construction(self):
        with pytest.raises(ValueError, match="out of range"):
            SfmGroupBehavior([_mock_member()], social_groups=[[0, 1]])

    def test_call_with_no_members_returns_no_actions(self):
        beh = SfmGroupBehavior([_mock_member()])
        assert beh([]) == []


# ===================================================================
# Behavior registration
# ===================================================================


class TestSFMBehaviorRegistration:
    def test_diff_sfm_registered(self):
        assert ("diff", "sfm") in behaviors_map

    def test_omni_sfm_registered(self):
        assert ("omni", "sfm") in behaviors_map


# ===================================================================
# End-to-end via temp YAML
# ===================================================================


# Track every temp YAML written by ``_write_temp_yaml`` so an autouse
# fixture can remove them after each test — prevents files leaking
# across repeated test runs.
_TEMP_YAML_PATHS: list[str] = []


def _write_temp_yaml(cfg: dict) -> str:
    fd, path = tempfile.mkstemp(suffix=".yaml")
    try:
        with os.fdopen(fd, "w") as f:
            yaml.safe_dump(cfg, f)
    except Exception:
        os.unlink(path)
        raise
    _TEMP_YAML_PATHS.append(path)
    return path


@pytest.fixture(autouse=True)
def _cleanup_temp_yamls():
    """Remove every temp YAML this test produced, even if the test failed."""
    yield
    while _TEMP_YAML_PATHS:
        path = _TEMP_YAML_PATHS.pop()
        with contextlib.suppress(FileNotFoundError):
            os.unlink(path)


class TestSFMEndToEnd:
    def test_two_robots_head_on_pass(self):
        """Two diff robots heading toward each other both reach their goals."""
        cfg = {
            "world": {
                "height": 6,
                "width": 14,
                "step_time": 0.1,
                "sample_time": 0.1,
                "offset": [-7, -3],
                "collision_mode": "unobstructed",
                "control_mode": "auto",
            },
            "robot": [
                {
                    "kinematics": {"name": "diff"},
                    "shape": [{"name": "circle", "radius": 0.3}],
                    "state": [-6, 0, 0],
                    "goal": [6, 0, 0],
                    "behavior": {"name": "sfm", "vmax": 1.0},
                    "vel_min": [-1.5, -3],
                    "vel_max": [1.5, 3],
                    "arrive_mode": "position",
                    "goal_threshold": 0.2,
                },
                {
                    "kinematics": {"name": "diff"},
                    "shape": [{"name": "circle", "radius": 0.3}],
                    "state": [6, 0.1, 3.14],
                    "goal": [-6, 0.1, 3.14],
                    "behavior": {"name": "sfm", "vmax": 1.0},
                    "vel_min": [-1.5, -3],
                    "vel_max": [1.5, 3],
                    "arrive_mode": "position",
                    "goal_threshold": 0.2,
                },
            ],
        }
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        for _ in range(300):
            env.step()
            if env.done():
                break
        assert env.done(), "both robots should have arrived under SFM"
        env.end(suppress_summary=True)

    def test_sfm_wander_runs_without_error(self):
        """Wander mode steps cleanly and re-samples goals on arrival."""
        cfg = {
            "world": {
                "height": 12,
                "width": 12,
                "step_time": 0.1,
                "sample_time": 0.1,
                "offset": [0, 0],
                "collision_mode": "unobstructed",
                "control_mode": "auto",
            },
            "robot": [
                {
                    "number": 3,
                    "distribution": {
                        "name": "random",
                        "range_low": [1, 1, -3.14],
                        "range_high": [11, 11, 3.14],
                    },
                    "kinematics": {"name": "diff"},
                    "shape": [{"name": "circle", "radius": 0.25}],
                    "behavior": {
                        "name": "sfm",
                        "vmax": 0.8,
                        "force_factor_social": 4.0,
                        "force_factor_desired": 0.6,
                        "wander": True,
                        "range_low": [1, 1, -3.14],
                        "range_high": [11, 11, 3.14],
                    },
                    "vel_min": [-1.5, -3],
                    "vel_max": [1.5, 3],
                    "arrive_mode": "position",
                    "goal_threshold": 0.2,
                }
            ],
        }
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        initial_goals = [r.goal.copy() for r in env.robot_list]
        for _ in range(300):
            env.step()
        # at least one robot should have arrived once and re-sampled a goal
        any_resampled = any(
            not np.allclose(r.goal, initial_goals[i])
            for i, r in enumerate(env.robot_list)
        )
        assert any_resampled, (
            "wander mode should re-sample at least one goal in 300 steps"
        )
        env.end(suppress_summary=True)


# ===================================================================
# End-to-end: ``group_behavior: sfm`` via temp YAML
# ===================================================================


def _corridor_world(height=8, width=14):
    return {
        "height": height,
        "width": width,
        "step_time": 0.1,
        "sample_time": 0.1,
        "offset": [-width / 2, -height / 2],
        "collision_mode": "unobstructed",
        "control_mode": "auto",
    }


def _wall(y, half_width=7):
    return {
        "shape": {
            "name": "linestring",
            "vertices": [[-half_width, y], [half_width, y]],
        },
        "state": [0, 0, 0],
        "unobstructed": True,
    }


class TestSFMGroupEndToEnd:
    def _single_robot_trajectory(self, behavior_key):
        cfg = {
            "world": _corridor_world(),
            "robot": [
                {
                    "kinematics": {"name": "diff"},
                    "shape": [{"name": "circle", "radius": 0.3}],
                    "state": [-6, 1.0, 0.3],
                    "goal": [6, -1.0, 0],
                    behavior_key: {"name": "sfm", "vmax": 1.0, "sigma_obstacle": 0.6},
                    "vel_min": [-1.5, -3],
                    "vel_max": [1.5, 3],
                    "arrive_mode": "position",
                    "goal_threshold": 0.2,
                }
            ],
            "obstacle": [
                _wall(2.0),
                {"shape": {"name": "circle", "radius": 0.5}, "state": [0, 0.3, 0]},
            ],
        }
        env = irsim.make(_write_temp_yaml(cfg), save_ani=False, display=False)
        traj = []
        for _ in range(150):
            env.step()
            traj.append(env.robot.state.copy())
        env.end(suppress_summary=True)
        return np.array(traj)

    def test_group_matches_per_object_behavior_for_one_robot(self):
        """With a single member the synchronous update is moot, so the group
        handler must reproduce the per-object ``sfm`` trajectory exactly,
        walls and circular obstacles included."""
        individual = self._single_robot_trajectory("behavior")
        grouped = self._single_robot_trajectory("group_behavior")
        np.testing.assert_allclose(grouped, individual, atol=1e-9)

    @pytest.mark.parametrize(
        ("kinematics", "vel_max"), [("diff", [1.5, 3]), ("omni", [1.5, 1.5])]
    )
    def test_pair_head_on_with_outside_robot_and_wall(self, kinematics, vel_max):
        """A two-member group passes head-on while a ``dash`` robot crosses
        the corridor; everyone reaches its goal."""
        cfg = {
            "world": _corridor_world(),
            "robot": [
                {
                    "number": 2,
                    "distribution": {"name": "manual"},
                    "kinematics": {"name": kinematics},
                    "shape": [{"name": "circle", "radius": 0.3}],
                    "state": [[-6, 0, 0], [6, 0.05, 3.14]],
                    "goal": [[6, 0, 0], [-6, 0.05, 3.14]],
                    "group_behavior": {
                        "name": "sfm",
                        "vmax": 1.0,
                        "force_factor_social": 5.0,
                        "gamma": 0.5,
                        "sigma_obstacle": 0.5,
                    },
                    "vel_min": [-v for v in vel_max],
                    "vel_max": vel_max,
                    "arrive_mode": "position",
                    "goal_threshold": 0.2,
                },
                {
                    "kinematics": {"name": "diff"},
                    "shape": [{"name": "circle", "radius": 0.3}],
                    "state": [0, -2.5, 1.57],
                    "goal": [0, 2.5, 0],
                    "behavior": {"name": "dash"},
                    "vel_min": [-1, -2],
                    "vel_max": [1, 2],
                    "arrive_mode": "position",
                    "goal_threshold": 0.2,
                },
            ],
            "obstacle": [_wall(3.0)],
        }
        env = irsim.make(_write_temp_yaml(cfg), save_ani=False, display=False)
        handler = env._object_groups[0].group_behavior._invoke_func
        assert type(handler).__name__ == "SfmGroupBehavior"
        for _ in range(400):
            env.step()
            if env.done():
                break
        assert env.done(), "group members and the crossing robot should all arrive"
        env.end(suppress_summary=True)

    def _pair_separation(self, social_groups):
        cfg = {
            "world": _corridor_world(),
            "robot": [
                {
                    "number": 2,
                    "distribution": {"name": "manual"},
                    "kinematics": {"name": "omni"},
                    "shape": [{"name": "circle", "radius": 0.25}],
                    "state": [[-6, 1.5, 0], [-6, -1.5, 0]],
                    "goal": [[6, 0.3, 0], [6, -0.3, 0]],
                    "group_behavior": {
                        "name": "sfm",
                        "vmax": 1.0,
                        "social_groups": social_groups,
                    },
                    "vel_min": [-1.5, -1.5],
                    "vel_max": [1.5, 1.5],
                    "arrive_mode": "position",
                    "goal_threshold": 0.3,
                }
            ],
        }
        env = irsim.make(_write_temp_yaml(cfg), save_ani=False, display=False)
        sep = []
        for _ in range(60):
            env.step()
            a, b = (r.state[:2, 0] for r in env.robot_list)
            sep.append(np.linalg.norm(a - b))
        env.end(suppress_summary=True)
        return np.array(sep)

    def test_social_group_closes_up_early(self):
        """Two pedestrians starting 3 m apart converge much sooner when they
        form a social group than when they merely share goals."""
        grouped = self._pair_separation([[0, 1]])
        loose = self._pair_separation([])
        assert grouped[20:].mean() < 0.6 * loose[20:].mean()
        # they never overlap while closing up
        assert grouped.min() > 0.5

    def test_invalid_social_groups_disable_the_handler(self):
        """An out-of-range index is reported at construction and the group
        falls back to having no handler instead of crashing ``make``."""
        cfg = {
            "world": _corridor_world(),
            "robot": [
                {
                    "number": 2,
                    "distribution": {"name": "manual"},
                    "kinematics": {"name": "omni"},
                    "shape": [{"name": "circle", "radius": 0.25}],
                    "state": [[-6, 1, 0], [-6, -1, 0]],
                    "goal": [[6, 1, 0], [6, -1, 0]],
                    "group_behavior": {"name": "sfm", "social_groups": [[0, 7]]},
                    "vel_min": [-1.5, -1.5],
                    "vel_max": [1.5, 1.5],
                }
            ],
        }
        env = irsim.make(_write_temp_yaml(cfg), save_ani=False, display=False)
        assert env._object_groups[0].group_behavior._invoke_func is None
        env.step()
        env.end(suppress_summary=True)


# ===================================================================
# Degenerate / edge-case branches in the SFM algorithm
# ===================================================================


class TestSFMEdgeCases:
    def test_social_force_skips_collocated_neighbor(self):
        # ``dist < 1e-6`` continue branch: a neighbour at the exact same
        # position must be skipped instead of dividing by zero.
        sfm = social_force_model(
            state=_state(x=0.0, y=0.0, vx=0.0, vy=0.0),
            neighbor_list=[[0.0, 0.0, 0.0, 0.0, 0.3]],
            lambda_importance=2.0,
        )
        fx, fy = sfm.social_force()
        assert fx == 0.0
        assert fy == 0.0

    def test_social_force_skips_zero_interaction_vector(self):
        # ``t_norm < 1e-9`` continue branch: with ``d_hat=(1,0)`` and
        # ``dv=(-0.5, 0)``, the interaction vector ``t = lambda*dv + d_hat``
        # is ``(-1+1, 0) = 0`` for ``lambda=2``. Without the guard this
        # would divide by zero; with it, the neighbour contributes nothing.
        sfm = social_force_model(
            state=_state(x=0.0, y=0.0, vx=-0.5, vy=0.0),
            neighbor_list=[[1.0, 0.0, 0.0, 0.0, 0.3]],
            lambda_importance=2.0,
        )
        fx, fy = sfm.social_force()
        assert fx == 0.0
        assert fy == 0.0

    def test_obstacle_force_skips_far_segments(self):
        # Segment well outside ``5 * sigma_obstacle`` must contribute zero.
        sfm = social_force_model(
            state=_state(x=0.0, y=0.0, r=0.3),
            line_obs_list=[[-1.0, 100.0, 1.0, 100.0]],
            sigma_obstacle=0.5,
        )
        assert sfm.obstacle_force() == [0.0, 0.0]

    def test_obstacle_force_handles_zero_distance_overlap(self):
        # Closest point on the segment coincides with the agent → the
        # "centre on the wall" branch fires and pushes along +x at unit
        # magnitude instead of dividing by zero.
        sfm = social_force_model(
            state=_state(x=0.0, y=0.0, r=0.3),
            line_obs_list=[[-1.0, 0.0, 1.0, 0.0]],
        )
        fx, fy = sfm.obstacle_force()
        assert fx == 1.0
        assert fy == 0.0

    def test_non_positive_divisors_rejected(self):
        # ``relaxation_time``, ``gamma`` and ``sigma_obstacle`` are all
        # divisors inside the force terms — non-positive values are a
        # logic error rather than weird-but-valid input.
        for kwargs in (
            {"relaxation_time": 0.0},
            {"relaxation_time": -0.1},
            {"gamma": 0.0},
            {"gamma": -0.5},
            {"sigma_obstacle": 0.0},
            {"sigma_obstacle": -1.0},
        ):
            param = next(iter(kwargs))
            with pytest.raises(ValueError, match=f"^{param} must be > 0"):
                social_force_model(state=_state(), **kwargs)

    def test_closest_point_on_degenerate_segment(self):
        # A "segment" whose endpoints coincide is a point; the helper must
        # short-circuit to that point instead of dividing by ``l2 == 0``.
        cx, cy = social_force_model._closest_point_on_segment(
            5.0, 5.0, [2.0, 3.0, 2.0, 3.0]
        )
        assert (cx, cy) == (2.0, 3.0)


# ===================================================================
# Per-tick reactive-state cache on ObjectBase
# ===================================================================


class TestReactiveStateCache:
    def test_velocity_xy_cache_hit_then_invalidated_on_step(self):
        """Cached value is reused until the next ``step()`` commits new state."""
        cfg = {
            "world": {
                "height": 10,
                "width": 10,
                "step_time": 0.1,
                "sample_time": 0.1,
                "offset": [0, 0],
                "collision_mode": "unobstructed",
                "control_mode": "auto",
            },
            "robot": [
                {
                    "kinematics": {"name": "diff"},
                    "shape": [{"name": "circle", "radius": 0.2}],
                    "state": [1.0, 1.0, 0.0],
                    "goal": [9.0, 9.0, 0.0],
                    "behavior": {"name": "dash"},
                    "vel_min": [-1, -1],
                    "vel_max": [1, 1],
                }
            ],
        }
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        robot = env.robot_list[0]

        # Cache miss → compute and store.
        first = robot.velocity_xy
        # Cache hit → identical object returned.
        second = robot.velocity_xy
        assert first is second

        # Same for the list properties.
        nb_first = robot.rvo_neighbor_state
        nb_second = robot.rvo_neighbor_state
        assert nb_first is nb_second

        # After step(), the velocity/state mutate → caches must invalidate.
        env.step()
        after = robot.velocity_xy
        nb_after = robot.rvo_neighbor_state
        assert after is not first
        assert nb_after is not nb_first
        env.end(suppress_summary=True)

    def test_sfm_with_goal_none_returns_zero_velocity(self):
        """``beh_diff_sfm`` / ``beh_omni_sfm`` must warn-and-stop when goal is None."""
        cfg = {
            "world": {
                "height": 6,
                "width": 6,
                "step_time": 0.1,
                "sample_time": 0.1,
                "offset": [-3, -3],
                "collision_mode": "unobstructed",
                "control_mode": "auto",
            },
            "robot": {
                "kinematics": {"name": "diff"},
                "shape": [{"name": "circle", "radius": 0.2}],
                "state": [0, 0, 0],
                "goal": [2, 0, 0],
                "behavior": {"name": "sfm", "vmax": 0.5},
                "vel_min": [-1, -1],
                "vel_max": [1, 1],
            },
        }
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        robot = env.robot_list[0]
        robot.set_goal(None)
        # gen_behavior_vel hits the goal-None branch — must return zeros.
        vel = robot.gen_behavior_vel()
        assert np.allclose(vel, np.zeros((2, 1)))
        env.end(suppress_summary=True)

    def test_omni_sfm_with_goal_none_returns_zero(self):
        """``beh_omni_sfm`` must warn-and-stop when goal is None."""
        cfg = {
            "world": {
                "height": 6,
                "width": 6,
                "step_time": 0.1,
                "sample_time": 0.1,
                "offset": [-3, -3],
                "collision_mode": "unobstructed",
                "control_mode": "auto",
            },
            "robot": {
                "kinematics": {"name": "omni"},
                "shape": [{"name": "circle", "radius": 0.2}],
                "state": [0, 0, 0],
                "goal": [2, 0, 0],
                "behavior": {"name": "sfm", "vmax": 0.5},
                "vel_min": [-1, -1],
                "vel_max": [1, 1],
            },
        }
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        robot = env.robot_list[0]
        robot.set_goal(None)
        vel = robot.gen_behavior_vel()
        assert np.allclose(vel, np.zeros((2, 1)))
        env.end(suppress_summary=True)

    def test_omni_sfm_with_neighbors_and_wall(self):
        """Omni-SFM scene with another agent and a linestring obstacle —
        exercises the per-object loop body in ``beh_omni_sfm`` (the
        ``if segs: line_segments.extend(segs)`` and ``else``-branch
        ``neighbors.append`` paths)."""
        cfg = {
            "world": {
                "height": 6,
                "width": 6,
                "step_time": 0.1,
                "sample_time": 0.1,
                "offset": [-3, -3],
                "collision_mode": "unobstructed",
                "control_mode": "auto",
            },
            "robot": [
                {
                    "kinematics": {"name": "omni"},
                    "shape": [{"name": "circle", "radius": 0.2}],
                    "state": [-2, 0, 0],
                    "goal": [2, 0, 0],
                    "behavior": {"name": "sfm", "vmax": 0.6},
                    "vel_min": [-1, -1],
                    "vel_max": [1, 1],
                    "arrive_mode": "position",
                    "goal_threshold": 0.3,
                },
                {
                    "kinematics": {"name": "omni"},
                    "shape": [{"name": "circle", "radius": 0.2}],
                    "state": [2, 0.05, 3.14],
                    "goal": [-2, 0.05, 3.14],
                    "behavior": {"name": "sfm", "vmax": 0.6},
                    "vel_min": [-1, -1],
                    "vel_max": [1, 1],
                    "arrive_mode": "position",
                    "goal_threshold": 0.3,
                },
            ],
            "obstacle": [
                {
                    "shape": {"name": "linestring", "vertices": [[-2, 1.5], [2, 1.5]]},
                    "state": [0, 0, 0],
                    "unobstructed": True,
                },
            ],
        }
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        for _ in range(30):
            env.step()  # just exercise the code path
        env.end(suppress_summary=True)

    def test_sfm_velocity_handles_none_inputs(self):
        """``SFMVelocity`` defaults ``neighbor_list``/``line_segments`` to []
        when called with ``None`` — hits the two guard branches at the top."""
        from irsim.lib.behavior.behavior_methods import SFMVelocity

        state = [0.0, 0.0, 0.0, 0.0, 0.3, 1.0, 0.0, 0.0]  # heading east, vmax desired
        vx, vy = SFMVelocity(state, neighbor_list=None, line_segments=None, vmax=1.0)
        # No neighbours, no walls — desired-force only, finite output.
        assert np.isfinite(vx)
        assert np.isfinite(vy)

    def test_omni_sfm_end_to_end(self):
        """End-to-end exercise of ``beh_omni_sfm`` (was registration-only).

        Asserts the behaviour produces a non-zero velocity and the agent
        makes net progress toward its goal — full arrival is not required,
        since the goal here is just to invoke the omni-SFM code path.
        """
        cfg = {
            "world": {
                "height": 8,
                "width": 8,
                "step_time": 0.1,
                "sample_time": 0.1,
                "offset": [-4, -4],
                "collision_mode": "unobstructed",
                "control_mode": "auto",
            },
            "robot": {
                "kinematics": {"name": "omni"},
                "shape": [{"name": "circle", "radius": 0.2}],
                "state": [-3, 0, 0],
                "goal": [3, 0, 0],
                "behavior": {"name": "sfm", "vmax": 0.8},
                "vel_min": [-1.5, -1.5],
                "vel_max": [1.5, 1.5],
                "arrive_mode": "position",
                "goal_threshold": 0.3,
            },
        }
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        robot = env.robot_list[0]
        start_x = float(robot.state[0, 0])
        for _ in range(50):
            env.step()
        # The omni-SFM behavior must have produced motion toward the goal.
        assert float(robot.state[0, 0]) > start_x + 1.0
        env.end(suppress_summary=True)

    def test_sfm_with_line_obstacle_in_scene(self):
        """End-to-end with a linestring obstacle: covers the
        ``if segs: line_segments.extend(segs)`` branch in ``beh_diff_sfm``."""
        cfg = {
            "world": {
                "height": 6,
                "width": 12,
                "step_time": 0.1,
                "sample_time": 0.1,
                "offset": [-6, -3],
                "collision_mode": "unobstructed",
                "control_mode": "auto",
            },
            "robot": {
                "kinematics": {"name": "diff"},
                "shape": [{"name": "circle", "radius": 0.2}],
                "state": [-5, 0, 0],
                "goal": [5, 0, 0],
                "behavior": {"name": "sfm", "vmax": 0.8},
                "vel_min": [-1.5, -3],
                "vel_max": [1.5, 3],
                "arrive_mode": "position",
                "goal_threshold": 0.3,
            },
            "obstacle": [
                {
                    "shape": {"name": "linestring", "vertices": [[-5, 2], [5, 2]]},
                    "state": [0, 0, 0],
                    "unobstructed": True,
                },
            ],
        }
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        for _ in range(300):
            env.step()
            if env.done():
                break
        assert env.done(), "robot must arrive past the linestring wall"
        env.end(suppress_summary=True)

    def test_rvo_line_segments_invalidated_when_linestring_moves(self):
        """A dynamic linestring obstacle re-transforms its vertices each
        step; ``rvo_line_segments`` must invalidate so the cached value
        does not lag behind the actual geometry."""
        cfg = {
            "world": {
                "height": 10,
                "width": 10,
                "step_time": 0.1,
                "sample_time": 0.1,
                "offset": [-5, -5],
                "collision_mode": "unobstructed",
                "control_mode": "auto",
            },
            "robot": {
                "kinematics": {"name": "diff"},
                "shape": [{"name": "circle", "radius": 0.2}],
                "state": [-3, -2, 0],
                "goal": [3, -2, 0],
                "behavior": {"name": "dash"},
                "vel_min": [-1, -1],
                "vel_max": [1, 1],
            },
            "obstacle": [
                {
                    "kinematics": {"name": "omni"},
                    "shape": {"name": "linestring", "vertices": [[-1, 0], [1, 0]]},
                    "state": [0, 0, 0],
                    "goal": [0, 3, 0],
                    "behavior": {"name": "dash"},
                    "vel_max": [1, 1],
                    "unobstructed": True,
                },
            ],
        }
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        moving_line = env.obstacle_list[0]
        before = moving_line.rvo_line_segments
        before_copy = [list(seg) for seg in before]
        for _ in range(20):
            env.step()
        after = moving_line.rvo_line_segments
        # Segment endpoints must have translated with the obstacle's motion.
        assert after != before_copy, (
            "rvo_line_segments cache went stale after the linestring moved"
        )
        env.end(suppress_summary=True)

    def test_cache_invalidated_by_set_state_set_velocity(self):
        """Manual state/velocity mutation must drop the per-tick caches
        so the next reactive read sees the new pose, not the previous
        cached one."""
        cfg = {
            "world": {
                "height": 10,
                "width": 10,
                "step_time": 0.1,
                "sample_time": 0.1,
                "offset": [0, 0],
                "collision_mode": "unobstructed",
                "control_mode": "auto",
            },
            "robot": {
                "kinematics": {"name": "diff"},
                "shape": [{"name": "circle", "radius": 0.2}],
                "state": [1, 1, 0],
                "goal": [9, 9, 0],
                "behavior": {"name": "dash"},
                "vel_min": [-1, -1],
                "vel_max": [1, 1],
            },
        }
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        robot = env.robot_list[0]

        # Prime the caches.
        first = robot.velocity_xy
        nb_first = robot.rvo_neighbor_state

        # Mutate state externally — caches must drop.
        robot.set_state([5, 5, 0])
        after_state = robot.velocity_xy
        nb_after_state = robot.rvo_neighbor_state
        assert after_state is not first
        assert nb_after_state is not nb_first
        # x coordinate of the cached neighbour state must reflect the
        # new pose, not the old one (centroid-based, so approximate).
        assert nb_after_state[0] == pytest.approx(5.0)

        # Mutate velocity externally — caches must drop again.
        primed_vxy = robot.velocity_xy
        robot.set_velocity([0.5, 0.0])
        assert robot.velocity_xy is not primed_vxy
        env.end(suppress_summary=True)

    def test_cache_invalidated_by_reset(self):
        """``env.reset()`` returns the object to its initial state; the
        caches populated during the previous run must not leak across
        the reset boundary."""
        cfg = {
            "world": {
                "height": 10,
                "width": 10,
                "step_time": 0.1,
                "sample_time": 0.1,
                "offset": [0, 0],
                "collision_mode": "unobstructed",
                "control_mode": "auto",
            },
            "robot": {
                "kinematics": {"name": "diff"},
                "shape": [{"name": "circle", "radius": 0.2}],
                "state": [1, 1, 0],
                "goal": [9, 9, 0],
                "behavior": {"name": "dash"},
                "vel_min": [-1, -1],
                "vel_max": [1, 1],
            },
        }
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        robot = env.robot_list[0]

        for _ in range(5):
            env.step()
        moved_x = float(robot.state[0, 0])
        assert moved_x > 1.0

        primed = robot.velocity_xy
        primed_nb = robot.rvo_neighbor_state
        env.reset()
        # reset() must drop the cached velocity/neighbour-state.
        post_reset_nb = robot.rvo_neighbor_state
        assert post_reset_nb is not primed_nb
        assert post_reset_nb[0] == pytest.approx(1.0)  # back to initial x
        # velocity_xy was also invalidated; new lookup recomputes.
        assert robot.velocity_xy is not primed
        env.end(suppress_summary=True)

    def test_rvo_line_segments_cached_per_object(self):
        """Linestring obstacles cache vertices once; non-linestrings cache []."""
        cfg = {
            "world": {
                "height": 10,
                "width": 10,
                "step_time": 0.1,
                "sample_time": 0.1,
                "offset": [-5, -5],
                "collision_mode": "unobstructed",
                "control_mode": "auto",
            },
            "robot": {
                "kinematics": {"name": "diff"},
                "shape": [{"name": "circle", "radius": 0.2}],
                "state": [-3, 0, 0],
                "goal": [3, 0, 0],
                "behavior": {"name": "dash"},
                "vel_min": [-1, -1],
                "vel_max": [1, 1],
            },
            "obstacle": [
                {
                    "shape": {"name": "linestring", "vertices": [[-2, 1], [2, 1]]},
                    "state": [0, 0, 0],
                    "unobstructed": True,
                },
                {
                    "shape": {"name": "circle", "radius": 0.3},
                    "state": [0, -2, 0],
                    "unobstructed": True,
                },
            ],
        }
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        line_obs = env.obstacle_list[0]
        circle_obs = env.obstacle_list[1]

        # Linestring: same list object returned on second access.
        segs_a = line_obs.rvo_line_segments
        segs_b = line_obs.rvo_line_segments
        assert segs_a is segs_b
        assert len(segs_a) == 1  # one segment between the two vertices

        # Non-linestring: cached empty list returned on second access.
        empty_a = circle_obs.rvo_line_segments
        empty_b = circle_obs.rvo_line_segments
        assert empty_a is empty_b
        assert empty_a == []
        env.end(suppress_summary=True)
