"""Tests for the Social Force Model behavior.

Covers:
- social_force_model algorithm: desired/social/obstacle forces, vmax clipping.
- ("diff", "sfm") and ("omni", "sfm") behavior registration.
- End-to-end env step with `behavior: sfm` via a temp YAML.
"""

import importlib
import tempfile

import numpy as np
import yaml

import irsim
from irsim.lib.algorithm.social_force_model import social_force_model
from irsim.lib.behavior.behavior_registry import behaviors_map

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


def _write_temp_yaml(cfg: dict) -> str:
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        yaml.safe_dump(cfg, f)
        return f.name


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
# Degenerate / edge-case branches in the SFM algorithm
# ===================================================================


class TestSFMEdgeCases:
    def test_social_force_skips_degenerate_interaction(self):
        # If a neighbour has the exact same position AND velocity as ego,
        # both ``d_hat`` and ``dv`` are zero so ``t_norm < 1e-9``; the
        # neighbour must be skipped instead of dividing by zero.
        sfm = social_force_model(
            state=_state(x=0.0, y=0.0, vx=0.0, vy=0.0),
            neighbor_list=[[0.0, 0.0, 0.0, 0.0, 0.3]],
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
