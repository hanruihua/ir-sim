"""Tests for the static/dynamic STRtree cache optimisation in EnvBase.build_tree.

Strategy
--------
* Verify that after the optimisation the *static-geometry cache* is populated
  on the first ``build_tree()`` call and reused on subsequent calls.
* Verify that adding or removing an object invalidates the cache (triggering a
  full re-partition on the next ``build_tree()`` call).
* Verify that collision detection results — the primary consumer of the tree —
  are identical to the unoptimised behaviour.
"""
from __future__ import annotations

import contextlib
import os
import tempfile

import pytest
import yaml

import irsim

# ---------------------------------------------------------------------------
# YAML helpers
# ---------------------------------------------------------------------------

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
    yield
    while _TEMP_YAML_PATHS:
        path = _TEMP_YAML_PATHS.pop()
        with contextlib.suppress(FileNotFoundError):
            os.unlink(path)


def _simple_cfg(*, n_dynamic: int = 1) -> dict:
    """A small world with one static box obstacle and n_dynamic diff robots."""
    robots = [
        {
            "kinematics": {"name": "diff"},
            "shape": [{"name": "circle", "radius": 0.2}],
            "state": [float(i * 2 - 3), 0.0, 0.0],
            "goal": [float(i * 2 + 2), 0.0, 0.0],
            "behavior": {"name": "dash"},
            "vel_min": [-1, -1],
            "vel_max": [1, 1],
        }
        for i in range(n_dynamic)
    ]
    cfg: dict = {
        "world": {
            "height": 12,
            "width": 12,
            "step_time": 0.1,
            "sample_time": 0.1,
            "offset": [-6, -6],
            "collision_mode": "unobstructed",
            "control_mode": "auto",
        },
        "robot": robots if n_dynamic > 1 else robots[0],
        "obstacle": {
            # A static rectangle — should go into the static-geom cache.
            "shape": {"name": "rectangle", "length": 1.0, "width": 1.0},
            "state": [0, 3, 0],
        },
    }
    return cfg


@pytest.fixture
def env_with_static():
    """Environment: one static obstacle + one dynamic robot."""
    cfg = _simple_cfg(n_dynamic=1)
    path = _write_temp_yaml(cfg)
    env = irsim.make(path, save_ani=False, display=False)
    yield env
    env.end(suppress_summary=True)


# ---------------------------------------------------------------------------
# Cache structure after initialisation
# ---------------------------------------------------------------------------


class TestCacheStructure:
    def test_cache_exists_after_build(self, env_with_static):
        env = env_with_static
        assert hasattr(env, "_static_geom_cache"), (
            "EnvBase.build_tree() must populate '_static_geom_cache'"
        )
        assert env._static_geom_cache is not None

    def test_cache_length_matches_objects(self, env_with_static):
        env = env_with_static
        assert len(env._static_geom_cache) == len(env.objects)

    def test_static_slots_are_filled(self, env_with_static):
        env = env_with_static
        for i, obj in enumerate(env.objects):
            if obj.static:
                assert env._static_geom_cache[i] is not None, (
                    f"Static object at index {i} must have a cached geometry"
                )

    def test_dynamic_indices_lists_only_dynamic(self, env_with_static):
        env = env_with_static
        assert hasattr(env, "_dynamic_indices")
        for idx in env._dynamic_indices:
            assert not env.objects[idx].static, (
                f"Index {idx} is in _dynamic_indices but points to a static object"
            )

    def test_static_objects_not_in_dynamic_indices(self, env_with_static):
        env = env_with_static
        dynamic_set = set(env._dynamic_indices)
        for i, obj in enumerate(env.objects):
            if obj.static:
                assert i not in dynamic_set, (
                    f"Static object at index {i} must not appear in _dynamic_indices"
                )


# ---------------------------------------------------------------------------
# Cache reuse — static geometry is not re-extracted each step
# ---------------------------------------------------------------------------


class TestCacheReuse:
    def test_static_cached_object_is_same_reference(self, env_with_static):
        """The cached geometry for a static object must be the exact same object
        as retrieved directly from obj.geometry (pointer equality), showing that
        the cache was populated once and never rebuilt."""
        env = env_with_static
        for i, obj in enumerate(env.objects):
            if obj.static:
                assert env._static_geom_cache[i] is obj.geometry, (
                    f"Cache slot {i} is a copy, not the original geometry reference"
                )

    def test_cache_identity_preserved_across_steps(self, env_with_static):
        """The cache list object must be the same Python list between steps —
        proving static geometries are not re-extracted."""
        env = env_with_static
        cache_id_before = id(env._static_geom_cache)
        for _ in range(5):
            env.step()
        cache_id_after = id(env._static_geom_cache)
        # Same list object means build_tree() reused it, not replaced it.
        assert cache_id_before == cache_id_after


# ---------------------------------------------------------------------------
# Cache invalidation on object addition / deletion
# ---------------------------------------------------------------------------


class TestCacheInvalidationOnMutation:
    def test_invalidated_after_delete(self, env_with_static):
        """Deleting an object must trigger a re-partition (cache shrinks)."""
        env = env_with_static
        original_n = len(env._static_geom_cache)
        # Delete the robot (dynamic).
        robot_id = env.robot_list[0].id
        env.delete_object(robot_id)
        assert len(env._static_geom_cache) == original_n - 1

    def test_partition_updated_after_delete(self, env_with_static):
        """After deletion the cache and dynamic-index list stay consistent."""
        env = env_with_static
        robot_id = env.robot_list[0].id
        env.delete_object(robot_id)
        # Only the static obstacle remains — no dynamic indices.
        assert env._dynamic_indices == []
        assert len(env._static_geom_cache) == 1
        assert env._static_geom_cache[0] is not None  # static obstacle


# ---------------------------------------------------------------------------
# Collision detection correctness
# ---------------------------------------------------------------------------


class TestCollisionDetectionCorrectness:
    def test_no_collision_when_not_touching(self):
        """Robot far from the static obstacle must not register a collision."""
        cfg = _simple_cfg(n_dynamic=1)
        # Robot is at (-3, 0) goal (2, 0); static obstacle at (0, 3) — safe distance.
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        collisions = env._colliding_objects()
        robot_id = env.robot_list[0].id
        assert env.objects[0] in env.objects  # sanity
        # Robot should not be colliding with the static obstacle at step 0.
        robot_collisions = [o for o in collisions.get(robot_id, []) if o.static]
        assert robot_collisions == []
        env.end(suppress_summary=True)

    def test_results_consistent_across_repeated_builds(self):
        """_colliding_objects() must return the same results when called twice
        in a row without any step in between (both calls use the same tree)."""
        cfg = _simple_cfg(n_dynamic=1)
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        for _ in range(3):
            env.step()
        c1 = env._colliding_objects()
        env.build_tree()
        c2 = env._colliding_objects()
        assert set(c1.keys()) == set(c2.keys())
        for obj_id in c1:
            ids1 = {o.id for o in c1[obj_id]}
            ids2 = {o.id for o in c2[obj_id]}
            assert ids1 == ids2
        env.end(suppress_summary=True)

    def test_many_dynamic_objects(self):
        """Environment with several dynamic robots still uses cache correctly."""
        cfg = _simple_cfg(n_dynamic=4)
        path = _write_temp_yaml(cfg)
        env = irsim.make(path, save_ani=False, display=False)
        # Cache should have exactly one static slot (the obstacle) populated.
        n_static_slots = sum(
            1 for i, obj in enumerate(env.objects)
            if obj.static and env._static_geom_cache[i] is not None
        )
        n_static_objects = sum(1 for obj in env.objects if obj.static)
        assert n_static_slots == n_static_objects
        # Simulate a few steps without errors.
        for _ in range(5):
            env.step()
        assert len(env._static_geom_cache) == len(env.objects)
        env.end(suppress_summary=True)
