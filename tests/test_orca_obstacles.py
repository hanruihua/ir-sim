"""The ORCA group behavior treats the static objects around the group as obstacles."""

import numpy as np
import pytest
from shapely.geometry import LineString, Point

import irsim

pytest.importorskip("pyrvo")

SCENE = """
world:
  height: 10
  width: 10
  step_time: 0.1
  collision_mode: 'unobstructed'

robot:
  - number: 2
    distribution: {name: 'manual'}
    state: [[2, 2, 0], [8, 2, 0]]
    goal: [[5.5, 8, 0], [6, 8, 0]]
    kinematics: {name: 'omni'}
    shape: {name: 'circle', radius: 0.3}
    vel_max: [1.5, 1.5]
    vel_min: [-1.5, -1.5]
    goal_threshold: 0.2
    group_behavior:
      name: 'orca'
      neighborDist: 5.0
      maxNeighbors: 5
      timeHorizon: 2.0
      timeHorizonObst: 2.0
      safe_radius: 0.05

obstacle:
  - shape: {name: 'linestring', vertices: [[0.5, 5], [4, 5]]}
    state: [0, 0, 0]
  - shape: {name: 'rectangle', length: 3, width: 0.4}
    state: [8, 5, 0]
  - shape: {name: 'circle', radius: 0.5}
    state: [5, 5, 0]
    unobstructed: true
"""

WALL = LineString([(0.5, 5), (4, 5)])


@pytest.fixture
def env(tmp_path):
    path = tmp_path / "orca_obstacles.yaml"
    path.write_text(SCENE)
    env = irsim.make(str(path), display=False, log_level="ERROR")
    yield env
    env.end()


def _orca(env):
    return env._object_groups[0].group_behavior._invoke_func


def test_static_surroundings_become_obstacles(env):
    """Walls and polygon obstacles enter the simulator once the group moves."""
    orca = _orca(env)
    assert not orca._obstacles_loaded
    env.step()
    assert orca._obstacles_loaded
    # one two-vertex obstacle for the wall segment, one ring for the rectangle;
    # the unobstructed circle is left out
    assert orca._sim.get_num_obstacle_vertices() == 2 + 4


def test_members_go_around_walls(env):
    """Members whose straight path crosses a wall slide around it, untouched.

    ORCA is a local method, so the goals sit off the perpendicular: a member
    heading straight at a flat wall would stall against it.
    """
    wall, box = env.robot_list
    for _ in range(300):
        env.step()
        assert not wall.collision
        assert not box.collision
        # the body never overlaps the wall segment
        assert WALL.distance(Point(*wall.state[:2, 0])) >= 0.3 - 1e-6
        if wall.arrive and box.arrive:
            break
    assert wall.arrive, wall.state[:2, 0].tolist()
    assert box.arrive, box.state[:2, 0].tolist()
    assert np.hypot(*(wall.state[:2, 0] - np.array([5.5, 8.0]))) < 0.3


def test_rebuilt_simulator_reloads_obstacles(env):
    """A simulator rebuilt for a new member count gets the obstacles again."""
    orca = _orca(env)
    env.step()
    before = orca._sim.get_num_obstacle_vertices()
    orca._sim = orca._build_sim(env.robot_list)
    assert not orca._obstacles_loaded
    env.step()
    assert orca._sim.get_num_obstacle_vertices() == before


def test_grid_map_walls_are_obstacles(tmp_path):
    """The occupancy grid enters the simulator as its boundary, a few lines."""
    from PIL import Image

    img = np.full((40, 40), 255, np.uint8)
    img[16:20, 12:28] = 0  # a bar at x 3..7 m, y 5..6 m of a 10 m world
    Image.fromarray(img).save(tmp_path / "bar.png")
    scene = f"""
world:
  height: 10
  width: 10
  step_time: 0.1
  collision_mode: 'unobstructed'
  obstacle_map: '{tmp_path / "bar.png"}'

robot:
  - number: 2
    distribution: {{name: 'manual'}}
    state: [[1.5, 2, 0], [8.5, 8, 0]]
    goal: [[8.5, 8, 0], [1.5, 2, 0]]
    kinematics: {{name: 'omni'}}
    shape: {{name: 'circle', radius: 0.3}}
    vel_max: [1.5, 1.5]
    vel_min: [-1.5, -1.5]
    goal_threshold: 0.2
    group_behavior:
      name: 'orca'
      neighborDist: 5.0
      maxNeighbors: 5
      timeHorizon: 2.0
      timeHorizonObst: 2.0
      safe_radius: 0.05
"""
    path = tmp_path / "orca_map.yaml"
    path.write_text(scene)
    env = irsim.make(str(path), display=False, log_level="ERROR")
    try:
        orca = _orca(env)
        for _ in range(300):
            env.step()
            assert not any(r.collision for r in env.robot_list)
            if all(r.arrive for r in env.robot_list):
                break
        assert all(r.arrive for r in env.robot_list)
        # the bar is one rectangle: four segments, not one box per cell
        assert 0 < orca._sim.get_num_obstacle_vertices() <= 16
    finally:
        env.end()
