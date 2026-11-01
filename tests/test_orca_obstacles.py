"""The ORCA group behavior avoids the objects around the group that target_roles selects."""

import importlib.util
from types import SimpleNamespace

import numpy as np
import pytest
from shapely.geometry import LinearRing, LineString, MultiLineString, Point, Polygon

import irsim
from irsim.lib.behavior.group_behavior_methods import OrcaGroupBehavior

pyrvo_required = pytest.mark.skipif(
    importlib.util.find_spec("pyrvo") is None, reason="pyrvo not installed"
)

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
    state: [4.5, 6.5, 0]
    unobstructed: true  # collisions off, still in the way
"""

WALL = LineString([(0.5, 5), (4, 5)])
CIRCLE = Point(4.5, 6.5)


@pytest.fixture
def env(tmp_path):
    path = tmp_path / "orca_obstacles.yaml"
    path.write_text(SCENE)
    env = irsim.make(str(path), display=False, log_level="ERROR")
    yield env
    env.end()


def _orca(env):
    return env._object_groups[0].group_behavior._invoke_func


@pyrvo_required
def test_static_surroundings_become_obstacles(env):
    """Walls and polygon obstacles enter the simulator once the group moves."""
    orca = _orca(env)
    assert orca._loaded is None
    env.step()
    assert orca._loaded is not None
    # one two-vertex obstacle for the wall segment, one ring each for the
    # rectangle and the circle: unobstructed turns collisions off, not avoidance.
    # pyrvo may split segments while building its obstacle tree, so at least.
    circle = env.obstacle_list[2]
    assert circle.unobstructed
    ring = len(circle.geometry.exterior.coords) - 1
    assert orca._sim.get_num_obstacle_vertices() >= 2 + 4 + ring


@pyrvo_required
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
        # the body never overlaps the wall segment, nor the unobstructed circle
        assert WALL.distance(Point(*wall.state[:2, 0])) >= 0.3 - 1e-6
        assert CIRCLE.distance(Point(*wall.state[:2, 0])) >= 0.5 + 0.3 - 1e-6
        if wall.arrive and box.arrive:
            break
    assert wall.arrive, wall.state[:2, 0].tolist()
    assert box.arrive, box.state[:2, 0].tolist()
    assert np.hypot(*(wall.state[:2, 0] - np.array([5.5, 8.0]))) < 0.3


@pyrvo_required
def test_moved_wall_reloads_obstacles(env):
    """A static object moved after the first step is reloaded where it now is."""
    orca = _orca(env)
    env.step()
    before = orca._sim
    env.obstacle_list[1].set_state([8, 2, 0])  # the rectangle, from y 4.8..5.2
    env.step()
    assert orca._sim is not before
    lowest = min(
        orca._sim.get_obstacle_vertex(i).to_tuple()[1]
        for i in range(orca._sim.get_num_obstacle_vertices())
    )
    assert lowest == pytest.approx(1.8)


@pyrvo_required
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


class _FakeSim:
    def __init__(self):
        self.obstacles = []

    def add_obstacle(self, coords):
        self.obstacles.append(coords)

    def process_obstacles(self):
        pass


def _orca_with_fake_sim():
    orca = OrcaGroupBehavior.__new__(OrcaGroupBehavior)
    orca._sim = _FakeSim()
    return orca


def _static(**attributes):
    defaults = {"static": True, "shape": "polygon"}
    return SimpleNamespace(**{**defaults, **attributes})


class TestAddObstacles:
    def test_nothing_outside_adds_nothing(self):
        orca = _orca_with_fake_sim()
        assert orca._add_obstacles([]) == 0

    def test_polygon_holes_become_walls_too(self):
        courtyard = Polygon(
            [(0, 0), (6, 0), (6, 6), (0, 6)], [[(2, 2), (2, 4), (4, 4), (4, 2)]]
        )
        orca = _orca_with_fake_sim()
        assert orca._add_obstacles([_static(geometry=courtyard)]) == 2
        outer, inner = orca._sim.obstacles
        # pyrvo: counter-clockwise for a solid, clockwise for the pocket it bounds
        assert LinearRing(outer).is_ccw
        assert not LinearRing(inner).is_ccw

    def test_map_without_grid_falls_back_to_its_geometry(self):
        wall = MultiLineString([[(0, 1), (3, 1)], [(3, 1), (3, 4)]])
        gridless = _static(shape="map", grid_map=None, geometry=wall)
        orca = _orca_with_fake_sim()
        assert orca._add_obstacles([gridless]) == 2

    def test_moving_objects_are_not_walls(self):
        mover = _static(static=False, geometry=Point(5, 5).buffer(0.3))
        assert _orca_with_fake_sim()._add_obstacles([mover]) == 0


MOVING_SCENE = """
world:
  height: 10
  width: 10
  step_time: 0.1
  collision_mode: 'unobstructed'

robot:
  - number: 2
    distribution: {name: 'manual'}
    state: [[1, 5, 0], [9, 5.6, 0]]
    goal: [[9, 5, 0], [1, 5.6, 0]]
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
  - kinematics: {name: 'omni'}
    shape: {name: 'circle', radius: 0.4}
    state: [5, 2.5, 0]
    goal: [5, 9, 0]
    vel_max: [1.0, 1.0]
    vel_min: [-1.0, -1.0]
    behavior: {name: 'dash'}
"""


@pytest.fixture
def moving_env(tmp_path):
    path = tmp_path / "orca_moving.yaml"
    path.write_text(MOVING_SCENE)
    env = irsim.make(str(path), display=False, log_level="ERROR")
    yield env
    env.end()


class TestMovingObjects:
    def test_only_moving_objects_become_agents(self):
        mover = _static(static=False)
        orca = _orca_with_fake_sim()
        assert orca._moving_objects([_static(), mover]) == [mover]
        assert orca._moving_objects([]) == []

    def test_scene_follows_agents_and_walls(self):
        member = SimpleNamespace(id=0)
        wall = _static(id=1, state=np.zeros((3, 1)))
        box = _static(id=2, state=np.ones((3, 1)))
        mover = _static(id=3, static=False)
        scene = OrcaGroupBehavior._scene([member], [mover], [wall, box, mover])
        assert scene == OrcaGroupBehavior._scene([member], [mover], [wall, box, mover])
        # the mover left, or another object took its place
        assert scene != OrcaGroupBehavior._scene([member], [], [wall, box])
        other = _static(id=4, static=False)
        assert scene != OrcaGroupBehavior._scene([member], [other], [wall, box, other])
        # the box moved
        moved = _static(id=2, state=np.full((3, 1), 2.0))
        assert scene != OrcaGroupBehavior._scene(
            [member], [mover], [wall, moved, mover]
        )


@pyrvo_required
def test_moving_objects_are_avoided_as_agents(moving_env):
    """A dynamic obstacle crossing the lane is an agent the members steer around."""
    orca = _orca(moving_env)
    mover = moving_env.obstacle_list[0]
    robots = moving_env.robot_list
    moving_env.step()
    assert orca._sim.get_num_agents() == 3  # two members and the mover
    for _ in range(300):
        moving_env.step()
        for robot in robots:
            assert not robot.collision
            gap = np.hypot(*(robot.state[:2, 0] - mover.state[:2, 0]))
            assert gap >= 0.3 + 0.4 - 1e-6
        # the simulator keeps the mover where the scene has it
        seen = np.array(orca._sim.get_agent_position(2).to_tuple())
        assert np.hypot(*(seen - mover.state[:2, 0])) < 0.2
        if all(r.arrive for r in robots):
            break
    assert all(r.arrive for r in robots)


ROBOT_OUTSIDE = """
  - kinematics: {name: 'omni'}
    shape: {name: 'circle', radius: 0.3}
    state: [5, 8, 0]
    goal: [5, 1, 0]
    behavior: {name: 'dash'}
"""


@pyrvo_required
@pytest.mark.parametrize(
    ("target_roles", "avoided"),
    [("", {"obstacle"}), ("all", {"obstacle", "robot"}), ("robot", {"robot"})],
)
def test_target_roles_selects_the_outsiders(tmp_path, target_roles, avoided):
    """The obstacles are avoided by default; 'all' makes way for a robot too."""
    scene = MOVING_SCENE.replace("\nobstacle:", ROBOT_OUTSIDE + "\nobstacle:")
    if target_roles:
        scene = scene.replace(
            "safe_radius: 0.05",
            f"safe_radius: 0.05\n      target_roles: '{target_roles}'",
        )
    path = tmp_path / "orca_target_roles.yaml"
    path.write_text(scene)
    env = irsim.make(str(path), display=False, log_level="ERROR")
    try:
        assert len(env.robot_list) == 3
        env.step()
        outside = env._object_groups[0].group_behavior.target_objects
        assert {obj.role for obj in outside} == avoided
        orca = _orca(env)
        assert orca._sim.get_num_agents() == 2 + len(orca._moving_objects(outside))
    finally:
        env.end()


@pyrvo_required
def test_simulator_follows_moving_objects_removed(moving_env):
    """Deleting a moving object rebuilds the simulator without its agent."""
    orca = _orca(moving_env)
    moving_env.step()
    assert orca._sim.get_num_agents() == 3
    moving_env.delete_object(moving_env.obstacle_list[0].id)
    moving_env.step()
    assert orca._sim.get_num_agents() == 2
    assert orca._loaded is not None


@pyrvo_required
def test_replaced_moving_object_gets_its_own_agent(moving_env):
    """A mover swapped for a bigger one between steps gets an agent of its size."""
    orca = _orca(moving_env)
    moving_env.step()
    assert orca._sim.get_agent_radius(2) == pytest.approx(0.4 + 0.05)
    moving_env.delete_object(moving_env.obstacle_list[0].id)
    moving_env.add_object(
        moving_env.create_obstacle(
            kinematics={"name": "omni"},
            shape={"name": "circle", "radius": 0.8},
            state=[5, 2.5, 0],
            goal=[5, 9, 0],
            behavior={"name": "dash"},
        )
    )
    moving_env.step()
    assert orca._sim.get_num_agents() == 3
    assert orca._sim.get_agent_radius(2) == pytest.approx(0.8 + 0.05)
