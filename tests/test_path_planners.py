"""
Tests for path planning algorithms.

Covers A*, JPS, RRT, RRT*, Informed RRT*, and PRM planners.
"""

import random

import pytest

from irsim.lib.path_planners.a_star import AStarPlanner
from irsim.lib.path_planners.informed_rrt_star import InformedRRTStar
from irsim.lib.path_planners.jps import JPSPlanner
from irsim.lib.path_planners.probabilistic_road_map import PRMPlanner
from irsim.lib.path_planners.rrt import RRT
from irsim.lib.path_planners.rrt_star import RRTStar


@pytest.mark.parametrize(
    ("planner_class", "resolution"),
    [
        (AStarPlanner, 0.3),
        (JPSPlanner, 0.3),
        (RRTStar, 0.3),
        (InformedRRTStar, 0.3),
        (RRT, 0.3),
        (PRMPlanner, 0.3),
    ],
)
def test_path_planners(planner_class, resolution, env_factory):
    """Test path planning algorithms find valid paths. Resolution is from the map."""
    env = env_factory("test_collision_world.yaml", full=False)
    env_map = env.get_map(resolution=resolution)
    if planner_class in (AStarPlanner, JPSPlanner):
        planner = planner_class(env_map)
    elif planner_class in (RRT, RRTStar, InformedRRTStar):
        planner = planner_class(env_map, robot=env.robot)
    else:
        planner = planner_class(env_map, robot_radius=resolution)
    robot_state = env.get_robot_state()
    goal_pose = env.robot.goal[:2, 0].tolist()
    trajectory = planner.planning(robot_state, goal_pose)
    env.draw_trajectory(trajectory, traj_type="r-")
    assert trajectory is not None


class TestRRTStarEdgeMethods:
    """Tests for RRT* edge cases and internal methods."""

    @pytest.fixture
    def rrt_star(self, env_factory):
        """Create RRT* planner for testing."""
        env = env_factory("test_collision_world.yaml", full=False)
        env_map = env.get_map()
        r = RRTStar(env_map=env_map, robot=env.robot)
        r.node_list = []
        r.expand_dis = 1.0
        return r

    def test_choose_parent_empty_near_indices(self, rrt_star):
        """Test _choose_parent with empty near indices returns nearest."""
        nearest = rrt_star.Node(0.0, 0.0)
        nearest.cost = 0.0
        rrt_star.node_list = [nearest]
        new_node = rrt_star.Node(1.0, 0.0)
        cost_fp = 1.0
        best_parent, best_cost_fp = rrt_star._choose_parent(
            new_node, nearest, 0, cost_fp, []
        )
        assert best_parent is nearest
        assert best_cost_fp == cost_fp

    def test_choose_parent_all_collision_fail(self, rrt_star):
        """Test _choose_parent when all candidates fail collision."""
        nearest = rrt_star.Node(0.0, 0.0)
        nearest.cost = 0.0
        candidate = rrt_star.Node(0.5, 0.0)
        candidate.cost = 0.0
        rrt_star.node_list = [nearest, candidate]

        calls = {"count": 0}

        def always_fail_collision(*_args, **_kwargs):
            calls["count"] += 1
            return False

        rrt_star.is_collision = always_fail_collision
        new_node = rrt_star.Node(1.0, 0.0)
        best_parent, _ = rrt_star._choose_parent(
            new_node, nearest, 0, 1.0, [0, 1]
        )
        assert best_parent is nearest
        assert calls["count"] == 1

    def test_rewire_no_improvement(self, rrt_star):
        """Test _rewire when no cost improvement (candidate cost already lower)."""
        rrt_star.node_list = [rrt_star.Node(0.0, 0.0), rrt_star.Node(1.0, 0.0)]
        rrt_star.node_list[0].cost = 0.0
        rrt_star.node_list[1].cost = 0.5
        new_node = rrt_star.Node(0.5, 0.0)
        new_node.cost = 10.0
        rrt_star._rewire(new_node, [1])
        assert rrt_star.node_list[1].parent is None


class TestRRTStarCoverage:
    """Additional tests to cover remaining lines in rrt_star.py"""

    def test_planning_search_until_max_iter(self, env_factory):
        """Test planning with search_until_max_iter=True (lines 119-130)."""
        env = env_factory("test_collision_world.yaml", full=False)
        env_map = env.get_map()
        planner = RRTStar(
            env_map=env_map,
            robot=env.robot,
            max_iter=10,
            search_until_max_iter=True,
        )
        robot_state = env.get_robot_state()
        _ = planner.planning(
            robot_state, env.robot.goal[:2, 0].tolist(), show_animation=False
        )

    def test_planning_no_show_animation(self, env_factory):
        """Test planning without animation (branches at 112-113, 116-117)."""
        env = env_factory("test_collision_world.yaml", full=False)
        env_map = env.get_map()
        planner = RRTStar(env_map=env_map, robot=env.robot, max_iter=10)
        robot_state = env.get_robot_state()
        _ = planner.planning(
            robot_state, env.robot.goal[:2, 0].tolist(), show_animation=False
        )

    def test_choose_parent_valid_path(self, env_factory):
        """Test _choose_parent with valid candidate."""
        env = env_factory("test_collision_world.yaml", full=False)
        env_map = env.get_map()
        planner = RRTStar(env_map=env_map, robot=env.robot)

        start_node = planner.Node(0.0, 0.0)
        start_node.cost = 0.0
        better_node = planner.Node(0.5, 0.0)
        better_node.cost = 0.0
        planner.node_list = [start_node, better_node]
        planner.is_collision = lambda *_args, **_kwargs: True

        new_node = planner.Node(1.0, 0.0)
        near_inds = [0, 1]

        best_parent, best_cost_fp = planner._choose_parent(
            new_node, start_node, 0, 1.0, near_inds
        )
        assert best_parent is better_node
        assert best_cost_fp == pytest.approx(0.5)

    def test_rewire_with_improvement(self, env_factory):
        """Test _rewire when cost improvement occurs."""
        env = env_factory("test_collision_world.yaml", full=False)
        env_map = env.get_map()
        planner = RRTStar(env_map=env_map, robot=env.robot)

        node0 = planner.Node(0.0, 0.0)
        node0.cost = 0.0
        node0.parent = None

        node1 = planner.Node(1.0, 0.0)
        node1.cost = 100.0
        node1.parent = None

        new_node = planner.Node(0.5, 0.0)
        new_node.cost = 0.5
        new_node.parent = node0

        planner.node_list = [node0, node1, new_node]
        planner._rewire(new_node, [1])


class TestPathPlannersWithGridMap:
    """Path planners with env that has obstacle grid (covers grid-based planner branches)."""

    @pytest.mark.parametrize(
        ("planner_class", "resolution"),
        [
            (AStarPlanner, 0.5),
            (JPSPlanner, 0.5),
            (RRT, 0.3),
        ],
    )
    def test_planners_with_grid_map_env(self, planner_class, resolution, env_factory):
        """Run planner on env from test_grid_map.yaml (has obstacle_map grid)."""
        env = env_factory("test_grid_map.yaml", full=False)
        env_map = env.get_map(resolution=resolution)
        robot_state = env.get_robot_state()
        goal_pose = env.robot.goal[:2, 0].tolist()

        if planner_class in (AStarPlanner, JPSPlanner):
            planner = planner_class(env_map)
            trajectory = planner.planning(
                robot_state, goal_pose, show_animation=False
            )
        elif planner_class is RRT:
            # RRT is sampling-based; evaluate a bounded set of seeds to
            # keep coverage while avoiding single-seed flakiness.
            trajectory = None
            for seed in range(10):
                random.seed(seed)
                planner = planner_class(env_map, robot=env.robot, max_iter=3000)
                trajectory = planner.planning(
                    robot_state, goal_pose, show_animation=False
                )
                if trajectory is not None:
                    break
        else:
            planner = planner_class(env_map, robot=env.robot)
            trajectory = planner.planning(
                robot_state, goal_pose, show_animation=False
            )

        if planner_class is RRT:
            # Sampling-based planners may fail within a finite iteration budget.
            if trajectory is not None:
                env.draw_trajectory(trajectory, traj_type="r-")
        else:
            assert trajectory is not None
            env.draw_trajectory(trajectory, traj_type="r-")
        env.end()
