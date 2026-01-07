"""
Tests for path planning algorithms.

Covers A*, RRT, RRT*, and PRM planners.
"""

import pytest

from irsim.lib.path_planners.a_star import AStarPlanner
from irsim.lib.path_planners.probabilistic_road_map import PRMPlanner
from irsim.lib.path_planners.rrt import RRT
from irsim.lib.path_planners.rrt_star import RRTStar


@pytest.mark.parametrize(
    ("planner_class", "resolution"),
    [
        (AStarPlanner, 0.3),
        (RRTStar, 0.3),
        (RRT, 0.3),
        (PRMPlanner, 0.3),
    ],
)
def test_path_planners(planner_class, resolution, env_factory):
    """Test path planning algorithms find valid paths."""
    env = env_factory("test_collision_world.yaml", full=False)
    env_map = env.get_map()
    planner = planner_class(env_map, resolution)
    robot_info = env.get_robot_info()
    robot_state = env.get_robot_state()
    trajectory = planner.planning(robot_state, robot_info.goal)
    env.draw_trajectory(trajectory, traj_type="r-")
    assert trajectory is not None


class TestRRTStarEdgeMethods:
    """Tests for RRT* edge cases and methods."""

    @pytest.fixture
    def rrt_star(self, env_factory):
        """Create RRT* planner for testing."""
        env = env_factory("test_collision_world.yaml", full=False)
        env_map = env.get_map()
        r = RRTStar(env_map=env_map, robot_radius=0.5)
        r.node_list = []
        r.robot_radius = 0.5
        r.expand_dis = 1.0
        return r

    def test_choose_parent_empty_near_indices(self, rrt_star):
        """Test choose_parent with empty near indices."""
        nn = rrt_star.Node(0.0, 0.0)
        assert rrt_star.choose_parent(nn, []) is None

    def test_choose_parent_all_costs_inf(self, rrt_star):
        """Test choose_parent when all costs are infinite."""
        rrt_star.node_list = [rrt_star.Node(0.0, 0.0)]
        rrt_star.steer = lambda *_args, **_kwargs: None
        rrt_star.check_collision = lambda *_args, **_kwargs: False
        nn = rrt_star.Node(0.0, 0.0)
        assert rrt_star.choose_parent(nn, [0]) is None

    def test_rewire_edge_node_none(self, rrt_star):
        """Test rewire when edge_node is None."""
        rrt_star.node_list = [rrt_star.Node(0.0, 0.0), rrt_star.Node(1.0, 0.0)]
        new_node = rrt_star.Node(0.5, 0.0)
        rrt_star.steer = lambda *_args, **_kwargs: None
        rrt_star.rewire(new_node, [0])

    def test_rewire_no_improvement(self, rrt_star):
        """Test rewire when no cost improvement."""
        rrt_star.node_list = [rrt_star.Node(0.0, 0.0), rrt_star.Node(1.0, 0.0)]
        new_node = rrt_star.Node(0.5, 0.0)
        tmp_edge = rrt_star.Node(0.6, 0.0)
        tmp_edge.cost = 10.0
        rrt_star.steer = lambda *_args, **_kwargs: tmp_edge
        rrt_star.check_collision = lambda *_args, **_kwargs: True
        rrt_star.node_list[0].cost = 0.0
        rrt_star.calc_new_cost = lambda _a, _b: 100.0
        rrt_star.rewire(new_node, [0])

    def test_search_best_goal_node_no_safe_goals(self, rrt_star):
        """Test search_best_goal_node when no safe goals."""
        rrt_star.calc_dist_to_goal = lambda *_args, **_kwargs: 999.0
        rrt_star.steer = lambda *_args, **_kwargs: rrt_star.Node(0.0, 0.0)
        assert rrt_star.search_best_goal_node() is None
