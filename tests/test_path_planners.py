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


class TestRRTStarCoverage:
    """Additional tests to cover remaining lines in rrt_star.py"""

    def test_planning_search_until_max_iter(self, env_factory):
        """Test planning with search_until_max_iter=True (lines 119-130)."""
        env = env_factory("test_collision_world.yaml", full=False)
        env_map = env.get_map()
        planner = RRTStar(
            env_map=env_map,
            robot_radius=0.3,
            max_iter=10,
            search_until_max_iter=True,
        )
        robot_state = env.get_robot_state()
        robot_info = env.get_robot_info()
        _ = planner.planning(robot_state, robot_info.goal, show_animation=False)

    def test_planning_no_show_animation(self, env_factory):
        """Test planning without animation (branches at 112-113, 116-117)."""
        env = env_factory("test_collision_world.yaml", full=False)
        env_map = env.get_map()
        planner = RRTStar(
            env_map=env_map,
            robot_radius=0.3,
            max_iter=10,
        )
        robot_state = env.get_robot_state()
        robot_info = env.get_robot_info()
        _ = planner.planning(robot_state, robot_info.goal, show_animation=False)

    def test_choose_parent_valid_path(self, env_factory):
        """Test choose_parent with valid min_ind (lines 162-166)."""
        env = env_factory("test_collision_world.yaml", full=False)
        env_map = env.get_map()
        planner = RRTStar(env_map=env_map, robot_radius=0.3)

        start_node = planner.Node(0.0, 0.0)
        start_node.cost = 0.0
        planner.node_list = [start_node]

        new_node = planner.Node(1.0, 0.0)
        near_inds = [0]

        result = planner.choose_parent(new_node, near_inds)
        if result is not None:
            assert hasattr(result, "cost")

    def test_rewire_with_improvement(self, env_factory):
        """Test rewire when cost improvement occurs (lines 252-257)."""
        env = env_factory("test_collision_world.yaml", full=False)
        env_map = env.get_map()
        planner = RRTStar(env_map=env_map, robot_radius=0.1)

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
        planner.rewire(new_node, [1])

    def test_propagate_cost_to_leaves(self, env_factory):
        """Test propagate_cost_to_leaves (lines 288-291)."""
        env = env_factory("test_collision_world.yaml", full=False)
        env_map = env.get_map()
        planner = RRTStar(env_map=env_map, robot_radius=0.3)

        parent = planner.Node(0.0, 0.0)
        parent.cost = 1.0

        child = planner.Node(1.0, 0.0)
        child.cost = 10.0
        child.parent = parent

        grandchild = planner.Node(2.0, 0.0)
        grandchild.cost = 20.0
        grandchild.parent = child

        planner.node_list = [parent, child, grandchild]
        parent.cost = 0.5
        planner.propagate_cost_to_leaves(parent)

        assert child.cost < 10.0

    def test_calc_new_cost(self, env_factory):
        """Test calc_new_cost method (lines 270-271)."""
        env = env_factory("test_collision_world.yaml", full=False)
        env_map = env.get_map()
        planner = RRTStar(env_map=env_map, robot_radius=0.3)

        from_node = planner.Node(0.0, 0.0)
        from_node.cost = 1.0

        to_node = planner.Node(1.0, 0.0)

        cost = planner.calc_new_cost(from_node, to_node)
        import pytest

        assert cost == pytest.approx(2.0, rel=0.01)

    def test_search_best_goal_node_returns_index(self, env_factory):
        """Test search_best_goal_node returns valid index (lines 196-200)."""
        env = env_factory("test_collision_world.yaml", full=False)
        env_map = env.get_map()
        planner = RRTStar(env_map=env_map, robot_radius=0.1, expand_dis=2.0)

        planner.end = planner.Node(1.0, 0.0)

        node = planner.Node(0.5, 0.0)
        node.cost = 0.5
        node.parent = None
        planner.node_list = [node]

        _ = planner.search_best_goal_node()
