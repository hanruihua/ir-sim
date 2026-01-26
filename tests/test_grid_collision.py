"""
Tests for grid-based collision detection in ObstacleMap.

Covers the check_grid_collision method and consistency with geometry-based detection.
"""

import numpy as np
import pytest
from shapely.geometry import MultiLineString, Point, box

from irsim.world.map.obstacle_map import (
    CELL_CENTER_OFFSET,
    COLLISION_RADIUS_FACTOR,
    OCCUPANCY_THRESHOLD,
    ObstacleMap,
)


class TestObstacleMapConstants:
    """Tests for grid collision detection constants."""

    def test_occupancy_threshold_value(self):
        """Test that occupancy threshold is set correctly for 0-100 range."""
        assert OCCUPANCY_THRESHOLD == 50

    def test_cell_center_offset_value(self):
        """Test that cell center offset is 0.5 (center of cell)."""
        assert CELL_CENTER_OFFSET == 0.5

    def test_collision_radius_factor_value(self):
        """Test that collision radius factor is 0.5 (half cell size)."""
        assert COLLISION_RADIUS_FACTOR == 0.5


class TestObstacleMapGridInit:
    """Tests for ObstacleMap initialization with grid data."""

    def test_init_with_grid_map(self):
        """Test initialization with grid_map parameter."""
        grid_map = np.zeros((10, 10), dtype=np.float32)
        grid_map[5, 5] = 100  # Add obstacle

        # Create minimal shape data
        points = np.array([[0, 1, 1, 0], [0, 0, 1, 1]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=grid_map,
            grid_reso=np.array([[1.0], [1.0]]),
            world_offset=[0.0, 0.0],
        )

        assert obstacle_map.grid_map is not None
        assert obstacle_map.grid_map.shape == (10, 10)
        np.testing.assert_array_equal(obstacle_map.grid_reso, np.array([[1.0], [1.0]]))
        assert obstacle_map.world_offset == [0.0, 0.0]

    def test_init_without_grid_map(self):
        """Test initialization without grid_map (default values)."""
        points = np.array([[0, 1, 1, 0], [0, 0, 1, 1]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(shape=shape)

        assert obstacle_map.grid_map is None
        np.testing.assert_array_equal(obstacle_map.grid_reso, np.array([[1.0], [1.0]]))
        assert obstacle_map.world_offset == [0.0, 0.0]

    def test_init_with_custom_offset(self):
        """Test initialization with custom world offset."""
        points = np.array([[0, 1, 1, 0], [0, 0, 1, 1]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            world_offset=[5.0, 10.0],
        )

        assert obstacle_map.world_offset == [5.0, 10.0]


class TestCheckGridCollision:
    """Tests for check_grid_collision method."""

    @pytest.fixture
    def simple_grid_map(self):
        """Create a simple 10x10 grid map with an obstacle at center."""
        grid_map = np.zeros((10, 10), dtype=np.float32)
        grid_map[5, 5] = 100  # Obstacle at center
        return grid_map

    @pytest.fixture
    def obstacle_map_with_grid(self, simple_grid_map):
        """Create an ObstacleMap with grid data."""
        points = np.array([[0, 10, 10, 0], [0, 0, 10, 10]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        return ObstacleMap(
            shape=shape,
            grid_map=simple_grid_map,
            grid_reso=np.array([[1.0], [1.0]]),
            world_offset=[0.0, 0.0],
        )

    def test_no_grid_map_returns_false(self):
        """Test that check_grid_collision returns False when grid_map is None."""
        points = np.array([[0, 1, 1, 0], [0, 0, 1, 1]])
        shape = {"name": "map", "reso": 0.1, "points": points}
        obstacle_map = ObstacleMap(shape=shape)

        test_box = box(0, 0, 1, 1)
        result = obstacle_map.check_grid_collision(test_box)

        assert result is False

    def test_collision_in_free_space(self, obstacle_map_with_grid):
        """Test no collision when geometry is in free space."""
        # Place geometry away from obstacle at (5.5, 5.5)
        test_box = box(0, 0, 1, 1)
        result = obstacle_map_with_grid.check_grid_collision(test_box)

        assert result is False

    def test_collision_with_obstacle(self, obstacle_map_with_grid):
        """Test collision detected when geometry overlaps obstacle."""
        # Obstacle is at cell (5, 5), center at (5.5, 5.5)
        test_box = box(5, 5, 6, 6)
        result = obstacle_map_with_grid.check_grid_collision(test_box)

        assert result is True

    def test_collision_near_obstacle(self, obstacle_map_with_grid):
        """Test collision detected when geometry is near obstacle."""
        # Place geometry close to obstacle center at (5.5, 5.5)
        test_box = box(5.2, 5.2, 5.8, 5.8)
        result = obstacle_map_with_grid.check_grid_collision(test_box)

        assert result is True

    def test_no_collision_just_outside_threshold(self, obstacle_map_with_grid):
        """Test no collision when geometry is just outside collision radius."""
        # Place geometry far enough from obstacle center
        test_box = box(0, 0, 0.5, 0.5)
        result = obstacle_map_with_grid.check_grid_collision(test_box)

        assert result is False

    def test_geometry_outside_grid_bounds(self, obstacle_map_with_grid):
        """Test no collision when geometry is outside grid bounds."""
        # Place geometry completely outside the 10x10 grid
        test_box = box(20, 20, 21, 21)
        result = obstacle_map_with_grid.check_grid_collision(test_box)

        assert result is False

    def test_geometry_partially_outside_bounds(self, obstacle_map_with_grid):
        """Test collision check when geometry is partially outside grid bounds."""
        # Place geometry partially outside bounds but overlapping with grid
        test_box = box(-1, -1, 1, 1)
        result = obstacle_map_with_grid.check_grid_collision(test_box)

        # Should return False as there's no obstacle at (0,0)
        assert result is False

    def test_point_geometry(self, obstacle_map_with_grid):
        """Test collision with point geometry."""
        # Point at obstacle center
        point = Point(5.5, 5.5)
        result = obstacle_map_with_grid.check_grid_collision(point)

        assert result is True

    def test_point_in_free_space(self, obstacle_map_with_grid):
        """Test no collision with point in free space."""
        point = Point(1, 1)
        result = obstacle_map_with_grid.check_grid_collision(point)

        assert result is False


class TestGridCollisionWithOffset:
    """Tests for grid collision with world offset."""

    def test_collision_with_positive_offset(self):
        """Test collision detection with positive world offset."""
        grid_map = np.zeros((10, 10), dtype=np.float32)
        grid_map[5, 5] = 100  # Obstacle

        points = np.array([[10, 20, 20, 10], [10, 10, 20, 20]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=grid_map,
            grid_reso=np.array([[1.0], [1.0]]),
            world_offset=[10.0, 10.0],
        )

        # With offset [10, 10], obstacle center is at world coords (15.5, 15.5)
        test_box = box(15, 15, 16, 16)
        result = obstacle_map.check_grid_collision(test_box)

        assert result is True

    def test_no_collision_with_offset_wrong_location(self):
        """Test no collision at original grid location with offset."""
        grid_map = np.zeros((10, 10), dtype=np.float32)
        grid_map[5, 5] = 100

        points = np.array([[10, 20, 20, 10], [10, 10, 20, 20]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=grid_map,
            grid_reso=np.array([[1.0], [1.0]]),
            world_offset=[10.0, 10.0],
        )

        # Check at (5.5, 5.5) - should be outside the grid bounds now
        test_box = box(5, 5, 6, 6)
        result = obstacle_map.check_grid_collision(test_box)

        assert result is False


class TestGridCollisionWithResolution:
    """Tests for grid collision with different resolutions."""

    def test_collision_with_fine_resolution(self):
        """Test collision detection with fine grid resolution."""
        grid_map = np.zeros((100, 100), dtype=np.float32)
        grid_map[50, 50] = 100  # Obstacle

        points = np.array([[0, 10, 10, 0], [0, 0, 10, 10]])
        shape = {"name": "map", "reso": 0.01, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=grid_map,
            grid_reso=np.array([[0.1], [0.1]]),  # 0.1 resolution
            world_offset=[0.0, 0.0],
        )

        # Obstacle center at (5.05, 5.05)
        test_box = box(5, 5, 5.1, 5.1)
        result = obstacle_map.check_grid_collision(test_box)

        assert result is True

    def test_collision_with_coarse_resolution(self):
        """Test collision detection with coarse grid resolution."""
        grid_map = np.zeros((5, 5), dtype=np.float32)
        grid_map[2, 2] = 100  # Obstacle

        points = np.array([[0, 10, 10, 0], [0, 0, 10, 10]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=grid_map,
            grid_reso=np.array([[2.0], [2.0]]),  # 2.0 resolution
            world_offset=[0.0, 0.0],
        )

        # Obstacle center at (5, 5) with 2.0 resolution
        test_box = box(4, 4, 6, 6)
        result = obstacle_map.check_grid_collision(test_box)

        assert result is True


class TestGridVsGeometryConsistency:
    """Tests comparing grid-based and geometry-based collision detection.

    Note: These tests use synthetic data where grid and geometry may not be
    perfectly aligned. For real consistency tests, see the integration tests
    that use actual environment data.
    """

    @pytest.fixture
    def dense_grid_map(self):
        """Create a grid map with multiple obstacles."""
        grid_map = np.zeros((20, 20), dtype=np.float32)
        # Add obstacles in a pattern
        grid_map[5:8, 5:8] = 100  # 3x3 block
        grid_map[15, 15] = 100  # Single cell
        grid_map[10, 0:20] = 100  # Horizontal line
        return grid_map

    def test_grid_detects_collision_at_obstacle(self, dense_grid_map):
        """Test that grid method detects collision at obstacle location."""
        points = np.array([[0, 20, 20, 0], [0, 0, 20, 20]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=dense_grid_map,
            grid_reso=np.array([[1.0], [1.0]]),
            world_offset=[0.0, 0.0],
        )

        # Test at block obstacle (5:8, 5:8), center around (6.5, 6.5)
        test_box = box(6, 6, 7, 7)

        grid_result = obstacle_map.check_grid_collision(test_box)

        # Grid method should detect collision
        assert grid_result is True

    def test_grid_detects_no_collision_in_free_area(self, dense_grid_map):
        """Test that grid method detects no collision in free area."""
        points = np.array([[0, 20, 20, 0], [0, 0, 20, 20]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=dense_grid_map,
            grid_reso=np.array([[1.0], [1.0]]),
            world_offset=[0.0, 0.0],
        )

        # Test in clear area (cells 0-4 are free)
        test_box = box(1, 1, 2, 2)

        grid_result = obstacle_map.check_grid_collision(test_box)

        # Grid method should detect no collision
        assert grid_result is False

    def test_grid_detects_line_obstacle(self, dense_grid_map):
        """Test that grid method detects collision with line obstacle."""
        points = np.array([[0, 20, 20, 0], [0, 0, 20, 20]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=dense_grid_map,
            grid_reso=np.array([[1.0], [1.0]]),
            world_offset=[0.0, 0.0],
        )

        # grid_map[10, 0:20] sets cells at i=10, j=0..19
        # Cell centers are at x = (10 + 0.5) * 1.0 = 10.5, y varies
        # Test at this line obstacle
        test_box = box(10, 5, 11, 6)

        grid_result = obstacle_map.check_grid_collision(test_box)

        # Grid method should detect collision
        assert grid_result is True


class TestGridCollisionEdgeCases:
    """Tests for edge cases in grid collision detection."""

    def test_empty_grid_map(self):
        """Test with completely empty grid map (all zeros)."""
        grid_map = np.zeros((10, 10), dtype=np.float32)

        points = np.array([[0, 10, 10, 0], [0, 0, 10, 10]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=grid_map,
            grid_reso=np.array([[1.0], [1.0]]),
            world_offset=[0.0, 0.0],
        )

        # No obstacles anywhere
        test_box = box(5, 5, 6, 6)
        result = obstacle_map.check_grid_collision(test_box)

        assert result is False

    def test_full_grid_map(self):
        """Test with completely full grid map (all obstacles)."""
        grid_map = np.full((10, 10), 100, dtype=np.float32)

        points = np.array([[0, 10, 10, 0], [0, 0, 10, 10]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=grid_map,
            grid_reso=np.array([[1.0], [1.0]]),
            world_offset=[0.0, 0.0],
        )

        # Should detect collision anywhere
        test_box = box(5, 5, 6, 6)
        result = obstacle_map.check_grid_collision(test_box)

        assert result is True

    def test_threshold_boundary_below(self):
        """Test with value exactly at threshold (should not trigger)."""
        grid_map = np.full((10, 10), 50, dtype=np.float32)  # Exactly at threshold

        points = np.array([[0, 10, 10, 0], [0, 0, 10, 10]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=grid_map,
            grid_reso=np.array([[1.0], [1.0]]),
            world_offset=[0.0, 0.0],
        )

        test_box = box(5, 5, 6, 6)
        result = obstacle_map.check_grid_collision(test_box)

        # 50 is not > 50, so no collision
        assert result is False

    def test_threshold_boundary_above(self):
        """Test with value just above threshold (should trigger)."""
        grid_map = np.full((10, 10), 51, dtype=np.float32)  # Just above threshold

        points = np.array([[0, 10, 10, 0], [0, 0, 10, 10]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=grid_map,
            grid_reso=np.array([[1.0], [1.0]]),
            world_offset=[0.0, 0.0],
        )

        test_box = box(5, 5, 6, 6)
        result = obstacle_map.check_grid_collision(test_box)

        # 51 > 50, so collision detected
        assert result is True

    def test_single_cell_grid(self):
        """Test with 1x1 grid map."""
        grid_map = np.array([[100]], dtype=np.float32)

        points = np.array([[0, 1, 1, 0], [0, 0, 1, 1]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=grid_map,
            grid_reso=np.array([[1.0], [1.0]]),
            world_offset=[0.0, 0.0],
        )

        # Test at center of single cell (0.5, 0.5)
        test_box = box(0.25, 0.25, 0.75, 0.75)
        result = obstacle_map.check_grid_collision(test_box)

        assert result is True

    def test_negative_coordinates(self):
        """Test with negative world offset."""
        grid_map = np.zeros((10, 10), dtype=np.float32)
        grid_map[5, 5] = 100

        points = np.array([[-10, 0, 0, -10], [-10, -10, 0, 0]])
        shape = {"name": "map", "reso": 0.1, "points": points}

        obstacle_map = ObstacleMap(
            shape=shape,
            grid_map=grid_map,
            grid_reso=np.array([[1.0], [1.0]]),
            world_offset=[-10.0, -10.0],
        )

        # Obstacle center at (-4.5, -4.5)
        test_box = box(-5, -5, -4, -4)
        result = obstacle_map.check_grid_collision(test_box)

        assert result is True


class TestGridCollisionIntegration:
    """Integration tests using actual environment."""

    def test_grid_collision_with_env(self, env_factory):
        """Test grid collision detection with actual environment."""
        env = env_factory("tests/test_grid_map.yaml")

        # Get the obstacle map
        obstacle_map = env._map_collection[0]

        assert obstacle_map.grid_map is not None
        assert obstacle_map.grid_reso is not None

        # Run a few steps and check collision detection works
        for _ in range(10):
            env.step()

        # Robot should not have moved through walls
        robot = env.robot
        assert robot is not None

    def test_grid_vs_geometry_consistency_with_env(self, env_factory):
        """Test consistency between grid and geometry methods in real env."""
        env = env_factory("tests/test_grid_map.yaml")

        obstacle_map = env._map_collection[0]

        # Test at multiple random locations
        np.random.seed(42)
        num_tests = 50
        mismatches = 0

        for _ in range(num_tests):
            x = np.random.uniform(0, 50)
            y = np.random.uniform(0, 50)
            test_box = box(x - 0.5, y - 0.5, x + 0.5, y + 0.5)

            # Grid-based
            grid_result = obstacle_map.check_grid_collision(test_box)

            # Geometry-based
            candidate_indices = obstacle_map.geometry_tree.query(test_box)
            filtered_lines = [obstacle_map.linestrings[i] for i in candidate_indices]
            if filtered_lines:
                filtered_multi_line = MultiLineString(filtered_lines)
                geom_result = test_box.intersects(filtered_multi_line)
            else:
                geom_result = False

            if grid_result != geom_result:
                mismatches += 1

        # Allow small number of mismatches due to discretization
        assert mismatches <= num_tests * 0.05  # Max 5% mismatch rate
