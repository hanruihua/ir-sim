"""
Tests for irsim.world.map: grid map generators, Map, and obstacle map resolution.

Covers:
- PerlinGridGenerator and generate_perlin_noise
- ImageGridGenerator (image-file based grid loading)
- GridMapGenerator base (grid property, preview)
- resolve_obstacle_map and build_grid_from_generator
- Map (grid_occupied, grid_resolution, is_collision)
- _grid_collision_geometry (grid vs geometry collision)
- FogMap (fog-of-map overlay) and its world/env integration
"""

import os
import runpy
import tempfile
from unittest.mock import patch

import matplotlib.pyplot as plt
import numpy as np
import pytest
from shapely.geometry import MultiLineString, Point, box
from shapely.geometry import Point as ShapelyPoint

from irsim.world import map as world_map_module
from irsim.world.map import (
    FogMap,
    GridMapGenerator,
    ImageGridGenerator,
    Map,
    PerlinGridGenerator,
    build_grid_from_generator,
    resolve_obstacle_map,
)
from irsim.world.map.obstacle_map import (
    CELL_CENTER_OFFSET,
    COLLISION_RADIUS_FACTOR,
    OCCUPANCY_THRESHOLD,
    ObstacleMap,
)
from irsim.world.map.perlin_map_generator import generate_perlin_noise
from irsim.world.world import World


class TestPerlinGridGenerator:
    """Tests for PerlinGridGenerator."""

    def test_perlin_generate_returns_self(self):
        """Test that generate() returns self for chaining."""
        pmap = PerlinGridGenerator(50, 50, seed=42)
        out = pmap.generate()
        assert out is pmap

    def test_perlin_grid_shape(self):
        """Test that PerlinGridGenerator.grid has correct shape."""
        pmap = PerlinGridGenerator(100, 80, seed=42).generate()
        assert pmap.grid.shape == (100, 80)

    def test_perlin_grid_lazy(self):
        """Test that accessing .grid triggers generation if not yet built."""
        pmap = PerlinGridGenerator(50, 50, seed=42)
        assert pmap._grid is None
        grid = pmap.grid
        assert grid.shape == (50, 50)
        assert pmap._grid is not None

    def test_perlin_save_as_image(self):
        """Test that save_as_image creates a file."""
        pmap = PerlinGridGenerator(50, 50, seed=42).generate()
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "perlin2d.png")
            pmap.save_as_image(filepath)
            assert os.path.exists(filepath)
            assert os.path.getsize(filepath) > 0

    def test_perlin_same_params_same_grid(self):
        """Test that same params produce identical grid."""
        pmap1 = PerlinGridGenerator(100, 100, fill=0.38, seed=42).generate()
        pmap2 = PerlinGridGenerator(100, 100, fill=0.38, seed=42).generate()
        np.testing.assert_array_equal(pmap1.grid, pmap2.grid)


class TestPerlinNoiseGeneration:
    """Tests for generate_perlin_noise."""

    def test_generate_noise_shape(self):
        """Test that noise array has correct shape."""
        noise = generate_perlin_noise(100, 80)
        assert noise.shape == (100, 80)

    def test_generate_noise_range(self):
        """Test that noise values are in [0, 1] range."""
        noise = generate_perlin_noise(100, 100, seed=42)
        assert noise.min() >= 0.0
        assert noise.max() <= 1.0

    def test_noise_seed_reproducibility(self):
        """Test that same seed produces identical noise."""
        noise1 = generate_perlin_noise(50, 50, seed=123)
        noise2 = generate_perlin_noise(50, 50, seed=123)
        np.testing.assert_array_equal(noise1, noise2)

    def test_noise_different_seeds(self):
        """Test that different seeds produce different noise."""
        noise1 = generate_perlin_noise(50, 50, seed=1)
        noise2 = generate_perlin_noise(50, 50, seed=2)
        assert not np.allclose(noise1, noise2)

    def test_fractal_zero_raises(self):
        """fractal < 1 raises ValueError (avoids 0/0 in normalization)."""
        with pytest.raises(ValueError, match="fractal must be >= 1"):
            generate_perlin_noise(10, 10, fractal=0)

    def test_attenuation_zero_raises(self):
        """attenuation <= 0 raises ValueError."""
        with pytest.raises(ValueError, match="attenuation must be > 0"):
            generate_perlin_noise(10, 10, attenuation=0)
        with pytest.raises(ValueError, match="attenuation must be > 0"):
            generate_perlin_noise(10, 10, attenuation=-0.1)


class TestPerlinMapGeneration:
    """Tests for PerlinGridGenerator grid generation."""

    def test_generate_map_shape(self):
        """Test that map has correct shape."""
        grid = PerlinGridGenerator(100, 80).generate().grid
        assert grid.shape == (100, 80)

    def test_generate_map_value_range(self):
        """Test that map values are in [0, 100] range."""
        grid = PerlinGridGenerator(100, 100, seed=42).generate().grid
        assert grid.min() >= 0
        assert grid.max() <= 100

    def test_generate_map_binary_like(self):
        """Test that map is binary-like (0 or 100 values only)."""
        grid = PerlinGridGenerator(100, 100, seed=42).generate().grid
        unique_values = np.unique(grid)
        assert len(unique_values) == 2
        assert 0.0 in unique_values
        assert 100.0 in unique_values

    def test_map_seed_reproducibility(self):
        """Test that same seed produces identical maps."""
        grid1 = PerlinGridGenerator(50, 50, seed=42).generate().grid
        grid2 = PerlinGridGenerator(50, 50, seed=42).generate().grid
        np.testing.assert_array_equal(grid1, grid2)


class TestFillParameter:
    """Tests for fill parameter effects."""

    def test_fill_affects_obstacle_ratio(self):
        """Test that higher fill produces more obstacles."""
        grid_low = PerlinGridGenerator(100, 100, fill=0.3, seed=42).generate().grid
        grid_high = PerlinGridGenerator(100, 100, fill=0.7, seed=42).generate().grid

        obstacles_low = np.sum(grid_low > 50)
        obstacles_high = np.sum(grid_high > 50)

        assert obstacles_low < obstacles_high

    def test_fill_matches_obstacle_ratio(self):
        """Test that fill=0.38 produces ~38% obstacles."""
        grid = PerlinGridGenerator(100, 100, fill=0.38, seed=42).generate().grid
        obstacle_ratio = np.sum(grid > 50) / grid.size
        assert 0.36 <= obstacle_ratio <= 0.40

    def test_extreme_fill_high(self):
        """Test high fill creates mostly obstacles."""
        grid = PerlinGridGenerator(100, 100, fill=0.9, seed=42).generate().grid
        obstacle_ratio = np.sum(grid > 50) / grid.size
        assert obstacle_ratio > 0.85

    def test_extreme_fill_low(self):
        """Test low fill creates mostly free space."""
        grid = PerlinGridGenerator(100, 100, fill=0.1, seed=42).generate().grid
        obstacle_ratio = np.sum(grid > 50) / grid.size
        assert obstacle_ratio < 0.15


class TestComplexityParameter:
    """Tests for complexity parameter."""

    def test_complexity_affects_feature_size(self):
        """Test that different complexity produces different patterns."""
        grid_small = (
            PerlinGridGenerator(100, 100, complexity=0.02, seed=42).generate().grid
        )
        grid_large = (
            PerlinGridGenerator(100, 100, complexity=0.2, seed=42).generate().grid
        )

        assert not np.array_equal(grid_small, grid_large)


class TestFractalParameter:
    """Tests for fractal parameter."""

    def test_fractal_affects_detail(self):
        """Test that more fractal layers add more detail."""
        grid_few = PerlinGridGenerator(100, 100, fractal=1, seed=42).generate().grid
        grid_many = PerlinGridGenerator(100, 100, fractal=6, seed=42).generate().grid

        assert not np.array_equal(grid_few, grid_many)


class TestSaveMapAsImage:
    """Tests for save_as_image."""

    def test_save_creates_file(self):
        """Test that save_as_image creates a file."""
        pmap = PerlinGridGenerator(50, 50, seed=42).generate()
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "test_map.png")
            pmap.save_as_image(filepath)
            assert os.path.exists(filepath)

    def test_save_file_not_empty(self):
        """Test that saved file has content."""
        pmap = PerlinGridGenerator(50, 50, seed=42).generate()
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "test_map.png")
            pmap.save_as_image(filepath)
            assert os.path.getsize(filepath) > 0

    def test_save_invert_option(self):
        """Test that invert option works."""
        pmap = PerlinGridGenerator(50, 50, seed=42).generate()
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath_normal = os.path.join(tmpdir, "normal.png")
            filepath_inverted = os.path.join(tmpdir, "inverted.png")
            pmap.save_as_image(filepath_normal, invert=False)
            pmap.save_as_image(filepath_inverted, invert=True)
            assert os.path.getsize(filepath_normal) > 0
            assert os.path.getsize(filepath_inverted) > 0


class TestEdgeCases:
    """Tests for edge cases."""

    def test_small_map(self):
        """Test generation of very small map."""
        grid = PerlinGridGenerator(10, 10, seed=42).generate().grid
        assert grid.shape == (10, 10)

    def test_non_square_map(self):
        """Test generation of non-square map."""
        grid = PerlinGridGenerator(50, 100, seed=42).generate().grid
        assert grid.shape == (50, 100)

    def test_single_fractal(self):
        """Test generation with single fractal layer."""
        grid = PerlinGridGenerator(50, 50, fractal=1, seed=42).generate().grid
        assert grid.shape == (50, 50)

    def test_many_fractal(self):
        """Test generation with many fractal layers."""
        grid = PerlinGridGenerator(50, 50, fractal=8, seed=42).generate().grid
        assert grid.shape == (50, 50)

    def test_fractal_zero_raises_in_generator(self):
        """PerlinGridGenerator with fractal=0 raises at construction."""
        with pytest.raises(ValueError, match="fractal must be >= 1"):
            PerlinGridGenerator(50, 50, fractal=0)

    def test_attenuation_zero_raises_in_generator(self):
        """PerlinGridGenerator with attenuation <= 0 raises at construction."""
        with pytest.raises(ValueError, match="attenuation must be > 0"):
            PerlinGridGenerator(50, 50, attenuation=0)


# ---------------------------------------------------------------------------
# resolve_obstacle_map tests
# ---------------------------------------------------------------------------


class TestResolveObstacleMap:
    """Tests for the central resolve_obstacle_map dispatch function."""

    def test_none_returns_none(self):
        """None input produces None output."""
        assert resolve_obstacle_map(None) is None

    def test_ndarray_passthrough(self):
        """An ndarray is returned as float64."""
        arr = np.ones((10, 10), dtype=np.int32) * 50
        result = resolve_obstacle_map(arr)
        assert result is not None
        assert result.dtype == np.float64
        np.testing.assert_array_equal(result, arr.astype(np.float64))

    def test_dict_spec_perlin(self):
        """A dict with name='perlin' and resolution builds ndarray when world size is passed."""
        spec = {"name": "perlin", "resolution": 1.0, "seed": 7}
        result = resolve_obstacle_map(spec, world_width=30.0, world_height=30.0)
        assert isinstance(result, np.ndarray)
        assert result.shape == (30, 30)
        assert result.dtype == np.float64

    def test_dict_spec_without_world_raises(self):
        """resolve_obstacle_map with generator spec but no world dimensions raises."""
        spec = {"name": "perlin", "resolution": 0.1, "seed": 7}
        with pytest.raises(ValueError, match="world_width"):
            resolve_obstacle_map(spec)

    def test_unknown_input_raises(self):
        """Unrecognised types raise TypeError."""
        with pytest.raises(TypeError, match="obstacle_map must be"):
            resolve_obstacle_map(12345)
        with pytest.raises(TypeError, match="obstacle_map must be"):
            resolve_obstacle_map([1, 2, 3])

    def test_dict_without_name_raises(self):
        """A dict missing the 'name' key raises TypeError."""
        with pytest.raises(TypeError, match="obstacle_map must be"):
            resolve_obstacle_map({"width": 10})

    def test_str_path_treated_as_image(self):
        """A str is treated as image path; returns ndarray when file exists."""
        cave_path = os.path.join(os.path.dirname(__file__), "cave.png")
        result = resolve_obstacle_map(cave_path)
        assert result is not None
        assert isinstance(result, np.ndarray)
        assert result.dtype == np.float64
        assert result.shape[0] > 0
        assert result.shape[1] > 0

    def test_dict_image_missing_path_raises(self):
        """Dict with name='image' but no path raises ValueError."""
        with pytest.raises(ValueError, match="requires 'path'"):
            resolve_obstacle_map({"name": "image"})
        with pytest.raises(ValueError, match="requires 'path'"):
            resolve_obstacle_map({"name": "image", "path": ""})

    def test_dict_image_with_path_returns_ndarray(self):
        """Explicit dict with name='image' and path returns ndarray (ImageGridGenerator)."""
        cave_path = os.path.join(os.path.dirname(__file__), "cave.png")
        result = resolve_obstacle_map({"name": "image", "path": cave_path})
        assert result is not None
        assert isinstance(result, np.ndarray)
        assert result.dtype == np.float64
        assert result.shape[0] > 0
        assert result.shape[1] > 0


# ---------------------------------------------------------------------------
# build_grid_from_generator tests
# ---------------------------------------------------------------------------


class TestBuildGridFromGenerator:
    """Tests for build_grid_from_generator."""

    def test_builds_perlin_grid(self):
        """Correctly builds an ndarray from a perlin spec with resolution and world size."""
        spec = {"name": "perlin", "resolution": 0.1, "seed": 99}
        grid = build_grid_from_generator(spec, world_width=4.0, world_height=4.0)
        assert isinstance(grid, np.ndarray)
        assert grid.shape == (40, 40)
        assert grid.dtype == np.float64

    def test_unknown_name_raises(self):
        """Raises ValueError for an unregistered generator name."""
        with pytest.raises(ValueError, match="Unknown"):
            build_grid_from_generator(
                {"name": "nonexistent_generator", "resolution": 0.1},
                world_width=10.0,
                world_height=10.0,
            )

    def test_missing_name_raises(self):
        """Raises ValueError when name key is absent."""
        with pytest.raises(ValueError, match="Unknown"):
            build_grid_from_generator(
                {"resolution": 0.1}, world_width=10.0, world_height=10.0
            )

    def test_missing_resolution_raises(self):
        """Raises ValueError when resolution is absent in spec."""
        with pytest.raises(ValueError, match="resolution"):
            build_grid_from_generator(
                {"name": "perlin", "seed": 1},
                world_width=10.0,
                world_height=10.0,
            )

    def test_extra_keys_ignored(self):
        """Keys not in yaml_param_names are silently ignored."""
        spec = {
            "name": "perlin",
            "resolution": 0.1,
            "seed": 1,
            "unknown_param": 999,
        }
        grid = build_grid_from_generator(spec, world_width=2.0, world_height=2.0)
        assert grid.shape == (20, 20)

    def test_resolution_computes_grid_from_world_size(self):
        """Grid size = world size / resolution."""
        spec = {
            "name": "perlin",
            "resolution": 0.1,
            "seed": 42,
            "complexity": 0.12,
            "fill": 0.32,
        }
        grid = build_grid_from_generator(spec, world_width=20.0, world_height=15.0)
        assert grid.shape == (200, 150)
        assert grid.dtype == np.float64

    def test_perlin_registered(self):
        """PerlinGridGenerator is auto-registered under 'perlin'."""
        assert "perlin" in GridMapGenerator.registry
        assert GridMapGenerator.registry["perlin"] is PerlinGridGenerator

    def test_image_registered(self):
        """ImageGridGenerator is auto-registered under 'image'."""
        assert "image" in GridMapGenerator.registry
        assert GridMapGenerator.registry["image"] is ImageGridGenerator


# ---------------------------------------------------------------------------
# GridMapGenerator base and ImageGridGenerator tests
# ---------------------------------------------------------------------------


class TestGridMapGeneratorBase:
    """Tests for GridMapGenerator base (grid property, preview)."""

    def test_grid_raises_when_build_returns_none(self):
        """Accessing .grid when _build_grid returns None raises RuntimeError."""

        class FailingGenerator(GridMapGenerator):
            name = "failing"
            yaml_param_names = ()

            def _build_grid(self):
                return None  # type: ignore[return-value]

            def generate(self):
                self._grid = self._build_grid()  # leaves _grid None
                return self

        gen = FailingGenerator()
        with pytest.raises(RuntimeError, match="Grid generation failed"):
            _ = gen.grid

    def test_preview_calls_show(self):
        """preview() calls plt.show() and does not raise."""
        pmap = PerlinGridGenerator(10, 10, seed=42).generate()
        with patch("matplotlib.pyplot.show"):
            pmap.preview(title="Test")
        # No exception; show was called (covered by mock)

    def test_perlin_module_main_runnable(self):
        """Running perlin_map_generator as __main__ does not raise (covers __main__ block)."""
        import warnings

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            with patch("matplotlib.pyplot.show"):
                runpy.run_module(
                    "irsim.world.map.perlin_map_generator",
                    run_name="__main__",
                )


class TestImageGridGenerator:
    """Tests for ImageGridGenerator."""

    def test_load_grayscale_png(self):
        """Load a known grayscale PNG and verify grid properties."""
        # tests/cave.png is 500x500 grayscale
        cave_path = os.path.join(os.path.dirname(__file__), "cave.png")
        gen = ImageGridGenerator(path=cave_path).generate()
        assert gen.grid.shape[0] > 0
        assert gen.grid.shape[1] > 0
        assert gen.grid.dtype == np.float64
        assert gen.grid.min() >= 0
        assert gen.grid.max() <= 100

    def test_missing_file_raises(self):
        """FileNotFoundError raised for a nonexistent image path."""
        gen = ImageGridGenerator(path="this_image_does_not_exist_xyz.png")
        with pytest.raises(FileNotFoundError):
            gen.generate()

    def test_lazy_grid_access(self):
        """Accessing .grid without calling .generate() triggers generation."""
        cave_path = os.path.join(os.path.dirname(__file__), "cave.png")
        gen = ImageGridGenerator(path=cave_path)
        assert gen._grid is None
        grid = gen.grid  # should auto-generate
        assert grid is not None
        assert grid.shape[0] > 0

    def test_load_rgb_image_uses_grayscale(self):
        """RGB image is converted to grayscale (covers _rgb2gray branch)."""
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as f:
            # 5x5 RGB image
            rgb = np.random.randint(0, 255, (5, 5, 3), dtype=np.uint8)
            plt.imsave(f.name, rgb)
            try:
                gen = ImageGridGenerator(path=f.name).generate()
                assert gen.grid.shape == (5, 5)
                assert gen.grid.dtype == np.float64
            finally:
                os.unlink(f.name)

    def test_load_integer_image_normalized(self):
        """Integer dtype image is normalized by iinfo max (covers integer branch)."""
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as f:
            arr = np.random.randint(0, 255, (4, 4), dtype=np.uint8)
            plt.imsave(f.name, arr, cmap="gray", vmin=0, vmax=255)
            try:
                gen = ImageGridGenerator(path=f.name).generate()
                assert gen.grid.dtype == np.float64
                assert gen.grid.min() >= 0
                assert gen.grid.max() <= 100
            finally:
                os.unlink(f.name)

    def test_load_float_max_gt_one_normalized(self):
        """Float image with max > 1 is normalized by max (covers max>1 branch)."""
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as f:
            arr = np.random.rand(4, 4).astype(np.float64) * 10
            plt.imsave(f.name, arr, cmap="gray", vmin=0, vmax=10)
            try:
                gen = ImageGridGenerator(path=f.name).generate()
                assert gen.grid.dtype == np.float64
                assert gen.grid.min() >= 0
                assert gen.grid.max() <= 100
            finally:
                os.unlink(f.name)

    def test_load_float_in_zero_one_passthrough(self):
        """Float image already in [0, 1] is cast to float64 (covers else branch)."""
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as f:
            arr = np.random.rand(4, 4).astype(np.float64)  # max <= 1
            plt.imsave(f.name, arr, cmap="gray", vmin=0, vmax=1)
            try:
                gen = ImageGridGenerator(path=f.name).generate()
                assert gen.grid.dtype == np.float64
                assert gen.grid.min() >= 0
                assert gen.grid.max() <= 100
            finally:
                os.unlink(f.name)


# ---------------------------------------------------------------------------
# RNG isolation tests
# ---------------------------------------------------------------------------


class TestRNGIsolation:
    """Verify that Perlin noise generation does not mutate global RNG."""

    def test_perlin_does_not_affect_global_rng(self):
        """generate_perlin_noise with seed should not alter global RNG state."""
        from irsim.util.random import rng

        # Sample from global rng, then run perlin with a seed, then sample again
        rng.random()
        _ = generate_perlin_noise(30, 30, seed=42)
        rng_state_after = rng.random()

        # If perlin mutated global rng via set_seed, the next sample would be
        # deterministic. Advance rng and run perlin again.
        _ = rng.random()
        _ = generate_perlin_noise(30, 30, seed=42)
        rng_state_after2 = rng.random()

        # Global rng should keep advancing independently (not reset by perlin)
        assert rng_state_after != rng_state_after2


# ---------------------------------------------------------------------------
# Map: grid_occupied, grid_resolution, is_collision
# ---------------------------------------------------------------------------


class TestMapGridOccupiedBoundary:
    """Tests for Map.grid_occupied out-of-bounds behaviour (no false negatives at edges)."""

    def test_upper_boundary_treated_as_occupied(self):
        """Points at or over width/height are occupied so planners cannot escape."""
        # 10x10 world, 10x10 grid -> resolution 1.0; all cells free
        grid = np.zeros((10, 10), dtype=np.float64)
        m = Map(width=10.0, height=10.0, resolution=1.0, obstacle_list=[], grid=grid)
        # Just inside: not occupied
        assert m.grid_occupied(5.0, 5.0) is False
        assert m.grid_occupied(9.5, 9.5) is False
        # At/over upper boundary: must be occupied (was false negative before fix)
        assert m.grid_occupied(10.0, 5.0) is True  # x == width
        assert m.grid_occupied(5.0, 10.0) is True  # y == height
        assert m.grid_occupied(10.0, 10.0) is True
        assert m.grid_occupied(11.0, 5.0) is True
        assert m.grid_occupied(5.0, 11.0) is True

    def test_lower_boundary_treated_as_occupied(self):
        """Negative x/y are occupied."""
        grid = np.zeros((10, 10), dtype=np.float64)
        m = Map(width=10.0, height=10.0, resolution=1.0, obstacle_list=[], grid=grid)
        assert m.grid_occupied(-0.1, 5.0) is True
        assert m.grid_occupied(5.0, -0.1) is True
        assert m.grid_occupied(-1.0, -1.0) is True

    def test_grid_occupied_returns_none_when_no_grid(self):
        """When Map has no grid, grid_occupied returns None."""
        m = Map(width=10.0, height=10.0, resolution=1.0, obstacle_list=[], grid=None)
        assert m.grid_occupied(5.0, 5.0) is None

    def test_grid_occupied_with_margin(self):
        """grid_occupied with margin_x/margin_y expands check region."""
        # One occupied cell at (0,0); point at (1,1) with margin may or may not hit it
        grid = np.zeros((10, 10), dtype=np.float64)
        grid[0, 0] = 100.0
        m = Map(width=10.0, height=10.0, resolution=1.0, obstacle_list=[], grid=grid)
        assert m.grid_occupied(0.5, 0.5) is True
        assert m.grid_occupied(1.5, 1.5, margin_x=1.0, margin_y=1.0) is True
        assert m.grid_occupied(5.0, 5.0, margin_x=0.1, margin_y=0.1) is False

    def test_grid_resolution_none_when_no_grid(self):
        """grid_resolution is None when Map has no grid."""
        m = Map(width=10.0, height=10.0, resolution=1.0, obstacle_list=[], grid=None)
        assert m.grid_resolution is None

    def test_map_resolution_warning_when_diverges_from_grid(self):
        """Map warns when resolution differs from grid cell size by >5%."""
        grid = np.zeros(
            (20, 20), dtype=np.float64
        )  # 20x20 grid, width=10 -> 0.5 per cell
        with pytest.warns(UserWarning, match="resolution.*differs from grid"):
            Map(
                width=10.0,
                height=10.0,
                resolution=0.1,  # very different from 0.5
                obstacle_list=[],
                grid=grid,
            )


class TestMapIsCollision:
    """Tests for Map.is_collision (grid + obstacle_list)."""

    def test_is_collision_grid_occupied_cell(self):
        """is_collision returns True when geometry overlaps occupied grid cell."""
        # 10x10 grid, resolution 1.0, one occupied cell at (0,0)
        grid = np.zeros((10, 10), dtype=np.float64)
        grid[0, 0] = 100.0
        m = Map(
            width=10.0,
            height=10.0,
            resolution=1.0,
            obstacle_list=[],
            grid=grid,
            world_offset=(0.0, 0.0),
        )
        # Point at cell center (0.5, 0.5) is in collision
        pt = ShapelyPoint(0.5, 0.5)
        assert m.is_collision(pt) is True
        # Point far away is free
        pt_free = ShapelyPoint(5.0, 5.0)
        assert m.is_collision(pt_free) is False

    def test_is_collision_out_of_bounds_returns_true(self):
        """is_collision returns True when geometry is outside world bounds."""
        grid = np.zeros((10, 10), dtype=np.float64)
        m = Map(width=10.0, height=10.0, resolution=1.0, obstacle_list=[], grid=grid)
        assert m.is_collision(box(11, 11, 12, 12)) is True
        assert m.is_collision(box(-1, 0, 0, 1)) is True

    def test_is_collision_obstacle_list_only(self):
        """is_collision uses obstacle_list when no grid or grid reports free."""

        # No grid; obstacle_list with one object
        class SimpleObstacle:
            pass

        obj = SimpleObstacle()
        obj._geometry = box(2, 2, 4, 4)
        m = Map(
            width=10.0,
            height=10.0,
            resolution=1.0,
            obstacle_list=[obj],
            grid=None,
        )
        assert m.is_collision(ShapelyPoint(3, 3)) is True
        assert m.is_collision(ShapelyPoint(1, 1)) is False

    def test_grid_collision_geometry_none_grid_returns_false(self):
        """_grid_collision_geometry with grid=None returns False."""
        pt = ShapelyPoint(1, 1)
        out = world_map_module._grid_collision_geometry(
            None, (1.0, 1.0), pt, (0.0, 0.0)
        )
        assert out is False

    def test_grid_collision_geometry_bounds_outside_grid(self):
        """_grid_collision_geometry returns False when geometry bounds outside grid."""
        # Grid 10x10, reso 1.0, offset 0,0. Geometry at 100,100 -> i_min > i_max
        grid = np.zeros((10, 10), dtype=np.float64)
        pt = ShapelyPoint(100, 100)
        out = world_map_module._grid_collision_geometry(
            grid, (1.0, 1.0), pt, (0.0, 0.0)
        )
        assert out is False

    def test_map_world_offset_accepts_list(self):
        """Map accepts world_offset as list and stores as tuple."""
        grid = np.zeros((10, 10), dtype=np.float64)
        m = Map(
            width=10.0,
            height=10.0,
            resolution=1.0,
            obstacle_list=[],
            grid=grid,
            world_offset=[1.0, 2.0],
        )
        assert m.world_offset == (1.0, 2.0)


# ---------------------------------------------------------------------------
# World.gen_grid_map and World.get_map
# ---------------------------------------------------------------------------


class TestWorldGridMap:
    """Tests for World.gen_grid_map with various obstacle_map types."""

    def test_gen_grid_map_none_returns_none_and_zero_reso(self):
        """obstacle_map=None yields (None, None, None) and self.reso zeros."""
        w = World(obstacle_map=None, width=10, height=10)
        assert w.grid_map is None
        assert w.obstacle_index is None
        assert w.obstacle_positions is None
        np.testing.assert_array_equal(w.reso, np.zeros((2, 1)))

    def test_gen_grid_map_ndarray(self):
        """obstacle_map as ndarray produces grid_map and obstacle indices."""
        grid = np.zeros((20, 20), dtype=np.float64)
        grid[5, 5] = 100.0
        w = World(obstacle_map=grid, width=10, height=10)
        assert w.grid_map is not None
        assert w.grid_map.shape == (20, 20)
        assert w.obstacle_index is not None
        assert w.obstacle_positions is not None
        assert w.reso.shape == (2, 1)

    def test_gen_grid_map_dict_perlin(self):
        """obstacle_map as dict name=perlin uses build_grid_from_generator."""
        w = World(
            obstacle_map={"name": "perlin", "resolution": 0.5, "seed": 1},
            width=10,
            height=10,
        )
        assert w.grid_map is not None
        # 10/0.5 = 20 cells per dimension
        assert w.grid_map.shape == (20, 20)
        assert w.obstacle_index is not None

    def test_gen_grid_map_dict_image(self):
        """obstacle_map as dict name=image with path loads image grid."""
        cave_path = os.path.join(os.path.dirname(__file__), "cave.png")
        w = World(
            obstacle_map={"name": "image", "path": cave_path},
            width=50,
            height=50,
        )
        assert w.grid_map is not None
        assert w.grid_map.shape[0] > 0
        assert w.grid_map.shape[1] > 0


class TestWorldGetMap:
    """Tests for World.get_map when grid is not None (resolution branches)."""

    def test_get_map_invalid_resolution_warns_and_uses_grid_resolution(
        self, env_factory
    ):
        """get_map(0) or non-finite resolution warns and uses grid resolution."""
        env = env_factory("test_grid_map.yaml", full=False)
        with pytest.warns(UserWarning, match="resolution must be positive"):
            env_map = env.get_map(0)
        assert env_map is not None
        assert env_map.grid is not None
        env.end()

    def test_get_map_coarser_resolution_downsampled(self, env_factory):
        """get_map with coarser resolution triggers downsampling and warn."""
        env = env_factory("test_grid_map.yaml", full=False)
        # test_grid_map has 50x50 world, cave.png; mdownsample=2 so grid is ~250x250.
        # Request res=2.0 -> grid shape ~ (25, 25)
        with pytest.warns(UserWarning, match="downsampled"):
            env_map = env.get_map(resolution=2.0)
        assert env_map is not None
        assert env_map.grid is not None
        # Coarser than original
        assert env_map.grid.shape[0] <= env._world.grid_map.shape[0]
        assert env_map.grid.shape[1] <= env._world.grid_map.shape[1]
        env.end()

    def test_get_map_finer_resolution_warns(self, env_factory):
        """get_map with finer resolution than grid only warns, no upsampling."""
        env = env_factory("test_grid_map.yaml", full=False)
        with pytest.warns(UserWarning, match="finer than"):
            env_map = env.get_map(resolution=0.01)
        assert env_map is not None
        assert env_map.grid is not None
        # Still uses original grid (no upsampling)
        assert env_map.grid.shape == env._world.grid_map.shape
        env.end()


class TestFogMap:
    """Unit tests for the FogMap fog-of-map overlay."""

    def test_subclasses_map_and_starts_unexplored(self):
        fog = FogMap(width=10, height=10, resolution=0.5, world_offset=(-5, -5))
        assert isinstance(fog, Map)
        assert fog.shape == (20, 20)
        assert fog.explored_ratio == 0.0
        assert not fog.explored.any()

    def test_invalid_resolution_raises(self):
        for bad in (0.0, -0.5, float("inf"), float("nan")):
            with pytest.raises(ValueError, match="positive and finite"):
                FogMap(width=10, height=10, resolution=bad)

    def test_reveal_from_lidar_marks_line_of_sight(self):
        fog = FogMap(width=10, height=10, resolution=0.5, world_offset=(-5, -5))
        # full-circle lidar at the origin, range 3
        angles = np.linspace(-np.pi, np.pi, 16)
        ranges = np.full(16, 3.0)
        fog.reveal_from_lidar([0.0, 0.0, 0.0], angles, ranges)

        assert fog.explored_ratio > 0.0
        # the lidar origin cell is revealed ...
        assert fog.explored[10, 10]
        # ... and a far corner outside lidar range is not.
        assert not fog.explored[0, 0]

    def test_reveal_respects_per_beam_range(self):
        fog = FogMap(width=10, height=10, resolution=0.1, world_offset=(-5, -5))
        # single beam along +x with range 2 -> cells past 2 m stay fogged
        fog.reveal_from_lidar([0.0, 0.0, 0.0], [0.0], [2.0])
        # cell at ~1 m along +x is revealed
        assert fog.explored[int((1.0 + 5) / 0.1), int((0.0 + 5) / 0.1)]
        # cell at ~3 m along +x (beyond range) is not
        assert not fog.explored[int((3.0 + 5) / 0.1), int((0.0 + 5) / 0.1)]

    def test_reset_recovers_fog(self):
        fog = FogMap(width=10, height=10, resolution=0.5)
        fog.reveal_from_lidar([5.0, 5.0, 0.0], [0.0, np.pi], [2.0, 2.0])
        assert fog.explored_ratio > 0.0
        fog.reset()
        assert fog.explored_ratio == 0.0

    def test_to_rgba_transparent_where_explored(self):
        fog = FogMap(width=4, height=4, resolution=1.0, world_offset=(0, 0))
        fog.explored[1, 1] = True
        rgba = fog.to_rgba(color=(0.5, 0.5, 0.5), alpha=1.0)
        assert rgba.shape == (4, 4, 4)
        # explored cell -> fully transparent; fogged cell -> opaque
        assert rgba[1, 1, 3] == pytest.approx(0.0)
        assert rgba[0, 0, 3] == pytest.approx(1.0)

    def test_empty_scan_is_noop(self):
        fog = FogMap(width=4, height=4, resolution=1.0)
        fog.reveal_from_lidar([2.0, 2.0, 0.0], [], [])
        assert fog.explored_ratio == 0.0

    def test_reveal_fov_marks_sector(self):
        fog = FogMap(width=10, height=10, resolution=0.5, world_offset=(-5, -5))
        # 90-degree FOV facing +x (theta=0), radius 3, centred at the origin.
        fog.reveal_fov([0.0, 0.0, 0.0], fov=np.pi / 2, fov_radius=3.0)
        assert fog.explored_ratio > 0.0
        # in front and within the sector -> revealed
        assert fog.explored[int((2.0 + 5) / 0.5), int((0.0 + 5) / 0.5)]
        # behind the heading -> outside the sector -> not revealed
        assert not fog.explored[int((-2.0 + 5) / 0.5), int((0.0 + 5) / 0.5)]
        # beyond fov_radius -> not revealed
        assert not fog.explored[int((4.0 + 5) / 0.5), int((0.0 + 5) / 0.5)]

    def test_reveal_noops_on_degenerate_input(self):
        fog = FogMap(width=10, height=10, resolution=0.5)
        # non-empty beams but all-zero ranges -> max_range <= 0 -> no-op
        fog.reveal_from_lidar([5.0, 5.0, 0.0], [0.0, 1.0], [0.0, 0.0])
        assert fog.explored_ratio == 0.0
        # zero fov or zero radius -> no-op
        fog.reveal_fov([5.0, 5.0, 0.0], 0.0, 3.0)
        fog.reveal_fov([5.0, 5.0, 0.0], np.pi / 2, 0.0)
        assert fog.explored_ratio == 0.0
        # origin far outside the world -> empty bounding box -> no-op
        fog.reveal_fov([100.0, 100.0, 0.0], np.pi / 2, 3.0)
        assert fog.explored_ratio == 0.0

    def test_reveal_fov_wide_sector(self):
        """fov in (pi, 2*pi) uses the exact arctan2 path; only the rear blind
        spot stays fogged."""
        fog = FogMap(width=10, height=10, resolution=0.5, world_offset=(-5, -5))
        fog.reveal_fov([0.0, 0.0, 0.0], fov=1.5 * np.pi, fov_radius=3.0)
        # front (+x, inside the 270-degree sector) revealed
        assert fog.explored[int((2.0 + 5) / 0.5), int((0.0 + 5) / 0.5)]
        # directly behind (-x, in the rear 90-degree blind spot) not revealed
        assert not fog.explored[int((-2.0 + 5) / 0.5), int((0.0 + 5) / 0.5)]

    def test_plot_lifecycle(self):
        """FogMap owns its artist: _init_plot creates it, _step_plot refreshes,
        plot_clear removes it (mirrors ObjectBase)."""
        fog = FogMap(width=10, height=10, resolution=1.0)
        fig, ax = plt.subplots()
        try:
            assert fog._im is None
            fog._init_plot(ax)
            assert fog._im is not None
            artist = fog._im
            fog.reveal_from_lidar([5.0, 5.0, 0.0], [0.0], [3.0])
            fog._step_plot()  # refresh data; should not raise
            # calling _init_plot again reuses the existing artist (no stacking)
            fog._init_plot(ax)
            assert fog._im is artist
            fog.plot_clear()
            assert fog._im is None
        finally:
            plt.close(fig)


class TestFogIntegration:
    """FogMap integration through ``irsim.make`` (enable, reveal, reset)."""

    def _make_fog_env(self, tmp_path, fog=True):
        import irsim

        config = tmp_path / "fog.yaml"
        config.write_text(
            "world:\n"
            "  height: 10\n"
            "  width: 10\n"
            "  step_time: 0.1\n"
            f"  fog_map: {str(fog).lower()}\n"
            "  fog_map_resolution: 0.2\n"
            "robot:\n"
            "  - kinematics: {name: 'omni'}\n"
            "    shape: {name: 'circle', radius: 0.2}\n"
            "    state: [5, 5, 0]\n"
            "    behavior: {name: 'dash'}\n"
            "    goal: [8, 8, 0]\n"
            "    sensors:\n"
            "      - name: 'lidar2d'\n"
            "        range_max: 4.0\n"
            "        angle_range: 6.283\n"
            "        number: 90\n"
            "        noise: false\n"
        )
        return irsim.make(str(config), display=False, save_ani=False)

    def test_fog_enabled_reveals_over_time(self, tmp_path):
        env = self._make_fog_env(tmp_path, fog=True)
        try:
            assert env._world.fog_map is not None
            start = env._world.fog_map.explored_ratio
            for _ in range(15):
                env.step()
                env.render(0.001)
            end = env._world.fog_map.explored_ratio
            # the FogMap created its own overlay artist and the area grew
            assert env._world.fog_map._im is not None
            assert end > start
        finally:
            env.end()

    def test_fog_revealed_by_fov_without_lidar(self, tmp_path):
        """A robot with an `fov` but no lidar reveals fog via its FOV sector."""
        import irsim

        config = tmp_path / "fogfov.yaml"
        config.write_text(
            "world:\n"
            "  height: 12\n"
            "  width: 12\n"
            "  step_time: 0.1\n"
            "  fog_map: true\n"
            "robot:\n"
            "  - kinematics: {name: 'omni'}\n"
            "    shape: {name: 'circle', radius: 0.2}\n"
            "    state: [6, 6, 0]\n"
            "    behavior: {name: 'dash'}\n"
            "    goal: [10, 6, 0]\n"
            "    fov: 1.57\n"
            "    fov_radius: 4.0\n"
        )
        env = irsim.make(str(config), display=False, save_ani=False)
        try:
            assert env.robot_list[0].lidar is None  # no lidar -> FOV reveal
            start = env._world.fog_map.explored_ratio
            for _ in range(15):
                env.step()
                env.render(0.001)
            assert env._world.fog_map.explored_ratio > start
        finally:
            env.end()

    def test_fog_disabled_by_default(self, tmp_path):
        env = self._make_fog_env(tmp_path, fog=False)
        try:
            assert env._world.fog_map is None
            env.step()
            env.render(0.001)
        finally:
            env.end()

    def test_reset_recovers_fog(self, tmp_path):
        env = self._make_fog_env(tmp_path, fog=True)
        try:
            for _ in range(10):
                env.step()
            assert env._world.fog_map.explored_ratio > 0.0
            env.reset()
            assert env._world.fog_map.explored_ratio == 0.0
        finally:
            env.end()

    def test_reset_does_not_leak_imshow_artists(self, tmp_path):
        """Repeated env.reset() must not stack grid/fog imshow artists."""
        import irsim

        config = tmp_path / "fogleak.yaml"
        config.write_text(
            "world:\n"
            "  height: 10\n"
            "  width: 10\n"
            "  step_time: 0.1\n"
            "  fog_map: true\n"
            "  obstacle_map:\n"
            "    name: perlin\n"
            "    resolution: 0.2\n"
            "    fill: 0.2\n"
            "    seed: 7\n"
            "robot:\n"
            "  - kinematics: {name: 'omni'}\n"
            "    state: [2, 2, 0]\n"
            "    behavior: {name: 'dash'}\n"
            "    goal: [8, 8, 0]\n"
            "    sensors:\n"
            "      - name: 'lidar2d'\n"
            "        range_max: 4.0\n"
            "        number: 60\n"
            "        noise: false\n"
        )
        env = irsim.make(str(config), display=False, save_ani=False)
        try:
            ax = env._env_plot.ax
            for _ in range(3):
                env.step()
                env.render(0.001)
            env.reset()
            baseline = len(ax.images)  # grid map + fog overlay
            for _ in range(4):
                for _ in range(3):
                    env.step()
                    env.render(0.001)
                env.reset()
            assert len(ax.images) == baseline
        finally:
            env.end()

    def test_reload_does_not_leak_imshow_artists(self, tmp_path):
        """Repeated env.reload() must not stack imshow artists or open figures.

        Reload rebuilds the world (and FogMap) while reusing the figure/axes, so
        the previous grid/fog overlays must be cleared rather than orphaned.
        """
        import matplotlib.pyplot as plt

        import irsim

        config = tmp_path / "fogreload.yaml"
        config.write_text(
            "world:\n"
            "  height: 10\n"
            "  width: 10\n"
            "  step_time: 0.1\n"
            "  fog_map: true\n"
            "  obstacle_map:\n"
            "    name: perlin\n"
            "    resolution: 0.2\n"
            "    fill: 0.2\n"
            "    seed: 7\n"
            "robot:\n"
            "  - kinematics: {name: 'omni'}\n"
            "    state: [2, 2, 0]\n"
            "    behavior: {name: 'dash'}\n"
            "    goal: [8, 8, 0]\n"
            "    sensors:\n"
            "      - name: 'lidar2d'\n"
            "        range_max: 4.0\n"
            "        number: 60\n"
            "        noise: false\n"
        )
        env = irsim.make(str(config), display=False, save_ani=False)
        try:
            for _ in range(3):
                env.step()
                env.render(0.001)
            env.reload()
            baseline = len(env._env_plot.ax.images)  # grid map + fog overlay
            figs = len(plt.get_fignums())
            for _ in range(3):
                for _ in range(3):
                    env.step()
                    env.render(0.001)
                env.reload()
            assert len(env._env_plot.ax.images) == baseline
            assert len(plt.get_fignums()) == figs
        finally:
            env.end()

    def test_fog_coexists_with_obstacle_map(self, tmp_path):
        """The fog overlay and an obstacle (grid) map exist together; the fog is
        revealed over the underlying map as the robot explores."""
        import irsim

        config = tmp_path / "fog_grid.yaml"
        config.write_text(
            "world:\n"
            "  height: 20\n"
            "  width: 20\n"
            "  step_time: 0.1\n"
            "  fog_map: true\n"
            "  fog_map_resolution: 0.2\n"
            "  obstacle_map:\n"
            "    name: perlin\n"
            "    resolution: 0.2\n"
            "    fill: 0.2\n"
            "    seed: 7\n"
            "robot:\n"
            "  - kinematics: {name: 'omni'}\n"
            "    shape: {name: 'circle', radius: 0.2}\n"
            "    state: [2, 2, 0]\n"
            "    behavior: {name: 'dash'}\n"
            "    goal: [17, 17, 0]\n"
            "    sensors:\n"
            "      - name: 'lidar2d'\n"
            "        range_max: 5.0\n"
            "        angle_range: 6.283\n"
            "        number: 90\n"
            "        noise: false\n"
        )
        env = irsim.make(str(config), display=False, save_ani=False)
        try:
            # both the obstacle grid map and the fog overlay are present
            assert env._world.grid_map is not None
            assert env._world.fog_map is not None
            start = env._world.fog_map.explored_ratio
            for _ in range(15):
                env.step()
                env.render(0.001)
            assert env._world.fog_map.explored_ratio > start
        finally:
            env.end()

    def test_fog_map_resolution_defaults_to_map(self, tmp_path):
        """With no `fog_map_resolution`, the fog grid matches the obstacle map's."""
        import irsim

        config = tmp_path / "fogres.yaml"
        config.write_text(
            "world:\n"
            "  height: 20\n"
            "  width: 20\n"
            "  step_time: 0.1\n"
            "  fog_map: true\n"
            "  obstacle_map:\n"
            "    name: perlin\n"
            "    resolution: 0.2\n"
            "    fill: 0.2\n"
            "    seed: 7\n"
            "robot:\n"
            "  - kinematics: {name: 'omni'}\n"
            "    state: [2, 2, 0]\n"
            "    sensors:\n"
            "      - name: 'lidar2d'\n"
            "        range_max: 5.0\n"
            "        number: 60\n"
            "        noise: false\n"
        )
        env = irsim.make(str(config), display=False, save_ani=False)
        try:
            assert env._world.fog_map.shape == env._world.grid_map.shape
        finally:
            env.end()

    def test_fog_map_resolution_fallback_without_map(self, tmp_path):
        """Without an obstacle map, `fog_map_resolution` falls back to 0.1 m."""
        import irsim

        config = tmp_path / "fognomap.yaml"
        config.write_text(
            "world:\n"
            "  height: 10\n"
            "  width: 10\n"
            "  step_time: 0.1\n"
            "  fog_map: true\n"
            "robot:\n"
            "  - kinematics: {name: 'omni'}\n"
            "    state: [5, 5, 0]\n"
            "    sensors:\n"
            "      - name: 'lidar2d'\n"
            "        range_max: 4.0\n"
            "        number: 60\n"
            "        noise: false\n"
        )
        env = irsim.make(str(config), display=False, save_ani=False)
        try:
            # 10 m / 0.1 m = 100 cells per axis
            assert env._world.fog_map.shape == (100, 100)
        finally:
            env.end()


# ---------------------------------------------------------------------------
# Grid-based collision detection (ObstacleMap.check_grid_collision)
# ---------------------------------------------------------------------------


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


class TestWorldUnknownKwargs:
    """World kwargs validation and param/logger fallbacks."""

    def test_world_unknown_kwarg_warns(self):
        """World.__init__ with an unknown kwarg warns via check_unknown_kwargs."""
        from unittest.mock import MagicMock

        logger = MagicMock()
        with pytest.MonkeyPatch.context() as mp:
            mp.setattr(World, "logger", property(lambda self: logger))
            World(name="test", step_tme=0.05)  # typo: step_tme

        logger.warning.assert_called_once()
        assert "step_tme" in logger.warning.call_args[0][0]

    def test_world_env_param_fallback_to_global(self):
        """World._env_param falls back to the global env_param when _env is None."""
        from irsim.config import env_param as global_ep

        w = World(name="test")
        assert w._env is None
        assert w._env_param is global_ep

    def test_world_env_param_via_env(self):
        """World._env_param delegates to env._env_param when _env is set."""
        from unittest.mock import MagicMock

        w = World(name="test")
        mock_env = MagicMock()
        w._env = mock_env
        assert w._env_param is mock_env._env_param

    def test_world_logger_property(self):
        """World.logger returns the logger from _env_param."""
        w = World(name="test")
        assert w.logger is w._env_param.logger


class TestWorld3D:
    """World3D depth and offset handling."""

    def test_world3d_default_offset(self):
        """Omitting the offset defaults to the origin [0, 0, 0]."""
        from irsim.world.world3d import World3D

        w = World3D(name="w3")
        assert w.offset == [0, 0, 0]

    def test_world3d_offset_2d_padded(self):
        """A 2D [x, y] offset is padded with z=0."""
        from irsim.world.world3d import World3D

        w = World3D(name="w3", offset=[1, 2])
        assert w.offset == [1, 2, 0]
        assert w.z_range == [0, w.depth]

    def test_world3d_offset_3d_kept(self):
        """A full [x, y, z] offset is kept and shifts the z range."""
        from irsim.world.world3d import World3D

        w = World3D(name="w3", depth=5.0, offset=[1, 2, 3])
        assert w.offset == [1, 2, 3]
        assert w.z_range == [3, 8]
