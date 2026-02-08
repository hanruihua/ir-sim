"""
Perlin noise based 2D grid map generator for robot navigation.

Generates procedural occupancy grids using Perlin noise. Parameter semantics
(complexity, fill, fractal, attenuation) follow the map generation used in
GCOPTER (https://github.com/ZJU-FAST-Lab/GCOPTER). Pure NumPy implementation.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .grid_map_generator_base import GridMapGenerator

# Default gradient vectors for 2D Perlin noise
# 8 unit vectors: 4 cardinal + 4 diagonal (pre-normalized to unit length)
_SQRT2_INV = 1.0 / np.sqrt(2)
_GRADIENTS = np.array(
    [
        [1, 0],
        [-1, 0],
        [0, 1],
        [0, -1],
        [_SQRT2_INV, _SQRT2_INV],
        [-_SQRT2_INV, _SQRT2_INV],
        [_SQRT2_INV, -_SQRT2_INV],
        [-_SQRT2_INV, -_SQRT2_INV],
    ],
    dtype=np.float64,
)


def _fade(t: np.ndarray) -> np.ndarray:
    """Compute fade curve for smooth interpolation.

    Uses the improved Perlin noise fade function: 6t^5 - 15t^4 + 10t^3

    Args:
        t (np.ndarray): Input values in range [0, 1].

    Returns:
        np.ndarray: Smoothed values for interpolation.
    """
    return t * t * t * (t * (t * 6 - 15) + 10)


def _lerp(a: np.ndarray, b: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Linear interpolation between two values.

    Args:
        a (np.ndarray): Start values.
        b (np.ndarray): End values.
        t (np.ndarray): Interpolation factor in range [0, 1].

    Returns:
        np.ndarray: Interpolated values.
    """
    return a + t * (b - a)


def _generate_permutation_table(
    rng_instance: np.random.Generator,
    size: int = 256,
) -> np.ndarray:
    """Generate a permutation table for gradient selection.

    Args:
        rng_instance: NumPy random generator (local, not the global one).
        size (int): Size of the permutation table. Default 256.

    Returns:
        np.ndarray: Permutation table of shape (size * 2,).
    """
    perm = np.arange(size, dtype=np.int32)
    rng_instance.shuffle(perm)
    return np.concatenate([perm, perm])


def _perlin_2d(
    x: np.ndarray,
    y: np.ndarray,
    perm: np.ndarray,
) -> np.ndarray:
    """Compute 2D Perlin noise for given coordinates.

    Args:
        x (np.ndarray): X coordinates.
        y (np.ndarray): Y coordinates.
        perm (np.ndarray): Permutation table for gradient selection.

    Returns:
        np.ndarray: Noise values in range approximately [-1, 1].
    """
    # Grid cell coordinates
    xi = x.astype(np.int32) & 255
    yi = y.astype(np.int32) & 255

    # Relative position within cell
    xf = x - np.floor(x)
    yf = y - np.floor(y)

    # Fade curves for interpolation
    u = _fade(xf)
    v = _fade(yf)

    # Hash coordinates of the 4 corners
    aa = perm[perm[xi] + yi]
    ab = perm[perm[xi] + yi + 1]
    ba = perm[perm[xi + 1] + yi]
    bb = perm[perm[xi + 1] + yi + 1]

    # Gradient indices (map to 8 gradient vectors)
    grad_aa = _GRADIENTS[aa % 8]
    grad_ab = _GRADIENTS[ab % 8]
    grad_ba = _GRADIENTS[ba % 8]
    grad_bb = _GRADIENTS[bb % 8]

    # Dot products with distance vectors
    dot_aa = grad_aa[..., 0] * xf + grad_aa[..., 1] * yf
    dot_ab = grad_ab[..., 0] * xf + grad_ab[..., 1] * (yf - 1)
    dot_ba = grad_ba[..., 0] * (xf - 1) + grad_ba[..., 1] * yf
    dot_bb = grad_bb[..., 0] * (xf - 1) + grad_bb[..., 1] * (yf - 1)

    # Bilinear interpolation
    x1 = _lerp(dot_aa, dot_ba, u)
    x2 = _lerp(dot_ab, dot_bb, u)
    return _lerp(x1, x2, v)


def generate_perlin_noise(
    width: int,
    height: int,
    complexity: float = 0.142857,
    fractal: int = 1,
    attenuation: float = 0.5,
    seed: Optional[int] = None,
) -> np.ndarray:
    """Generate 2D Perlin noise array.

    Parameter semantics follow GCOPTER map_gen (see
    https://github.com/ZJU-FAST-Lab/GCOPTER).

    Args:
        width (int): Output width in grid cells.
        height (int): Output height in grid cells.
        complexity (float): Base noise frequency. Default 0.142857.
        fractal (int): Number of octave layers. Default 1. Must be >= 1.
        attenuation (float): Amplitude for layer k is attenuation / (k + 1).
            Default 0.5. Must be > 0.
        seed (int | None): Random seed for reproducibility.

    Returns:
        np.ndarray: Noise array of shape (width, height), values in [0, 1].

    Raises:
        ValueError: If fractal < 1 or attenuation <= 0.
    """
    if fractal < 1:
        raise ValueError("fractal must be >= 1 (got %s)" % fractal)
    if attenuation <= 0:
        raise ValueError("attenuation must be > 0 (got %s)" % attenuation)
    local_rng = np.random.default_rng(seed)
    perm = _generate_permutation_table(local_rng)

    x = np.linspace(0, width, width, endpoint=False)
    y = np.linspace(0, height, height, endpoint=False)
    xx, yy = np.meshgrid(x, y, indexing="ij")

    noise = np.zeros((width, height), dtype=np.float64)
    max_amplitude = 0.0

    for k in range(fractal):
        dfv = 2 ** k
        amplitude = attenuation / (k + 1)
        frequency = complexity * dfv
        noise += amplitude * _perlin_2d(
            xx * frequency, yy * frequency, perm
        )
        max_amplitude += amplitude

    # Normalize to [0, 1]
    noise /= max_amplitude
    noise = (noise + 1) / 2  # Map from [-1, 1] to [0, 1]
    return np.clip(noise, 0, 1)


class PerlinGridGenerator(GridMapGenerator):
    """Perlin noise based 2D occupancy grid map.

    Holds width, height, and a grid of occupancy values (0-100; values > 50
    are obstacles). Parameter semantics follow GCOPTER map_gen
    (https://github.com/ZJU-FAST-Lab/GCOPTER). Output can be saved as PNG
    for use with World.gen_grid_map().
    """

    name = "perlin"
    yaml_param_names = ("complexity", "fill", "fractal", "attenuation", "seed")

    def __init__(
        self,
        width: int,
        height: int,
        complexity: float = 0.142857,
        fill: float = 0.38,
        fractal: int = 1,
        attenuation: float = 0.5,
        seed: Optional[int] = None,
    ) -> None:
        """Initialize map parameters.

        Args:
            width (int): Map width in grid cells.
            height (int): Map height in grid cells.
            complexity (float): Base noise frequency. Default 0.142857.
            fill (float): Obstacle ratio in [0, 1]. Default 0.38.
            fractal (int): Number of octave layers. Default 1. Must be >= 1.
            attenuation (float): Amplitude decay per octave. Default 0.5. Must be > 0.
            seed (int | None): Random seed for reproducibility.
        """
        if fractal < 1:
            raise ValueError("fractal must be >= 1 (got %s)" % fractal)
        if attenuation <= 0:
            raise ValueError("attenuation must be > 0 (got %s)" % attenuation)
        super().__init__()
        self.width = width
        self.height = height
        self.complexity = complexity
        self.fill = fill
        self.fractal = fractal
        self.attenuation = attenuation
        self.seed = seed

    def _build_grid(self) -> np.ndarray:
        """Build the occupancy grid from Perlin noise."""
        noise = generate_perlin_noise(
            width=self.width,
            height=self.height,
            complexity=self.complexity,
            fractal=self.fractal,
            attenuation=self.attenuation,
            seed=self.seed,
        )
        flat = np.sort(noise.ravel())
        idx = int((1.0 - self.fill) * flat.size)
        idx = min(idx, flat.size - 1)
        th = float(flat[idx])
        return np.where(noise > th, 100.0, 0.0).astype(np.float64)

    def preview(
        self,
        title: str = "Perlin 2D Map",
        cmap: str = "gray_r",
    ) -> None:
        """Preview the grid with matplotlib."""
        super().preview(title=title, cmap=cmap)


if __name__ == "__main__":
    pmap = PerlinGridGenerator(
        width=200,
        height=200,
        complexity=0.142857,
        fill=0.38,
        fractal=1,
        attenuation=0.5,
        seed=42,
    ).generate()
    print(f"Generated map shape: {pmap.grid.shape}")
    print(f"Obstacle ratio: {np.sum(pmap.grid > 50) / pmap.grid.size:.2%}")
    pmap.preview()
