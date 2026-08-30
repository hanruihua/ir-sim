import numpy as np

_default = np.random.default_rng()  # shared by environments created without a seed
_env_param = None  # the ``irsim.config.env_param`` proxy, imported on first use


def _params():
    global _env_param
    if _env_param is None:
        from irsim.config import env_param

        _env_param = env_param
    return _env_param


def _generator() -> np.random.Generator:
    """The generator in use: the current environment's own one, else the shared default."""
    return _params().rng or _default


class _RNGProxy:
    """Proxy over the generator in use, so ``rng.uniform(...)`` follows the environment."""

    def __getattr__(self, name):
        return getattr(_generator(), name)


rng = _RNGProxy()


def set_seed(seed: int | None = None) -> None:
    """Reseed the generator in use (see :data:`rng`).

    Reseeds the current environment's own generator when it has one
    (``irsim.make(..., seed=...)`` or ``env.set_random_seed``), otherwise the
    shared default that unseeded environments draw from.

    Args:
        seed: Seed for deterministic sampling. ``None`` creates a fresh
            non-deterministic generator.
    """
    global _default
    params = _params()
    if params.rng is not None:
        params.rng = np.random.default_rng(seed)
    else:
        _default = np.random.default_rng(seed)


def random_uniform(low=None, high=None, size=(3, 1), min_distance=1.0):
    """
    Sample random points uniformly with a pairwise min-distance constraint.

    Args:
        low (list | np.ndarray): Lower bound as a 3D vector (x, y, theta).
            Default is [0.5, 0.5, 0.0].
        high (list | np.ndarray): Upper bound as a 3D vector (x, y, theta).
            Default is [9.5, 9.5, 6.28].
        size (tuple): (dim, n) where dim is 2 or 3 and n is the number of
            points to sample. When dim == 2, only x and y are sampled and
            theta is set to 0. Default is (3, 1).
        min_distance (float): Minimum pairwise distance in the xy plane.
            Default is 1.0.

    Returns:
        np.ndarray: Random points of shape (3, n).
    """
    low = np.asarray([0.5, 0.5, 0.0] if low is None else low, dtype=float).reshape(-1)
    high = np.asarray([9.5, 9.5, 6.28] if high is None else high, dtype=float).reshape(
        -1
    )
    dim, n = size

    points = np.zeros((3, n))
    max_attempts = 1000

    for i in range(n):
        for _ in range(max_attempts):
            candidate = rng.uniform(low[:dim], high[:dim])
            if i == 0:
                break
            diffs = points[:2, :i] - candidate[:2, None]
            if np.min(np.linalg.norm(diffs, axis=0)) >= min_distance:
                break
        points[:dim, i] = candidate

    return points
