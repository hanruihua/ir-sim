from typing import Optional

import numpy as np

_rng = np.random.default_rng()


class _RNGProxy:
    def __getattr__(self, name):
        return getattr(_rng, name)


rng = _RNGProxy()


def set_seed(seed: Optional[int] = None) -> None:
    global _rng
    _rng = (
        np.random.default_rng(seed) if seed is not None else np.random.default_rng()
    )  # for numpy random
