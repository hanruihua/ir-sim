"""Optional C+OpenMP-accelerated ray-casting kernel.

Drop-in replacement for :func:`~irsim.lib.algorithm.ray_casting_2d.cast_ray_segments`
when a C compiler with OpenMP is available.  If compilation fails the module still
imports cleanly and :func:`cast_ray_segments_omp` falls back to the NumPy kernel.

Build
-----
The first call to :func:`build_omp_lib` (or :func:`ensure_built`) compiles
``ray_casting_omp.c`` next to this file into ``ray_casting_omp.so`` using::

    gcc -O3 -march=native -fopenmp -shared -fPIC

The compiled artefact is cached between Python sessions.  Pass ``force=True``
to recompile.

Usage::

    from irsim.lib.algorithm.ray_casting_2d_omp import (
        cast_ray_segments_omp,
        is_omp_available,
        ensure_built,
    )

    if is_omp_available():
        ensure_built()
        ranges, hits = cast_ray_segments_omp(
            origin, directions, seg_start, seg_end, max_range
        )
"""

from __future__ import annotations

import ctypes
import subprocess
from pathlib import Path

import numpy as np

_HERE = Path(__file__).parent
_C_SRC = _HERE / "ray_casting_omp.c"
_SO_OUT = _HERE / "ray_casting_omp.so"

_lib: ctypes.CDLL | None = None
_OMP_AVAILABLE: bool | None = None  # None = not yet probed


def _try_build(force: bool = False) -> bool:
    """Compile the C source.  Returns True on success."""
    if not force and _SO_OUT.exists():
        return True
    if not _C_SRC.exists():
        return False
    cmd = [
        "gcc",
        "-O3", "-march=native", "-fopenmp",
        "-shared", "-fPIC",
        str(_C_SRC),
        "-o", str(_SO_OUT),
        "-lm",
    ]
    try:
        subprocess.run(cmd, check=True, capture_output=True)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False


def _load_lib() -> ctypes.CDLL | None:
    """Load the compiled shared library and set up argtypes."""
    if not _SO_OUT.exists():
        return None
    try:
        lib = ctypes.CDLL(str(_SO_OUT))
        lib.cast_ray_segments_omp.restype = None
        lib.cast_ray_segments_omp.argtypes = [
            ctypes.POINTER(ctypes.c_double),   # origin
            ctypes.POINTER(ctypes.c_double),   # directions
            ctypes.POINTER(ctypes.c_double),   # seg_start
            ctypes.POINTER(ctypes.c_double),   # seg_end
            ctypes.c_int,                       # N
            ctypes.c_int,                       # M
            ctypes.c_double,                    # max_range
            ctypes.POINTER(ctypes.c_double),   # out_ranges
            ctypes.POINTER(ctypes.c_int64),    # out_hit
        ]
        return lib
    except OSError:
        return None


def build_omp_lib(force: bool = False) -> bool:
    """Build (or rebuild) the OpenMP shared library.

    Args:
        force: Recompile even if the ``.so`` already exists.

    Returns:
        ``True`` if the library is available (built or pre-existing).
    """
    global _lib, _OMP_AVAILABLE
    ok = _try_build(force=force)
    if ok:
        _lib = _load_lib()
        _OMP_AVAILABLE = _lib is not None
    else:
        _OMP_AVAILABLE = False
    return bool(_OMP_AVAILABLE)


def ensure_built() -> bool:
    """Build if not already done.  Returns availability."""
    global _OMP_AVAILABLE
    if _OMP_AVAILABLE is None:
        build_omp_lib()
    return bool(_OMP_AVAILABLE)


def is_omp_available() -> bool:
    """Return ``True`` when the OpenMP kernel is compiled and loaded."""
    if _OMP_AVAILABLE is None:
        ensure_built()
    return bool(_OMP_AVAILABLE)


def _np_ptr(arr: np.ndarray):
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_double))


def _i64_ptr(arr: np.ndarray):
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_int64))


def cast_ray_segments_omp(
    origin: np.ndarray,
    directions: np.ndarray,
    seg_start: np.ndarray,
    seg_end: np.ndarray,
    max_range: float,
) -> tuple[np.ndarray, np.ndarray]:
    """C+OpenMP ray-segment intersection -- same interface as :func:`cast_ray_segments`.

    Falls back to the NumPy kernel when the C library is unavailable.

    Args:
        origin: Shared ray origin ``(2,)``.
        directions: Unit ray directions ``(N, 2)``.
        seg_start: Segment start points ``(M, 2)``.
        seg_end: Segment end points ``(M, 2)``.
        max_range: Maximum ray length; misses return this.

    Returns:
        ``(ranges, hit_index)`` -- same semantics as
        :func:`~irsim.lib.algorithm.ray_casting_2d.cast_ray_segments`.
    """
    if not is_omp_available():
        from irsim.lib.algorithm.ray_casting_2d import cast_ray_segments
        return cast_ray_segments(origin, directions, seg_start, seg_end, max_range)

    if len(seg_start) == 0:
        return (
            np.full(len(directions), max_range, dtype=np.float64),
            np.full(len(directions), -1, dtype=np.int64),
        )

    N = len(directions)
    M = len(seg_start)

    origin_c     = np.ascontiguousarray(origin,     dtype=np.float64)
    directions_c = np.ascontiguousarray(directions, dtype=np.float64)
    seg_start_c  = np.ascontiguousarray(seg_start,  dtype=np.float64)
    seg_end_c    = np.ascontiguousarray(seg_end,    dtype=np.float64)

    out_ranges = np.empty(N, dtype=np.float64)
    out_hit    = np.empty(N, dtype=np.int64)

    _lib.cast_ray_segments_omp(
        _np_ptr(origin_c),
        _np_ptr(directions_c),
        _np_ptr(seg_start_c),
        _np_ptr(seg_end_c),
        ctypes.c_int(N),
        ctypes.c_int(M),
        ctypes.c_double(float(max_range)),
        _np_ptr(out_ranges),
        _i64_ptr(out_hit),
    )
    return out_ranges, out_hit.astype(int)
