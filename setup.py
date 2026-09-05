"""Build the optional C+OpenMP ray-casting extension.

setuptools compiles ``_ray_casting_omp`` on supported platforms.
The extension is *optional*: if it cannot be built (missing compiler,
missing OpenMP), the package still installs cleanly and the Python
fallback in ``ray_casting_2d_omp.py`` is used instead.

Platform flags
--------------
Linux   : gcc  -O3 -march=native -fopenmp
macOS   : clang -Xpreprocessor -fopenmp -I/opt/homebrew/opt/libomp/include
          (falls back to serial -O3 when libomp is not installed)
Windows : cl.exe /O2 /openmp
"""

from __future__ import annotations

import os
import sys

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

# LIBOMP_PREFIX is set by cibuildwheel's environment config to point at a
# conda-forge llvm-openmp install whose macOS deployment target matches the
# wheel target.  Local Homebrew paths are checked as fallbacks.
_LIBOMP_PREFIX = os.environ.get("LIBOMP_PREFIX", "")
_LIBOMP_ROOTS = [
    *([_LIBOMP_PREFIX] if _LIBOMP_PREFIX else []),
    "/opt/homebrew/opt/libomp",  # Apple Silicon Homebrew
    "/usr/local/opt/libomp",  # Intel Homebrew
]


class _OmpBuildExt(build_ext):
    """Inject per-platform OpenMP compiler/linker flags at build time."""

    def build_extension(self, ext: Extension) -> None:
        if ext.name == "irsim.lib.algorithm._ray_casting_omp":
            self._apply_omp_flags(ext)
        try:
            super().build_extension(ext)
        except Exception:
            # optional=True on the Extension lets the overall build continue
            if getattr(ext, "optional", False):
                return
            raise

    def _apply_omp_flags(self, ext: Extension) -> None:
        if sys.platform == "win32":
            ext.extra_compile_args = ["/O2", "/openmp"]
            ext.extra_link_args = []
        elif sys.platform == "darwin":
            root = next((r for r in _LIBOMP_ROOTS if os.path.isdir(r)), None)
            if root:
                ext.extra_compile_args = [
                    "-Xpreprocessor",
                    "-fopenmp",
                    f"-I{root}/include",
                ]
                ext.extra_link_args = [f"-L{root}/lib", "-lomp"]
            else:
                # Build without OpenMP: still correct, just serial
                ext.extra_compile_args = ["-O3"]
                ext.extra_link_args = []
        else:
            # Linux / other POSIX with gcc/clang + libgomp
            ext.extra_compile_args = ["-O3", "-march=native", "-fopenmp"]
            ext.extra_link_args = ["-fopenmp"]


setup(
    ext_modules=[
        Extension(
            "irsim.lib.algorithm._ray_casting_omp",
            # Relative paths required by setuptools; absolute paths are rejected.
            sources=[
                "irsim/lib/algorithm/_ray_casting_omp_module.c",
                "irsim/lib/algorithm/ray_casting_omp.c",
            ],
            optional=True,  # build failure is non-fatal
        )
    ],
    cmdclass={"build_ext": _OmpBuildExt},
)
