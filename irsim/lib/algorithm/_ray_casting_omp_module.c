/*
 * _ray_casting_omp_module.c - Minimal Python extension stub.
 *
 * Both this file and ray_casting_omp.c are compiled together by setuptools
 * so the result is a proper Python extension (.so/.pyd) with the correct
 * platform- and OpenMP-specific flags. ray_casting_2d_omp.py locates the
 * extension via importlib and loads cast_ray_segments_omp through ctypes.
 *
 * No public Python API is exposed; the module exists only to give
 * setuptools a valid PyInit_ entry point for each platform.
 */
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <stdint.h>

/* Forward-declare the kernel compiled from ray_casting_omp.c */
void cast_ray_segments_omp(
    const double *origin,
    const double *directions,
    const double *seg_start,
    const double *seg_end,
    int N, int M,
    double max_range,
    double *out_ranges,
    int64_t *out_hit
);

static struct PyModuleDef _mod = {
    PyModuleDef_HEAD_INIT,
    "_ray_casting_omp",
    "OpenMP ray-casting extension (loaded via ctypes by ray_casting_2d_omp.py).",
    -1,
    NULL
};

PyMODINIT_FUNC
PyInit__ray_casting_omp(void)
{
    return PyModule_Create(&_mod);
}
