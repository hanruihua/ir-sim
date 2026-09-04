/*
 * ray_casting_omp.c - OpenMP-parallel 2D ray-segment intersection kernel.
 *
 * Compile:
 *   gcc -O3 -march=native -fopenmp -shared -fPIC -o ray_casting_omp.so \
 *       ray_casting_omp.c -lm
 *
 * Called from ray_casting_2d_omp.py via ctypes.  The function signature is
 * a plain-C ABI so no Python headers are required.
 *
 * Algorithm mirrors ray_casting_2d_numba.py:
 *   - Outer loop over N beams is distributed across cores with OpenMP.
 *   - Each thread keeps only two scalars (best_t, best_j) in registers.
 *   - Total memory: O(N) outputs + O(1) per thread -- no M*N temporaries.
 */

#include <math.h>
#include <stdint.h>
#include <float.h>

#define ORIGIN_EPS 1e-9

/*
 * cast_ray_segments_omp
 *
 * Parameters (all arrays are row-major / C order):
 *   origin      - double[2]     ray origin
 *   directions  - double[N*2]   unit beam directions (row = [dx, dy])
 *   seg_start   - double[M*2]   segment start points
 *   seg_end     - double[M*2]   segment end points
 *   N           - int           number of beams
 *   M           - int           number of segments
 *   max_range   - double        miss distance
 *   out_ranges  - double[N]     output: hit distances
 *   out_hit     - int64_t[N]    output: hit segment indices (-1 = miss)
 */
void cast_ray_segments_omp(
    const double *origin,
    const double *directions,
    const double *seg_start,
    const double *seg_end,
    int N, int M,
    double max_range,
    double *out_ranges,
    int64_t *out_hit
) {
    #pragma omp parallel for schedule(dynamic, 32)
    for (int i = 0; i < N; i++) {
        double dx  = directions[2*i];
        double dy  = directions[2*i + 1];
        double pdx = -dy;   /* perpendicular to beam */
        double pdy =  dx;

        double best_t = max_range + 1.0;  /* sentinel */
        int    best_j = -1;

        for (int j = 0; j < M; j++) {
            double svx = seg_end[2*j]     - seg_start[2*j];
            double svy = seg_end[2*j + 1] - seg_start[2*j + 1];
            double sox = origin[0] - seg_start[2*j];
            double soy = origin[1] - seg_start[2*j + 1];

            /* denom = dot(seg_vec, perp_dir) */
            double denom = svx * pdx + svy * pdy;

            if (denom == 0.0) {
                /* Parallel: check collinear overlap */
                double cross = svx * soy - svy * sox;
                if (fabs(cross) <= ORIGIN_EPS) {
                    double ta = (seg_start[2*j]     - origin[0]) * dx
                              + (seg_start[2*j + 1] - origin[1]) * dy;
                    double tb = (seg_end[2*j]        - origin[0]) * dx
                              + (seg_end[2*j + 1]   - origin[1]) * dy;
                    double ov_start = ta < tb ? ta : tb;
                    double ov_end   = ta < tb ? tb : ta;
                    if (ov_end > ORIGIN_EPS && ov_start <= max_range) {
                        double hit_t = ov_start > ORIGIN_EPS
                            ? ov_start
                            : (ov_end <= max_range ? ov_end : max_range);
                        if (hit_t <= max_range && hit_t < best_t) {
                            best_t = hit_t;
                            best_j = j;
                        }
                    }
                }
                continue;
            }

            double u = (sox * pdx + soy * pdy) / denom;
            if (u < 0.0 || u > 1.0) continue;

            double cross = svx * soy - svy * sox;
            double t = cross / denom;
            if (t > ORIGIN_EPS && t <= max_range && t < best_t) {
                best_t = t;
                best_j = j;
            }
        }

        if (best_j >= 0) {
            out_ranges[i] = best_t;
            out_hit[i]    = (int64_t)best_j;
        } else {
            out_ranges[i] = max_range;
            out_hit[i]    = -1;
        }
    }
}
