"""Correctness and efficiency evaluation of ray-casting backends on
warehouse, factory, and campus maps with dynamic moving objects.

Correctness
-----------
* NumPy vs Numba vs C+OpenMP output must match to floating-point tolerance
  across all (map, density, timestep) combinations.
* Edge-case suite: empty scene, single segment, collinear ray, grazing ray,
  near-origin hit, full-miss scene.

Efficiency
----------
* Simulated N_STEPS timesteps per (map x sensor x backend) with dynamic
  forklifts/AGVs updating segment positions each step.
* Reports mean FPS and per-step kernel latency.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from irsim.lib.algorithm.ray_casting_2d import cast_ray_segments  # noqa: E402
from irsim.lib.algorithm.ray_casting_2d_numba import (  # noqa: E402
    cast_ray_segments_numba,
    is_numba_available,
    warmup,
)
from irsim.lib.algorithm.ray_casting_2d_omp import (  # noqa: E402
    cast_ray_segments_omp,
    is_omp_available,
)
from irsim.lib.algorithm.ray_casting_2d_omp import (  # noqa: E402
    ensure_built as omp_ensure_built,
)
from tests.map_generators import (  # noqa: E402
    DynamicSegments,
    campus,
    factory,
    make_dynamic_objects,
    scene_with_dynamics,
    warehouse,
)

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

N_DYNAMIC = 6  # moving objects per scene
N_STEPS = 60  # timesteps per efficiency run
N_RUNS = 3  # repeated efficiency runs (average)
ATOL = 1e-9  # absolute tolerance for range comparison

SENSOR_BEAMS = {
    "RPLiDAR A1M8": (360, 6.2832, 12.0),
    "YDLIDAR TG15": (2000, 6.2832, 15.0),
    "Hokuyo UTM-30LX": (1080, 4.7124, 30.0),
    "SICK LMS511": (1139, 3.3161, 80.0),
}

MAP_FUNS = {
    "warehouse": warehouse,
    "factory": factory,
    "campus": campus,
}


def _beams(n_beams: int, angle_range: float):
    half = angle_range / 2
    a = np.linspace(-half, half, n_beams)
    return np.column_stack([np.cos(a), np.sin(a)])


# ---------------------------------------------------------------------------
# Correctness edge-case suite
# ---------------------------------------------------------------------------


def _correctness_edge_cases(fns: dict) -> list[dict]:
    """Run hand-crafted edge cases; return list of failure dicts."""
    origin = np.zeros(2)
    dirs = np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]])
    max_r = 10.0
    failures = []

    cases = {
        "empty_scene": (np.empty((0, 2)), np.empty((0, 2))),
        "single_segment_hit": (np.array([[4.9, -0.5]]), np.array([[4.9, 0.5]])),
        "single_segment_miss": (np.array([[4.9, 1.0]]), np.array([[4.9, 2.0]])),
        "collinear_overlap": (np.array([[0.5, 0.0]]), np.array([[3.0, 0.0]])),
        "grazing_parallel": (np.array([[0.0, 0.01]]), np.array([[5.0, 0.01]])),
        "at_max_range": (
            np.array([[max_r - 0.01, -0.5]]),
            np.array([[max_r - 0.01, 0.5]]),
        ),
        "beyond_max_range": (
            np.array([[max_r + 1.0, -0.5]]),
            np.array([[max_r + 1.0, 0.5]]),
        ),
        "multi_segment": (
            np.array([[2.0, -1.0], [5.0, -1.0], [8.0, -1.0]]),
            np.array([[2.0, 1.0], [5.0, 1.0], [8.0, 1.0]]),
        ),
    }

    ref_fn = fns["numpy"]
    for case_name, (ss, se) in cases.items():
        ref_r, ref_hit = ref_fn(origin, dirs, ss, se, max_r)
        for backend, fn in fns.items():
            if backend == "numpy":
                continue
            r, hit = fn(origin, dirs, ss, se, max_r)
            if not np.allclose(r, ref_r, atol=ATOL):
                failures.append(
                    {
                        "case": case_name,
                        "backend": backend,
                        "type": "range",
                        "max_err": float(np.max(np.abs(r - ref_r))),
                    }
                )
            # hit indices only need to agree on miss vs hit (different backends
            # may pick a different segment among ties)
            hit_sign_mismatch = (hit >= 0) != (ref_hit >= 0)
            if hit_sign_mismatch.any():
                failures.append(
                    {
                        "case": case_name,
                        "backend": backend,
                        "type": "hit_index",
                        "count": int(hit_sign_mismatch.sum()),
                    }
                )
    return failures


# ---------------------------------------------------------------------------
# Correctness cross-backend on map data
# ---------------------------------------------------------------------------


def _correctness_maps(fns: dict, max_r: float = 20.0) -> list[dict]:
    """Compare backends on all three map types at a representative density."""
    failures = []
    dirs = _beams(360, 6.2832)
    origin = np.zeros(2)
    for map_name, map_fn in MAP_FUNS.items():
        ss, se = map_fn(max_r)
        ref_r, _ = fns["numpy"](origin, dirs, ss, se, max_r)
        for backend, fn in fns.items():
            if backend == "numpy":
                continue
            r, _ = fn(origin, dirs, ss, se, max_r)
            if not np.allclose(r, ref_r, atol=ATOL):
                failures.append(
                    {
                        "map": map_name,
                        "backend": backend,
                        "type": "range",
                        "max_err": float(np.max(np.abs(r - ref_r))),
                        "n_segments": len(ss),
                    }
                )
    return failures


def _correctness_dynamic(
    fns: dict, max_r: float = 20.0, n_steps: int = 20
) -> list[dict]:
    """Verify consistency across N_STEPS of simulation with dynamic objects."""
    failures = []
    dirs = _beams(360, 6.2832)
    origin = np.zeros(2)
    for map_name, map_fn in MAP_FUNS.items():
        ss_static, se_static = map_fn(max_r, seed=42)
        dyn = make_dynamic_objects(map_name, max_r, n=4, seed=42)
        for step in range(n_steps):
            for d in dyn:
                d.step()
            ss, se = scene_with_dynamics(ss_static, se_static, dyn)
            ref_r, _ = fns["numpy"](origin, dirs, ss, se, max_r)
            for backend, fn in fns.items():
                if backend == "numpy":
                    continue
                r, _ = fn(origin, dirs, ss, se, max_r)
                if not np.allclose(r, ref_r, atol=ATOL):
                    failures.append(
                        {
                            "map": map_name,
                            "backend": backend,
                            "step": step,
                            "type": "dynamic_range",
                            "max_err": float(np.max(np.abs(r - ref_r))),
                        }
                    )
    return failures


# ---------------------------------------------------------------------------
# Efficiency: FPS over N simulation steps
# ---------------------------------------------------------------------------


def _fps_run(fn, origin, dirs, ss_static, se_static, dynamics, max_r):
    """Return mean step latency (ms) over N_STEPS."""
    dyn = [
        DynamicSegments(
            d.seg_start.copy(),
            d.seg_end.copy(),
            d.center.copy(),
            d.vel.copy(),
            d.omega,
            d.bounds,
        )
        for d in dynamics
    ]
    latencies = []
    for _ in range(N_STEPS):
        for d in dyn:
            d.step()
        ss, se = scene_with_dynamics(ss_static, se_static, dyn)
        t0 = time.perf_counter()
        fn(origin, dirs, ss, se, max_r)
        latencies.append((time.perf_counter() - t0) * 1e3)
    return float(np.mean(latencies))


def run_efficiency(fns: dict) -> list[dict]:
    rows = []
    for map_name, map_fn in MAP_FUNS.items():
        for sensor_label, (n_beams, angle_range, max_r) in SENSOR_BEAMS.items():
            ss_static, se_static = map_fn(max_r, seed=7)
            dyn = make_dynamic_objects(map_name, max_r, n=N_DYNAMIC, seed=7)
            dirs = _beams(n_beams, angle_range)
            origin = np.zeros(2)
            n_seg = len(ss_static)

            # warmup each backend
            for fn in fns.values():
                fn(origin, dirs, ss_static, se_static, max_r)

            backend_results = {}
            for backend, fn in fns.items():
                ms_vals = [
                    _fps_run(fn, origin, dirs, ss_static, se_static, dyn, max_r)
                    for _ in range(N_RUNS)
                ]
                mean_ms = float(np.mean(ms_vals))
                backend_results[backend] = {
                    "mean_ms": round(mean_ms, 3),
                    "fps": round(1000.0 / mean_ms, 1),
                }
            # speedups relative to numpy
            np_ms = backend_results["numpy"]["mean_ms"]
            for backend in backend_results:
                if backend != "numpy":
                    backend_results[backend]["speedup"] = round(
                        np_ms / backend_results[backend]["mean_ms"], 2
                    )

            rows.append(
                {
                    "map": map_name,
                    "sensor": sensor_label,
                    "n_beams": n_beams,
                    "n_static": n_seg,
                    "n_dynamic": N_DYNAMIC,
                    "max_r": max_r,
                    **{f"{k}_ms": v["mean_ms"] for k, v in backend_results.items()},
                    **{f"{k}_fps": v["fps"] for k, v in backend_results.items()},
                    **{
                        f"{k}_speedup": v.get("speedup", 1.0)
                        for k, v in backend_results.items()
                    },
                }
            )

            np_ms_s = backend_results["numpy"]["mean_ms"]
            nb_s = backend_results.get("numba", {})
            om_s = backend_results.get("omp", {})
            print(
                f"  [{map_name:9s}] {sensor_label:<22} M={n_seg:<4} "
                f" NP={np_ms_s:.2f}ms"
                + (
                    f"  NB={nb_s['mean_ms']:.2f}ms ({nb_s.get('speedup', 1):.1f}x)"
                    if nb_s
                    else ""
                )
                + (
                    f"  OMP={om_s['mean_ms']:.2f}ms ({om_s.get('speedup', 1):.1f}x)"
                    if om_s
                    else ""
                )
            )
    return rows


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    have_numba = is_numba_available()
    have_omp = is_omp_available()

    if have_numba:
        print("Warming up Numba JIT...", end=" ", flush=True)
        warmup()
        print("done.")
    if have_omp:
        omp_ensure_built()
        print("C+OpenMP kernel: ready.")
    print()

    fns = {"numpy": cast_ray_segments}
    if have_numba:
        fns["numba"] = cast_ray_segments_numba
    if have_omp:
        fns["omp"] = cast_ray_segments_omp

    # ---------- correctness ----------
    print("=== Correctness: edge-case suite ===")
    ec_fails = _correctness_edge_cases(fns)
    if ec_fails:
        print(f"  FAILURES: {len(ec_fails)}")
        for f in ec_fails:
            print(f"    {f}")
    else:
        print("  All edge cases PASSED.\n")

    print("=== Correctness: map data (static) ===")
    map_fails = _correctness_maps(fns)
    if map_fails:
        print(f"  FAILURES: {len(map_fails)}")
        for f in map_fails:
            print(f"    {f}")
    else:
        print("  All map correctness checks PASSED.\n")

    print("=== Correctness: dynamic objects (60 steps x 3 maps) ===")
    dyn_fails = _correctness_dynamic(fns, n_steps=60)
    if dyn_fails:
        print(f"  FAILURES: {len(dyn_fails)}")
        for f in dyn_fails:
            print(f"    {f}")
    else:
        print("  All dynamic correctness checks PASSED.\n")

    # ---------- efficiency ----------
    print("=== Efficiency: FPS simulation (dynamic scene) ===")
    eff_rows = run_efficiency(fns)

    results = {
        "correctness": {
            "edge_case_failures": ec_fails,
            "map_failures": map_fails,
            "dynamic_failures": dyn_fails,
            "edge_cases_passed": len(ec_fails) == 0,
            "map_correctness_passed": len(map_fails) == 0,
            "dynamic_correctness_passed": len(dyn_fails) == 0,
        },
        "efficiency": eff_rows,
    }
    out = ROOT / "map_eval_results.json"
    out.write_text(json.dumps(results, indent=2))
    print(f"\nResults saved to {out}")
    return results


if __name__ == "__main__":
    main()
