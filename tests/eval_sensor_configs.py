"""Per-sensor configuration performance evaluation.

Loads each sensor YAML from usage/sensor_configs/, extracts beam count and
range_max, then benchmarks the ray-casting kernel (NumPy, Numba JIT, C+OpenMP)
against a structured complex scene (indoor rooms + corridors + pillars) at
three densities (M=100, 500, 2000 segments) to evaluate real-time feasibility.

Outputs JSON results and prints a comparison table.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import yaml

from irsim.lib.algorithm.ray_casting_2d import cast_ray_segments
from irsim.lib.algorithm.ray_casting_2d_numba import (
    cast_ray_segments_numba,
    is_numba_available,
    warmup,
)
from irsim.lib.algorithm.ray_casting_2d_omp import (
    cast_ray_segments_omp,
    is_omp_available,
)
from irsim.lib.algorithm.ray_casting_2d_omp import (
    ensure_built as omp_ensure_built,
)

ROOT = Path(__file__).parent.parent
CONFIGS_DIR = ROOT / "usage" / "sensor_configs"
SCENE_DENSITIES = [100, 500, 2000]
N_WARMUP = 3
N_RUNS = 25
SEED = 42


SENSOR_META = {
    "rplidar_a1m8": {
        "label": "RPLiDAR A1M8",
        "hz": 5.5,
        "fov_deg": 360,
        "class": "Hobbyist",
    },
    "rplidar_s2": {
        "label": "RPLiDAR S2",
        "hz": 10.0,
        "fov_deg": 360,
        "class": "Mobile robot",
    },
    "ydlidar_x4": {
        "label": "YDLIDAR X4",
        "hz": 10.0,
        "fov_deg": 360,
        "class": "Budget",
    },
    "ydlidar_tg15": {
        "label": "YDLIDAR TG15",
        "hz": 12.0,
        "fov_deg": 360,
        "class": "Budget+",
    },
    "hokuyo_urg04lx": {
        "label": "Hokuyo URG-04LX",
        "hz": 10.0,
        "fov_deg": 240,
        "class": "Research",
    },
    "hokuyo_utm30lx": {
        "label": "Hokuyo UTM-30LX",
        "hz": 40.0,
        "fov_deg": 270,
        "class": "Research HF",
    },
    "sick_tim571": {
        "label": "SICK TIM571",
        "hz": 15.0,
        "fov_deg": 270,
        "class": "Industrial",
    },
    "sick_lms511": {
        "label": "SICK LMS511",
        "hz": 25.0,
        "fov_deg": 190,
        "class": "Industrial LR",
    },
}


def load_sensor_params(yaml_path: Path) -> dict:
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    robot = cfg.get("robot", [{}])[0]
    sensor = robot.get("sensors", [{}])[0]
    step_time = cfg.get("world", {}).get("step_time", 0.1)
    return {
        "n_beams": int(sensor.get("number", 360)),
        "range_max": float(sensor.get("range_max", 10.0)),
        "range_min": float(sensor.get("range_min", 0.0)),
        "angle_range_rad": float(sensor.get("angle_range", 2 * math.pi)),
        "std": float(sensor.get("std", 0.0)),
        "step_time": float(step_time),
        "scan_hz": round(1.0 / step_time, 1),
    }


def make_scene(
    n_beams: int,
    n_segments: int,
    range_max: float,
    angle_range: float,
    seed: int = SEED,
) -> tuple:
    """Structured complex indoor scene (rooms, corridors, pillars)."""
    from tests.make_complex_scene import make_complex_scene

    origin, directions, seg_start, seg_end = make_complex_scene(
        n_beams=n_beams,
        range_max=range_max,
        angle_range=angle_range,
        seed=seed,
        target_segments=n_segments,
    )
    return origin, directions, seg_start, seg_end


def bench(fn, args, n_warmup=N_WARMUP, n_runs=N_RUNS) -> dict:
    for _ in range(n_warmup):
        fn(*args)
    times = []
    for _ in range(n_runs):
        t0 = time.perf_counter()
        fn(*args)
        times.append((time.perf_counter() - t0) * 1e3)
    arr = np.array(times)
    return {
        "mean_ms": float(np.mean(arr)),
        "min_ms": float(np.min(arr)),
        "p95_ms": float(np.percentile(arr, 95)),
        "std_ms": float(np.std(arr)),
    }


def _rt(ms, budget):
    return ms <= budget


def _headroom(budget, ms):
    return round(budget / ms, 1)


def main():
    print("=== IR-SIM Ray-Casting Sensor Performance Evaluation ===")
    print("Scene type: structured complex (rooms + corridors + pillars)\n")

    have_numba = is_numba_available()
    have_omp = is_omp_available()

    if have_numba:
        print("Warming up Numba JIT...", end=" ", flush=True)
        warmup()
        print("done.")
    else:
        print("Numba not available.")

    if have_omp:
        omp_ensure_built()
        print("C+OpenMP kernel: ready.")
    else:
        print("C+OpenMP kernel: unavailable.")
    print()

    results = []

    for stem, meta in SENSOR_META.items():
        yaml_path = CONFIGS_DIR / f"{stem}.yaml"
        if not yaml_path.exists():
            print(f"  SKIP {stem} -- config file not found")
            continue

        params = load_sensor_params(yaml_path)
        n = params["n_beams"]
        r = params["range_max"]
        ang = params["angle_range_rad"]
        hz = params["scan_hz"]
        budget_ms = 1000.0 / hz
        ang_res_deg = math.degrees(ang) / n

        row = {
            "stem": stem,
            "label": meta["label"],
            "class": meta["class"],
            "n_beams": n,
            "range_max": r,
            "fov_deg": meta["fov_deg"],
            "ang_res_deg": round(ang_res_deg, 3),
            "scan_hz": hz,
            "budget_ms": round(budget_ms, 2),
            "densities": {},
        }

        for m in SCENE_DENSITIES:
            origin, directions, ss, se = make_scene(n, m, r, ang)
            args_np = (origin, directions, ss, se, r)

            np_stats = bench(cast_ray_segments, args_np)
            d = {
                "n_segments": m,
                "actual_segments": len(ss),
                "numpy": np_stats,
                "numpy_realtime": _rt(np_stats["mean_ms"], budget_ms),
                "headroom_numpy": _headroom(budget_ms, np_stats["mean_ms"]),
            }

            if have_numba:
                nb_stats = bench(cast_ray_segments_numba, args_np)
                d["numba"] = nb_stats
                d["numba_realtime"] = _rt(nb_stats["mean_ms"], budget_ms)
                d["headroom_numba"] = _headroom(budget_ms, nb_stats["mean_ms"])
                d["speedup_numba"] = round(np_stats["mean_ms"] / nb_stats["mean_ms"], 2)

            if have_omp:
                omp_stats = bench(cast_ray_segments_omp, args_np)
                d["omp"] = omp_stats
                d["omp_realtime"] = _rt(omp_stats["mean_ms"], budget_ms)
                d["headroom_omp"] = _headroom(budget_ms, omp_stats["mean_ms"])
                d["speedup_omp"] = round(np_stats["mean_ms"] / omp_stats["mean_ms"], 2)

            row["densities"][str(m)] = d

        results.append(row)

        # Progress print
        print(
            f"  {meta['label']:<22} N={n:>5}  {hz:>5} Hz  budget={budget_ms:>6.1f} ms"
        )
        for m in SCENE_DENSITIES:
            d = row["densities"][str(m)]
            np_ms = d["numpy"]["mean_ms"]
            rt_np = "OK" if d["numpy_realtime"] else "MISS"
            parts = [f"M={m:<5}  NumPy {np_ms:.2f} ms [{rt_np}]"]
            if "numba" in d:
                nb_ms = d["numba"]["mean_ms"]
                rt_nb = "OK" if d["numba_realtime"] else "MISS"
                parts.append(
                    f"Numba {nb_ms:.2f} ms ({d['speedup_numba']:.1f}x) [{rt_nb}]"
                )
            if "omp" in d:
                om_ms = d["omp"]["mean_ms"]
                rt_om = "OK" if d["omp_realtime"] else "MISS"
                parts.append(f"OMP {om_ms:.2f} ms ({d['speedup_omp']:.1f}x) [{rt_om}]")
            print("    " + "  |  ".join(parts))
        print()

    return results


if __name__ == "__main__":
    results = main()
    out = Path("sensor_eval_results.json")
    out.write_text(json.dumps(results, indent=2))
    print(f"Results saved to {out}")
