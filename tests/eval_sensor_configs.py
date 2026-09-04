"""Per-sensor configuration performance evaluation.

Loads each sensor YAML from usage/sensor_configs/, extracts beam count and
range_max, then benchmarks the ray-casting kernel (NumPy and Numba JIT) at
three scene densities (M=100, 500, 2000 segments) to evaluate real-time
feasibility for each sensor class.

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

ROOT = Path(__file__).parent.parent
CONFIGS_DIR = ROOT / "usage" / "sensor_configs"
SCENE_DENSITIES = [100, 500, 2000]
N_WARMUP = 3
N_RUNS = 25
SEED = 42


SENSOR_META = {
    "rplidar_a1m8":   {"label": "RPLiDAR A1M8",    "hz": 5.5,  "fov_deg": 360, "class": "Hobbyist"},
    "rplidar_s2":     {"label": "RPLiDAR S2",       "hz": 10.0, "fov_deg": 360, "class": "Mobile robot"},
    "ydlidar_x4":     {"label": "YDLIDAR X4",       "hz": 10.0, "fov_deg": 360, "class": "Budget"},
    "ydlidar_tg15":   {"label": "YDLIDAR TG15",     "hz": 12.0, "fov_deg": 360, "class": "Budget+"},
    "hokuyo_urg04lx": {"label": "Hokuyo URG-04LX",  "hz": 10.0, "fov_deg": 240, "class": "Research"},
    "hokuyo_utm30lx": {"label": "Hokuyo UTM-30LX",  "hz": 40.0, "fov_deg": 270, "class": "Research HF"},
    "sick_tim571":    {"label": "SICK TIM571",       "hz": 15.0, "fov_deg": 270, "class": "Industrial"},
    "sick_lms511":    {"label": "SICK LMS511",       "hz": 25.0, "fov_deg": 190, "class": "Industrial LR"},
}


def load_sensor_params(yaml_path: Path) -> dict:
    """Extract n_beams and range_max from a sensor YAML."""
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


def make_scene(n_beams: int, n_segments: int, range_max: float,
               angle_range: float, seed: int = SEED) -> tuple:
    rng = np.random.default_rng(seed)
    # Beams spanning the FoV
    half = angle_range / 2
    angles = np.linspace(-half, half, n_beams)
    directions = np.column_stack([np.cos(angles), np.sin(angles)])
    origin = np.zeros(2)
    # Random segments inside the sensor's range
    radius = range_max * 0.85
    centers = rng.uniform(-radius, radius, (n_segments, 2))
    ang = rng.uniform(0, math.pi, n_segments)
    L = rng.uniform(0.1, min(2.0, range_max * 0.05), n_segments)
    half_seg = (L[:, None] / 2) * np.column_stack([np.cos(ang), np.sin(ang)])
    return origin, directions, centers - half_seg, centers + half_seg


def bench(fn, args, n_warmup=N_WARMUP, n_runs=N_RUNS) -> dict:
    for _ in range(n_warmup):
        fn(*args)
    times = []
    for _ in range(n_runs):
        t0 = time.perf_counter()
        fn(*args)
        times.append((time.perf_counter() - t0) * 1e3)  # ms
    arr = np.array(times)
    return {
        "mean_ms": float(np.mean(arr)),
        "min_ms":  float(np.min(arr)),
        "p95_ms":  float(np.percentile(arr, 95)),
        "std_ms":  float(np.std(arr)),
    }


def main():
    if is_numba_available():
        print("Warming up Numba JIT...", end=" ", flush=True)
        warmup()
        print("done.\n")
    else:
        print("Numba not available — only NumPy results.\n")

    results = []

    for stem, meta in SENSOR_META.items():
        yaml_path = CONFIGS_DIR / f"{stem}.yaml"
        if not yaml_path.exists():
            print(f"  SKIP {stem} — config file not found")
            continue

        params = load_sensor_params(yaml_path)
        n = params["n_beams"]
        r_max = params["range_max"]
        ang = params["angle_range_rad"]
        hz = params["scan_hz"]
        budget_ms = 1000.0 / hz

        ang_res_deg = math.degrees(ang) / n

        row = {
            "stem": stem,
            "label": meta["label"],
            "class": meta["class"],
            "n_beams": n,
            "range_max": r_max,
            "fov_deg": meta["fov_deg"],
            "ang_res_deg": round(ang_res_deg, 3),
            "scan_hz": hz,
            "budget_ms": round(budget_ms, 2),
            "densities": {},
        }

        for m in SCENE_DENSITIES:
            scene = make_scene(n, m, r_max, ang)
            args = (*scene, r_max)

            np_stats = bench(cast_ray_segments, args)

            density_row = {"n_segments": m, "numpy": np_stats}

            if is_numba_available():
                nb_stats = bench(cast_ray_segments_numba, args)
                speedup = np_stats["mean_ms"] / nb_stats["mean_ms"]
                headroom_np = budget_ms / np_stats["mean_ms"]
                headroom_nb = budget_ms / nb_stats["mean_ms"]
                density_row["numba"] = nb_stats
                density_row["speedup"] = round(speedup, 2)
                density_row["headroom_numpy"] = round(headroom_np, 1)
                density_row["headroom_numba"] = round(headroom_nb, 1)
                density_row["numpy_realtime"] = np_stats["mean_ms"] <= budget_ms
                density_row["numba_realtime"] = nb_stats["mean_ms"] <= budget_ms
            else:
                density_row["headroom_numpy"] = round(budget_ms / np_stats["mean_ms"], 1)
                density_row["numpy_realtime"] = np_stats["mean_ms"] <= budget_ms

            row["densities"][str(m)] = density_row

        results.append(row)

        # Progress print
        print(f"  {meta['label']:<22} N={n:>5}  {hz:>5} Hz  budget={budget_ms:>6.1f}ms")
        for m in SCENE_DENSITIES:
            d = row["densities"][str(m)]
            np_ms = d["numpy"]["mean_ms"]
            nb_str = f"  Numba {d['numba']['mean_ms']:.2f}ms ({d['speedup']:.1f}x)" if "numba" in d else ""
            rt_np = "OK" if d["numpy_realtime"] else "MISS"
            rt_nb = f"/{('OK' if d['numba_realtime'] else 'MISS')}" if "numba" in d else ""
            print(f"    M={m:<5}  NumPy {np_ms:.2f}ms [{rt_np}{rt_nb}]{nb_str}")
        print()

    return results


if __name__ == "__main__":
    results = main()
    out = Path("sensor_eval_results.json")
    out.write_text(json.dumps(results, indent=2))
    print(f"Results saved to {out}")
